using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using CAVAS.UB_MR.DT.Sensors;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEngine;

namespace UB_MR.Redis_Networking.Editor
{
    public static class TrafficVehicleCatalogValidator
    {
        public const string CatalogPath = "Assets/UB_MR/Prefabs/Traffic/TrafficVehicleCatalog.asset";

        [MenuItem("UB-MR/Traffic/Validate catalog")]
        public static void ValidateMenu() => ValidateAll(null);

        [MenuItem("UB-MR/Traffic/Compare server inventory")]
        public static void CompareInventory()
        {
            string path = EditorUtility.OpenFilePanel("CARLA vehicle inventory", "", "json");
            if (!string.IsNullOrEmpty(path)) ValidateAll(path);
        }

        public static void RunBatch()
        {
            try
            {
                string[] args = Environment.GetCommandLineArgs();
                int index = Array.IndexOf(args, "-trafficInventory");
                if (index >= 0 && index + 1 >= args.Length) throw new ArgumentException("-trafficInventory requires a file.");
                EditorApplication.Exit(ValidateAll(index >= 0 ? args[index + 1] : null) ? 0 : 1);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        public static bool ValidateAll(string inventoryPath)
        {
            var errors = new List<string>();
            var catalog = AssetDatabase.LoadAssetAtPath<TrafficVehicleCatalog>(CatalogPath);
            if (catalog == null) errors.Add("Missing traffic vehicle catalog.");
            else
            {
                if (!catalog.Validate(out string error)) errors.Add(error);
                foreach (var entry in catalog.Entries)
                {
                    if (entry.prefab == null) continue;
                    foreach (string issue in ValidatePrefab(entry.prefab)) errors.Add($"{entry.blueprintId}: {issue}");
                }
                if (inventoryPath != null)
                {
                    var inventory = JObject.Parse(File.ReadAllText(inventoryPath));
                    var blueprints = inventory["blueprints"] as JArray;
                    if (blueprints == null || blueprints.Any(b => b.Type != JTokenType.String))
                        throw new FormatException("Inventory requires a blueprints array of strings.");
                    var ids = new HashSet<string>(catalog.Entries.Select(e => e.blueprintId), StringComparer.Ordinal);
                    string[] missing = blueprints.Values<string>().Where(id => !ids.Contains(id)).ToArray();
                    Debug.Log($"CARLA {inventory["server_version"]}: {blueprints.Count - missing.Length}/{blueprints.Count} vehicle blueprints registered.");
                    if (missing.Length > 0) Debug.LogWarning("Unmapped fleet blueprints (fallback): " + string.Join(", ", missing));
                }
            }
            foreach (string error in errors) Debug.LogError("Traffic catalog: " + error);
            if (errors.Count == 0) Debug.Log("Traffic catalog validation PASSED.");
            return errors.Count == 0;
        }

        public static List<string> ValidatePrefab(GameObject root)
        {
            var errors = new List<string>();
            if (!root.activeSelf || root.transform.localPosition != Vector3.zero
                || Quaternion.Angle(root.transform.localRotation, Quaternion.identity) > 0.01f
                || root.transform.localScale != Vector3.one) errors.Add("Root must be active with an identity transform.");
            var alignment = root.transform.Find("Alignment");
            if (alignment == null) errors.Add("Missing shared Alignment transform.");
            var appearance = root.GetComponent<TrafficVehicleAppearance>();
            if (appearance == null) errors.Add("Missing root TrafficVehicleAppearance.");
            else if (!appearance.Validate(out string paintError)) errors.Add(paintError);
            var objects = root.GetComponentsInChildren<VirtualObject>(true);
            if (objects.Length != 1) errors.Add("Exactly one VirtualObject is required.");
            if (root.GetComponentsInChildren<Rigidbody>(true).Any(r => !r.isKinematic))
                errors.Add("Traffic must not have dynamic rigidbodies.");
            if (objects.Length != 1) return errors;
            var settings = new SerializedObject(objects[0]);
            var box = settings.FindProperty("boundingBox").objectReferenceValue as BoxCollider;
            var baker = settings.FindProperty("meshToSDF").objectReferenceValue as MeshToSDF;
            if (box == null || !box.enabled || !box.isTrigger || !box.gameObject.activeSelf
                || !box.transform.IsChildOf(root.transform) || !Positive(box.size)) errors.Add("Invalid trigger bounding box.");
            if (!Enum.IsDefined(typeof(VirtualObjectClassification), objects[0].Classification)
                || objects[0].Classification == VirtualObjectClassification.Unknown) errors.Add("Set a vehicle classification.");
            if (baker == null || baker.sdfTexture == null || !baker.enabled || !baker.gameObject.activeSelf)
            { errors.Add("Missing or disabled SDF baker/volume."); return errors; }
            if (!baker.transform.IsChildOf(root.transform) || !baker.sdfTexture.transform.IsChildOf(root.transform))
                errors.Add("SDF baker and volume must belong to this prefab.");
            if (root.GetComponentsInChildren<MeshToSDF>(true).Length != 1
                || root.GetComponentsInChildren<SDFTexture>(true).Length != 1)
                errors.Add("Exactly one SDF baker and volume are required.");
            if (new SerializedObject(baker).FindProperty("m_Compute").objectReferenceValue == null)
                errors.Add("Missing SDF compute shader.");
            var proxy = baker.GetComponent<MeshFilter>();
            if (proxy == null || proxy.sharedMesh == null) { errors.Add("SDF baker requires a proxy mesh."); return errors; }
            if (!proxy.sharedMesh.isReadable) errors.Add("Enable Read/Write on the SDF proxy: player GPU buffer recreation requires it.");
            if (alignment != null && !proxy.transform.IsChildOf(alignment)) errors.Add("Proxy must use the shared Alignment transform.");
            if (baker.GetComponentsInChildren<Renderer>(true).Any(r => r.enabled)) errors.Add("Proxy renderers must be disabled.");
            foreach (var renderer in root.GetComponentsInChildren<Renderer>(true).Where(r => r.enabled))
                if (alignment != null && !renderer.transform.IsChildOf(alignment)) errors.Add("Visible geometry must use the shared Alignment transform.");
            var volume = baker.sdfTexture;
            var size = settings.FindProperty("sdfTextureSize").vector3Value;
            int resolution = settings.FindProperty("sdfTextureResolution").intValue;
            if (!Positive(size) || resolution < 3) { errors.Add("Invalid SDF size/resolution."); return errors; }
            if ((volume.size - size).sqrMagnitude > 0.000001f || volume.resolution != resolution)
                errors.Add("VirtualObject and SDFTexture size/resolution must agree.");
            // Same quantization as SDFTexture.voxelBounds in Dynamic mode, without allocating GPU resources in the Editor.
            float voxel = size.x / resolution;
            var effective = new Vector3(resolution, Mathf.Clamp((int)(resolution * size.y / size.x), 1, 2048),
                Mathf.Clamp((int)(resolution * size.z / size.x), 1, 2048)) * voxel;
            var proxyBounds = MeshBoundsIn(proxy, volume.transform);
            var paddedExtent = effective * 0.5f - Vector3.one * voxel;
            for (int axis = 0; axis < 3; axis++)
                if (proxyBounds.min[axis] < -paddedExtent[axis] - 0.001f || proxyBounds.max[axis] > paddedExtent[axis] + 0.001f)
                    errors.Add($"Quantized SDF volume lacks one voxel of proxy padding on axis {axis}.");
            return errors;
        }

        internal static Bounds MeshBoundsIn(MeshFilter mesh, Transform reference)
        {
            Bounds source = mesh.sharedMesh.bounds;
            Matrix4x4 matrix = reference.worldToLocalMatrix * mesh.transform.localToWorldMatrix;
            var result = new Bounds(matrix.MultiplyPoint3x4(source.center), Vector3.zero);
            for (int i = 0; i < 8; i++)
                result.Encapsulate(matrix.MultiplyPoint3x4(source.center + Vector3.Scale(source.extents,
                    new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1))));
            return result;
        }

        private static bool Positive(Vector3 v) => v.x > 0 && v.y > 0 && v.z > 0
            && !float.IsInfinity(v.x) && !float.IsInfinity(v.y) && !float.IsInfinity(v.z);
    }
}
