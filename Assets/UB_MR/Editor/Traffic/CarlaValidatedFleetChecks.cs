using System;
using System.IO;
using System.Linq;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace UB_MR.Redis_Networking.Editor
{
    public static class CarlaValidatedFleetChecks
    {
        [MenuItem("UB-MR/Traffic/Check validated CARLA fleet GPU and previews")]
        public static void Run()
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode) throw new InvalidOperationException("Stop Play mode first.");
            if (!TrafficVehicleCatalogValidator.ValidateAll(null)) throw new InvalidOperationException("Invalid traffic catalog.");
            const string output = "Library/CarlaFleetChecks";
            Directory.CreateDirectory(output);
            var fleet = JArray.Parse(File.ReadAllText(CarlaValidatedFleetSetup.Assets + "/fleet.json"));
            var catalog = AssetDatabase.LoadAssetAtPath<TrafficVehicleCatalog>(TrafficVehicleCatalogValidator.CatalogPath);
            var report = new JArray();
            foreach (var spec in fleet)
            {
                string id = (string)spec["blueprintId"];
                if (!catalog.TryResolve(id, out var prefab) || AssetDatabase.GetAssetPath(prefab) !=
                    CarlaValidatedFleetSetup.Prefabs + "/" + spec["name"] + ".prefab")
                    throw new InvalidOperationException("Catalog resolves incorrect model: " + id);
                var preview = new PreviewRenderUtility();
                RenderTexture sdf = null;
                try
                {
                    var root = preview.InstantiatePrefabInScene(prefab);
                    var baker = root.GetComponentInChildren<MeshToSDF>();
                    baker.updateMode = MeshToSDF.UpdateMode.Explicit;
                    var volume = baker.sdfTexture;
                    var resolution = volume.voxelResolution;
                    sdf = new RenderTexture(resolution.x, resolution.y, 0, RenderTextureFormat.RHalf)
                    {
                        dimension = TextureDimension.Tex3D, volumeDepth = resolution.z, enableRandomWrite = true
                    };
                    if (!sdf.Create()) throw new InvalidOperationException("Unable to allocate SDF for " + id);
                    volume.sdf = sdf;
                    using (var cmd = new CommandBuffer())
                    {
                        baker.UpdateSDF(cmd);
                        Graphics.ExecuteCommandBuffer(cmd);
                    }
                    var readback = AsyncGPUReadback.Request(sdf, 0);
                    readback.WaitForCompletion();
                    if (readback.hasError) throw new InvalidOperationException("SDF GPU readback failed: " + id);
                    float min = float.PositiveInfinity, max = float.NegativeInfinity;
                    for (int layer = 0; layer < readback.layerCount; layer++)
                    foreach (ushort bits in readback.GetData<ushort>(layer))
                    {
                        float distance = Mathf.HalfToFloat(bits);
                        if (float.IsNaN(distance) || float.IsInfinity(distance)) throw new InvalidOperationException("Invalid SDF distance: " + id);
                        min = Mathf.Min(min, distance); max = Mathf.Max(max, distance);
                    }
                    if (min >= 0 || max <= 0) throw new InvalidOperationException($"SDF needs inside/outside distances: {id} ({min}, {max})");
                    var appearance = root.GetComponent<TrafficVehicleAppearance>();
                    var authored = appearance.PaintBindings.Select(b => b.renderer.sharedMaterials[b.materialIndex]).ToArray();
                    using (var cache = new TrafficMaterialCache())
                    {
                        appearance.ApplyColor("32,128,224", cache);
                        foreach (var binding in appearance.PaintBindings)
                        {
                            var color = binding.renderer.sharedMaterials[binding.materialIndex].GetColor("_BaseColor");
                            if (Mathf.Abs(color.g - 128f / 255) > 0.001f) throw new InvalidOperationException("Paint binding failed: " + id);
                        }
                        appearance.ReleaseColor();
                        if (cache.Count != 0 || !authored.SequenceEqual(appearance.PaintBindings.Select(b => b.renderer.sharedMaterials[b.materialIndex])))
                            throw new InvalidOperationException("Paint material restoration failed: " + id);
                    }
                    var box = root.GetComponent<BoxCollider>();
                    var center = box.center;
                    float radius = box.size.magnitude * 0.5f;
                    preview.camera.fieldOfView = 32;
                    preview.camera.nearClipPlane = 0.05f;
                    preview.camera.farClipPlane = radius * 20;
                    preview.camera.transform.position = center + new Vector3(1, 0.6f, 1).normalized * radius * 3.5f;
                    preview.camera.transform.LookAt(center);
                    preview.camera.clearFlags = CameraClearFlags.SolidColor;
                    preview.camera.backgroundColor = new Color(0.12f, 0.15f, 0.19f);
                    preview.lights[0].intensity = 1.5f;
                    preview.lights[0].transform.rotation = Quaternion.Euler(45, -30, 0);
                    preview.lights[1].intensity = 1;
                    preview.ambientColor = new Color(0.5f, 0.5f, 0.5f);
                    preview.BeginStaticPreview(new Rect(0, 0, 640, 400));
                    preview.Render(true);
                    var image = preview.EndStaticPreview();
                    try
                    {
                        File.WriteAllBytes(output + "/" + spec["name"] + ".png", image.EncodeToPNG());
                    }
                    finally { UnityEngine.Object.DestroyImmediate(image); }
                    report.Add(new JObject { ["blueprintId"] = id, ["sdfMin"] = min, ["sdfMax"] = max,
                        ["paint"] = "passed", ["gpu"] = "passed", ["preview"] = (string)spec["name"] + ".png" });
                    Debug.Log("CARLA GPU/paint/preview passed: " + id);
                }
                finally
                {
                    preview.Cleanup();
                    if (sdf != null) { sdf.Release(); UnityEngine.Object.DestroyImmediate(sdf); }
                }
            }
            File.WriteAllText(output + "/report.json", report.ToString() + "\n");
            Debug.Log($"CARLA fleet GPU and paint checks PASSED: {report.Count} prefabs.");
        }
    }
}
