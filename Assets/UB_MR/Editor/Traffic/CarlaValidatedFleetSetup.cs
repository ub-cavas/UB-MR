using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using CAVAS.UB_MR.DT.Sensors;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace UB_MR.Redis_Networking.Editor
{
    /// <summary>Builds the validated CARLA exports without modifying an open scene.</summary>
    public static class CarlaValidatedFleetSetup
    {
        public const string Assets = "Assets/UB_MR_Assets/Actors/Vehicles/CARLA_Validated";
        public const string Prefabs = "Assets/UB_MR/Prefabs/Traffic/CARLA_Validated";

        [MenuItem("UB-MR/Traffic/Rebuild validated CARLA fleet")]
        public static void Build() => Build(false);

        [MenuItem("UB-MR/Traffic/Import only new validated CARLA vehicles")]
        public static void ImportNew() => Build(true);

        public static void ImportNewBatch()
        {
            try { ImportNew(); EditorApplication.Exit(0); }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        private static void Build(bool onlyNew)
        {
            if (EditorApplication.isPlayingOrWillChangePlaymode)
                throw new InvalidOperationException("Stop Play mode before rebuilding traffic assets.");
            var fleet = JArray.Parse(File.ReadAllText(Assets + "/fleet.json"));
            var catalog = AssetDatabase.LoadAssetAtPath<TrafficVehicleCatalog>(TrafficVehicleCatalogValidator.CatalogPath);
            if (catalog == null) throw new InvalidOperationException("Missing traffic catalog.");
            if (!catalog.Validate(out string catalogError)) throw new InvalidOperationException(catalogError);
            if (onlyNew)
            {
                var existingIds = new HashSet<string>(catalog.Entries.Select(e => e.blueprintId), StringComparer.Ordinal);
                fleet = new JArray(fleet.Where(s => RegistersInCatalog(s)
                    ? !existingIds.Contains((string)s["blueprintId"])
                    : !File.Exists(Prefabs + "/" + s["name"] + ".prefab")));
                // Preflight every destination before modifying any model importer or material.
                var ids = new HashSet<string>(StringComparer.Ordinal);
                var names = new HashSet<string>(StringComparer.Ordinal);
                var folders = new HashSet<string>(StringComparer.Ordinal);
                foreach (var spec in fleet)
                {
                    string path = Prefabs + "/" + spec["name"] + ".prefab";
                    if ((RegistersInCatalog(spec) && !ids.Add((string)spec["blueprintId"])) || !names.Add((string)spec["name"])
                        || !folders.Add((string)spec["folder"]) || File.Exists(path)
                        || Directory.Exists(Assets + "/" + spec["folder"] + "/Materials"))
                        throw new InvalidOperationException("New vehicle conflicts with existing generated assets: " + path);
                }
                if (fleet.Count == 0) { Debug.Log("No new CARLA vehicles to import; existing assets unchanged."); return; }
            }
            Directory.CreateDirectory(Prefabs);
            AssetDatabase.Refresh();
            var registrations = new List<(string id, GameObject prefab)>();
            var report = onlyNew && File.Exists(Assets + "/import_report.json")
                ? JArray.Parse(File.ReadAllText(Assets + "/import_report.json")) : new JArray();
            foreach (JObject spec in fleet)
            {
                string folder = (string)spec["folder"];
                string directory = Assets + "/" + folder;
                string visualPath = directory + "/" + folder + "_Textured.fbx";
                string proxyPath = directory + "/" + folder + "_SDF_Proxy.fbx";
                ConfigureModel(visualPath, false);
                ConfigureModel(proxyPath, true);
                var materials = BuildMaterials(spec, directory, visualPath);
                GameObject prefab = BuildPrefab(spec, directory, visualPath, proxyPath, materials);
                var errors = TrafficVehicleCatalogValidator.ValidatePrefab(prefab);
                if (errors.Count != 0) throw new InvalidOperationException(folder + ": " + string.Join("; ", errors));
                if (RegistersInCatalog(spec)) registrations.Add(((string)spec["blueprintId"], prefab));
                var box = prefab.GetComponent<BoxCollider>();
                var result = new JObject
                {
                    ["blueprintId"] = (string)spec["blueprintId"],
                    ["prefab"] = AssetDatabase.GetAssetPath(prefab),
                    ["classification"] = (string)spec["classification"],
                    ["size_m"] = new JArray(box.size.x, box.size.y, box.size.z),
                    ["paintBindings"] = prefab.GetComponent<TrafficVehicleAppearance>().PaintBindings.Count,
                    ["validation"] = "passed"
                };
                if (!RegistersInCatalog(spec)) result["registerInCatalog"] = false;
                report.Add(result);
                Debug.Log("CARLA fleet built: " + spec["blueprintId"]);
            }
            // Only switch the live catalog after every generated prefab has passed validation.
            var settings = new SerializedObject(catalog);
            var entries = settings.FindProperty("entries");
            foreach (var registration in registrations)
            {
                int index = 0;
                while (index < entries.arraySize && entries.GetArrayElementAtIndex(index)
                    .FindPropertyRelative("blueprintId").stringValue != registration.id) index++;
                if (index == entries.arraySize) entries.arraySize++;
                var entry = entries.GetArrayElementAtIndex(index);
                entry.FindPropertyRelative("blueprintId").stringValue = registration.id;
                entry.FindPropertyRelative("prefab").objectReferenceValue = registration.prefab;
            }
            if (registrations.Count > 0)
            {
                settings.ApplyModifiedPropertiesWithoutUndo();
                EditorUtility.SetDirty(catalog);
            }
            AssetDatabase.SaveAssets();
            if (!TrafficVehicleCatalogValidator.ValidateAll(null))
                throw new InvalidOperationException("CARLA fleet catalog validation failed.");
            File.WriteAllText(Assets + "/import_report.json", report.ToString() + "\n");
            AssetDatabase.ImportAsset(Assets + "/import_report.json");
            Debug.Log($"CARLA fleet import PASSED: {fleet.Count} validated traffic prefabs, {registrations.Count} catalog registrations.");
        }

        public static bool RegistersInCatalog(JToken spec) => (bool?)spec["registerInCatalog"] ?? true;

        private static void ConfigureModel(string path, bool proxy)
        {
            var importer = AssetImporter.GetAtPath(path) as ModelImporter;
            if (importer == null) throw new InvalidOperationException("Missing model: " + path);
            importer.globalScale = 1;
            importer.useFileScale = true;
            importer.isReadable = proxy;
            importer.importAnimation = false;
            importer.importCameras = false;
            importer.importLights = false;
            importer.addCollider = false;
            importer.materialImportMode = proxy ? ModelImporterMaterialImportMode.None : ModelImporterMaterialImportMode.ImportStandard;
            importer.SaveAndReimport();
        }

        private static Dictionary<string, Material> BuildMaterials(JObject spec, string directory, string visualPath)
        {
            var mapping = JObject.Parse(File.ReadAllText(directory + "/material_texture_mapping.json"));
            Directory.CreateDirectory(directory + "/Materials");
            AssetDatabase.Refresh();
            var model = AssetDatabase.LoadAssetAtPath<GameObject>(visualPath);
            var result = new Dictionary<string, Material>(StringComparer.Ordinal);
            foreach (var source in model.GetComponentsInChildren<Renderer>(true).SelectMany(r => r.sharedMaterials).Distinct())
            {
                if (source == null) throw new InvalidOperationException("Missing source material: " + visualPath);
                string path = directory + "/Materials/" + source.name + ".mat";
                var material = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (material == null)
                {
                    material = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                    AssetDatabase.CreateAsset(material, path);
                }
                material.shader = Shader.Find("Universal Render Pipeline/Lit");
                material.shaderKeywords = Array.Empty<string>();
                var color = source.HasProperty("_Color") ? source.GetColor("_Color") : Color.white;
                Texture texture = null;
                var links = mapping[source.name] as JArray;
                if (links != null)
                {
                    foreach (var link in links)
                    {
                        if (link["connections"] is not JArray connections || !connections.Values<string>().Contains("Base Color")) continue;
                        string texturePath = directory + "/Textures/" + (string)link["texture"];
                        texture = AssetDatabase.LoadAssetAtPath<Texture2D>(texturePath);
                        if (texture == null) throw new InvalidOperationException("Missing base-color texture: " + texturePath);
                    }
                }
                // Source mappings are approximate Unreal shader reconstructions. Keep reviewed
                // albedo corrections in the manifest rather than changing the supplied reports.
                string baseName = source.name;
                int suffix = baseName.LastIndexOf('.');
                if (suffix >= 0 && int.TryParse(baseName.Substring(suffix + 1), out _)) baseName = baseName.Substring(0, suffix);
                var overrides = spec["baseColorOverrides"] as JObject;
                if (overrides != null && overrides.TryGetValue(baseName, out var replacement))
                {
                    texture = replacement.Type == JTokenType.Null ? null :
                        AssetDatabase.LoadAssetAtPath<Texture2D>(directory + "/Textures/" + (string)replacement);
                    if (replacement.Type != JTokenType.Null && texture == null)
                        throw new InvalidOperationException("Missing reviewed albedo: " + replacement);
                }
                if (texture != null)
                {
                    string textureName = texture.name.ToLowerInvariant();
                    if (textureName.Contains("_n_") || textureName.Contains("normal") || textureName.Contains("_orm")
                        || textureName.Contains("asphalt") || textureName.Contains("dirt") || textureName.Contains("mask"))
                        texture = null;
                }
                // The FBX's generic diffuse factor is not an authored tint for these texture maps.
                // Use the exported albedo directly; preserve diffuse colors on untextured slots.
                if (texture != null) color = Color.white;
                material.SetTexture("_BaseMap", texture);
                material.SetColor("_BaseColor", color);
                string lower = source.name.ToLowerInvariant();
                bool glass = lower.Contains("glass");
                material.SetFloat("_Metallic", lower.Contains("aluminium") || lower.Contains("metal") ? 0.7f : 0.1f);
                material.SetFloat("_Smoothness", glass ? 0.9f : 0.45f);
                material.SetFloat("_Surface", glass ? 1 : 0);
                material.SetFloat("_Blend", 0);
                material.SetFloat("_SrcBlend", (float)(glass ? BlendMode.SrcAlpha : BlendMode.One));
                material.SetFloat("_DstBlend", (float)(glass ? BlendMode.OneMinusSrcAlpha : BlendMode.Zero));
                material.SetFloat("_ZWrite", glass ? 0 : 1);
                material.SetOverrideTag("RenderType", glass ? "Transparent" : "Opaque");
                material.renderQueue = glass ? (int)RenderQueue.Transparent : -1;
                material.SetShaderPassEnabled("ShadowCaster", !glass);
                if (glass)
                {
                    color.a = Mathf.Min(color.a, 0.3f);
                    material.SetColor("_BaseColor", color);
                    material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                }
                material.enableInstancing = true;
                EditorUtility.SetDirty(material);
                result.Add(source.name, material);
            }
            return result;
        }

        private static GameObject BuildPrefab(JObject spec, string directory, string visualPath, string proxyPath,
            Dictionary<string, Material> materials)
        {
            var root = new GameObject((string)spec["name"]);
            root.SetActive(false);
            try
            {
                var alignment = new GameObject("Alignment").transform;
                alignment.SetParent(root.transform, false);
                // CARLA X-forward exports import facing Unity -X, as with the original Audi.
                alignment.localRotation = Quaternion.Euler(0, 90, 0);
                var visual = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(visualPath));
                visual.transform.SetParent(alignment, false);
                var proxy = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(proxyPath));
                proxy.transform.SetParent(alignment, false);
                foreach (var renderer in proxy.GetComponentsInChildren<Renderer>(true)) renderer.enabled = false;
                var filters = proxy.GetComponentsInChildren<MeshFilter>(true);
                if (filters.Length != 1) throw new InvalidOperationException("Expected one combined proxy: " + proxyPath);
                var filter = filters[0];
                var baker = filter.gameObject.AddComponent<MeshToSDF>();
                baker.floodFillIterations = 5;
                var bakerSettings = new SerializedObject(baker);
                bakerSettings.FindProperty("m_Compute").objectReferenceValue = AssetDatabase.LoadAssetAtPath<ComputeShader>(
                    AssetDatabase.GUIDToAssetPath("21a034c7a3ab6be46873a82e0d02ec7b"));
                bakerSettings.ApplyModifiedPropertiesWithoutUndo();
                var volumeObject = new GameObject("SDFTex");
                volumeObject.transform.SetParent(root.transform, false);
                var volume = volumeObject.AddComponent<SDFTexture>();
                Bounds proxyBounds = TrafficVehicleCatalogValidator.MeshBoundsIn(filter, root.transform);
                volumeObject.transform.localPosition = proxyBounds.center;
                const int resolution = 8;
                float voxel = proxyBounds.size.x / (resolution - 2.1f);
                var size = new Vector3(resolution * voxel, (Mathf.Ceil(proxyBounds.size.y / voxel) + 3) * voxel,
                    (Mathf.Ceil(proxyBounds.size.z / voxel) + 3) * voxel);
                volume.size = size;
                volume.resolution = resolution;
                baker.sdfTexture = volume;
                var visualMeshes = visual.GetComponentsInChildren<MeshFilter>(true);
                if (visualMeshes.Length == 0) throw new InvalidOperationException("No static visual meshes: " + visualPath);
                Bounds bounds = TrafficVehicleCatalogValidator.MeshBoundsIn(visualMeshes[0], root.transform);
                foreach (var mesh in visualMeshes.Skip(1)) bounds.Encapsulate(TrafficVehicleCatalogValidator.MeshBoundsIn(mesh, root.transform));
                // Check units and axis conversion against the supplied export's metre-scale bounds.
                var validation = JObject.Parse(File.ReadAllText(directory + "/export_validation.json"));
                var dimensions = (JArray)validation[Path.GetFileName(visualPath)]["dimensions_m"];
                var expected = new Vector3((float)dimensions[1], (float)dimensions[2], (float)dimensions[0]);
                if ((bounds.size - expected).magnitude > 0.02f)
                    throw new InvalidOperationException($"Unexpected imported dimensions for {root.name}: {bounds.size}, expected {expected}");
                var box = root.AddComponent<BoxCollider>();
                box.isTrigger = true;
                box.center = bounds.center;
                box.size = bounds.size;
                var virtualObject = root.AddComponent<VirtualObject>();
                var settings = new SerializedObject(virtualObject);
                settings.FindProperty("boundingBox").objectReferenceValue = box;
                settings.FindProperty("meshToSDF").objectReferenceValue = baker;
                settings.FindProperty("sdfTextureSize").vector3Value = size;
                settings.FindProperty("sdfTextureResolution").intValue = resolution;
                settings.FindProperty("classification").intValue = Convert.ToInt32(Enum.Parse(typeof(VirtualObjectClassification), (string)spec["classification"]));
                settings.ApplyModifiedPropertiesWithoutUndo();
                var paintNames = spec["paintMaterials"].Values<string>().ToArray();
                var bindings = new List<(Renderer renderer, int index)>();
                foreach (var renderer in visual.GetComponentsInChildren<Renderer>(true))
                {
                    var slots = renderer.sharedMaterials;
                    for (int i = 0; i < slots.Length; i++)
                    {
                        string name = slots[i].name;
                        slots[i] = materials[name];
                        if (paintNames.Any(p => name == p || (name.StartsWith(p + ".", StringComparison.Ordinal)
                            && int.TryParse(name.Substring(p.Length + 1), out _)))) bindings.Add((renderer, i));
                    }
                    renderer.sharedMaterials = slots;
                }
                if (bindings.Count == 0) throw new InvalidOperationException("No body paint slots found: " + root.name);
                // URP tint retains the source texture, including markings, when CARLA supplies a color.
                var appearance = root.AddComponent<TrafficVehicleAppearance>();
                var appearanceSettings = new SerializedObject(appearance);
                var entries = appearanceSettings.FindProperty("paintBindings");
                entries.arraySize = bindings.Count;
                for (int i = 0; i < bindings.Count; i++)
                {
                    var entry = entries.GetArrayElementAtIndex(i);
                    entry.FindPropertyRelative("renderer").objectReferenceValue = bindings[i].renderer;
                    entry.FindPropertyRelative("materialIndex").intValue = bindings[i].index;
                    entry.FindPropertyRelative("colorProperty").stringValue = "_BaseColor";
                }
                appearanceSettings.ApplyModifiedPropertiesWithoutUndo();
                var rootSettings = new SerializedObject(root);
                rootSettings.FindProperty("m_IsActive").boolValue = true;
                rootSettings.ApplyModifiedPropertiesWithoutUndo();
                return PrefabUtility.SaveAsPrefabAsset(root, Prefabs + "/" + root.name + ".prefab");
            }
            finally { UnityEngine.Object.DestroyImmediate(root); }
        }
    }
}
