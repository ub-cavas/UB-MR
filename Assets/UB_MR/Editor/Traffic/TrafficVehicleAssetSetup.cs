using System;
using System.Collections.Generic;
using System.IO;
using CAVAS.UB_MR.DT.Sensors;
using UnityEditor.SceneManagement;
using UnityEditor;
using UnityEngine;

namespace UB_MR.Redis_Networking.Editor
{
    /// <summary>Reproducible setup of the initial two imported traffic assets.</summary>
    public static class TrafficVehicleAssetSetup
    {
        private const string VehicleAssets = "Assets/UB_MR_Assets/Actors/Vehicles/";
        private const string Prefabs = "Assets/UB_MR/Prefabs/Traffic/";

        [MenuItem("UB-MR/Traffic/Rebuild initial Audi and Mustang prefabs")]
        public static void BuildInitialAssets()
        {
            Build("Audi_A2", "Audi_A2_Textured", "Audi_A2_SDF_Proxy", Quaternion.Euler(0, 90, 0), Vector3.zero,
                "MI_BodyWStaticMesh2", null);
            Build("Mustang_2012", "Mustang_Textured", "Mustang_SDF_Proxy", Quaternion.identity, new Vector3(0, 0.15f, 0),
                "Livery", VehicleAssets + "Mustang_2012/Textures/Car_Ford_MustangBoss302_2012_001_Livery_D.png");
            var catalog = AssetDatabase.LoadAssetAtPath<TrafficVehicleCatalog>(TrafficVehicleCatalogValidator.CatalogPath);
            if (catalog == null)
            {
                catalog = ScriptableObject.CreateInstance<TrafficVehicleCatalog>();
                AssetDatabase.CreateAsset(catalog, TrafficVehicleCatalogValidator.CatalogPath);
            }
            var serialized = new SerializedObject(catalog);
            var entries = serialized.FindProperty("entries");
            // Preserve subsequent registrations when rebuilding the initial models.
            SetEntry(entries, "vehicle.audi.a2", "Audi_A2");
            SetEntry(entries, "vehicle.ford.mustang", "Mustang_2012");
            serialized.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(catalog);
            AssetDatabase.SaveAssets();
            if (!TrafficVehicleCatalogValidator.ValidateAll(null)) throw new InvalidOperationException("Generated traffic assets failed validation.");
        }

        public static void BuildBatch()
        {
            try
            {
                BuildInitialAssets();
                var scene = EditorSceneManager.OpenScene("Assets/UB_MR/Modules/UB-Service-Center-Loop/UB-Service-Center-Loop.unity");
                var renderer = UnityEngine.Object.FindFirstObjectByType<TrafficRenderer>(FindObjectsInactive.Include);
                if (renderer == null) throw new InvalidOperationException("Scene has no TrafficRenderer.");
                var settings = new SerializedObject(renderer);
                settings.FindProperty("vehicleCatalog").objectReferenceValue =
                    AssetDatabase.LoadAssetAtPath<TrafficVehicleCatalog>(TrafficVehicleCatalogValidator.CatalogPath);
                settings.ApplyModifiedPropertiesWithoutUndo();
                EditorSceneManager.SaveScene(scene);
                EditorApplication.Exit(0);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        private static void SetEntry(SerializedProperty entries, string id, string prefab)
        {
            int index = 0;
            while (index < entries.arraySize && entries.GetArrayElementAtIndex(index).FindPropertyRelative("blueprintId").stringValue != id) index++;
            if (index == entries.arraySize) entries.arraySize++;
            var entry = entries.GetArrayElementAtIndex(index);
            entry.FindPropertyRelative("blueprintId").stringValue = id;
            entry.FindPropertyRelative("prefab").objectReferenceValue = AssetDatabase.LoadAssetAtPath<GameObject>(Prefabs + prefab + ".prefab");
        }

        private static void Build(string folder, string visualName, string proxyName, Quaternion rotation,
            Vector3 position, string paintName, string maskSource)
        {
            var root = new GameObject(folder);
            // Prevent perception registration while constructing the prefab.
            root.SetActive(false);
            try
            {
                var alignment = new GameObject("Alignment").transform;
                alignment.SetParent(root.transform, false);
                alignment.localRotation = rotation;
                alignment.localPosition = position;
                var visual = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(VehicleAssets + folder + "/" + visualName + ".fbx"));
                visual.transform.SetParent(alignment, false);
                string proxyPath = VehicleAssets + folder + "/" + proxyName + ".fbx";
                // MeshToSDF changes the GPU buffer targets at runtime. Player builds need a readable
                // source mesh to recreate buffers even though the proxy renderer is disabled.
                var proxyImporter = (ModelImporter)AssetImporter.GetAtPath(proxyPath);
                if (!proxyImporter.isReadable) { proxyImporter.isReadable = true; proxyImporter.SaveAndReimport(); }
                var proxy = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>(proxyPath));
                proxy.transform.SetParent(alignment, false);
                foreach (var r in proxy.GetComponentsInChildren<Renderer>()) r.enabled = false;
                var filter = proxy.GetComponentInChildren<MeshFilter>();
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
                // Eight X voxels, with >1 voxel padding per side. Round Y/Z upward to complete voxels.
                const int resolution = 8;
                float voxel = proxyBounds.size.x / (resolution - 2.1f);
                var size = new Vector3(resolution * voxel,
                    (Mathf.Ceil(proxyBounds.size.y / voxel) + 3) * voxel,
                    (Mathf.Ceil(proxyBounds.size.z / voxel) + 3) * voxel);
                volume.size = size;
                volume.resolution = resolution;
                baker.sdfTexture = volume;
                Bounds visualBounds = default;
                bool first = true;
                foreach (var mesh in visual.GetComponentsInChildren<MeshFilter>())
                {
                    var bounds = TrafficVehicleCatalogValidator.MeshBoundsIn(mesh, root.transform);
                    if (first) { visualBounds = bounds; first = false; } else visualBounds.Encapsulate(bounds);
                }
                var box = root.AddComponent<BoxCollider>();
                box.isTrigger = true;
                box.center = visualBounds.center;
                box.size = visualBounds.size;
                var virtualObject = root.AddComponent<VirtualObject>();
                var settings = new SerializedObject(virtualObject);
                settings.FindProperty("boundingBox").objectReferenceValue = box;
                settings.FindProperty("meshToSDF").objectReferenceValue = baker;
                settings.FindProperty("sdfTextureSize").vector3Value = size;
                settings.FindProperty("sdfTextureResolution").intValue = resolution;
                settings.FindProperty("classification").intValue = (int)VirtualObjectClassification.Car;
                settings.ApplyModifiedPropertiesWithoutUndo();
                SetupPaint(root, visual, folder, paintName, maskSource);
                // Save an active prefab without activating this temporary scene object.
                var rootSettings = new SerializedObject(root);
                rootSettings.FindProperty("m_IsActive").boolValue = true;
                rootSettings.ApplyModifiedPropertiesWithoutUndo();
                PrefabUtility.SaveAsPrefabAsset(root, Prefabs + folder + ".prefab");
            }
            finally { UnityEngine.Object.DestroyImmediate(root); }
        }

        private static void SetupPaint(GameObject root, GameObject visual, string folder, string paintName, string maskSource)
        {
            string directory = VehicleAssets + folder + "/Materials";
            Directory.CreateDirectory(directory);
            AssetDatabase.Refresh();
            var bindings = new List<(Renderer renderer, int slot)>();
            Material paint = null;
            foreach (var renderer in visual.GetComponentsInChildren<Renderer>())
            {
                var materials = renderer.sharedMaterials;
                for (int i = 0; i < materials.Length; i++)
                {
                    if (materials[i].name != paintName) continue;
                    if (paint == null)
                    {
                        string path = directory + "/TrafficPaint.mat";
                        paint = AssetDatabase.LoadAssetAtPath<Material>(path);
                        if (paint == null) { paint = new Material(Shader.Find("UB-MR/Traffic Paint")); AssetDatabase.CreateAsset(paint, path); }
                        paint.CopyPropertiesFromMaterial(materials[i]);
                        paint.shader = Shader.Find("UB-MR/Traffic Paint");
                        paint.SetFloat("_Metallic", 0.35f);
                        paint.SetFloat("_Smoothness", 0.65f);
                        if (maskSource != null)
                        {
                            var mask = CreatePaintMask(maskSource, directory + "/TrafficPaintMask.png", out Color defaultPaint);
                            paint.SetTexture("_PaintMask", mask);
                            paint.SetColor("_BaseColor", defaultPaint);
                        }
                        EditorUtility.SetDirty(paint);
                    }
                    materials[i] = paint;
                    bindings.Add((renderer, i));
                }
                renderer.sharedMaterials = materials;
            }
            if (bindings.Count == 0) throw new InvalidOperationException("Paint material not found: " + paintName);
            var appearance = root.AddComponent<TrafficVehicleAppearance>();
            var settings = new SerializedObject(appearance);
            var entries = settings.FindProperty("paintBindings");
            entries.arraySize = bindings.Count;
            for (int i = 0; i < bindings.Count; i++)
            {
                var entry = entries.GetArrayElementAtIndex(i);
                entry.FindPropertyRelative("renderer").objectReferenceValue = bindings[i].renderer;
                entry.FindPropertyRelative("materialIndex").intValue = bindings[i].slot;
                entry.FindPropertyRelative("colorProperty").stringValue = "_BaseColor";
            }
            settings.ApplyModifiedPropertiesWithoutUndo();
        }

        private static Texture2D CreatePaintMask(string source, string output, out Color defaultPaint)
        {
            var importer = (TextureImporter)AssetImporter.GetAtPath(source);
            bool wasReadable = importer.isReadable;
            defaultPaint = Color.white;
            try
            {
                importer.isReadable = true;
                importer.SaveAndReimport();
                var texture = AssetDatabase.LoadAssetAtPath<Texture2D>(source);
                var pixels = texture.GetPixels32();
                var counts = new Dictionary<Color32, int>();
                int maxCount = 0;
                for (int i = 0; i < pixels.Length; i++)
                {
                    var color = pixels[i];
                    // This livery uses saturated orange for paint and neutral grays for preserved markings.
                    float mask = Mathf.Clamp01((color.r - color.b) / Mathf.Max(1f, color.r));
                    if (mask > 0.99f)
                    {
                        counts.TryGetValue(color, out int count);
                        counts[color] = ++count;
                        if (count > maxCount) { defaultPaint = color; maxCount = count; }
                    }
                    byte value = (byte)Mathf.RoundToInt(mask * 255);
                    pixels[i] = new Color32(value, value, value, 255);
                }
                var result = new Texture2D(texture.width, texture.height, TextureFormat.RGB24, false, true);
                result.SetPixels32(pixels);
                result.Apply();
                File.WriteAllBytes(output, result.EncodeToPNG());
                UnityEngine.Object.DestroyImmediate(result);
            }
            finally { importer.isReadable = wasReadable; importer.SaveAndReimport(); }
            AssetDatabase.ImportAsset(output);
            var maskImporter = (TextureImporter)AssetImporter.GetAtPath(output);
            maskImporter.sRGBTexture = false;
            maskImporter.textureCompression = TextureImporterCompression.Uncompressed;
            maskImporter.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(output);
        }

        public static void Inspect()
        {
            foreach (string path in new[] {
                "Assets/UB_MR_Assets/Actors/Vehicles/Audi_A2/Audi_A2_Textured.fbx",
                "Assets/UB_MR_Assets/Actors/Vehicles/Audi_A2/Audi_A2_SDF_Proxy.fbx",
                "Assets/UB_MR_Assets/Actors/Vehicles/Mustang_2012/Mustang_Textured.fbx",
                "Assets/UB_MR_Assets/Actors/Vehicles/Mustang_2012/Mustang_SDF_Proxy.fbx" })
            {
                var root = AssetDatabase.LoadAssetAtPath<GameObject>(path);
                Debug.Log("TRAFFIC MODEL " + path);
                foreach (var filter in root.GetComponentsInChildren<MeshFilter>(true))
                    Debug.Log($"TRAFFIC MESH {filter.name} bounds={filter.sharedMesh.bounds} rotation={filter.transform.localEulerAngles} scale={filter.transform.localScale}");
                foreach (var renderer in root.GetComponentsInChildren<Renderer>(true))
                    for (int i = 0; i < renderer.sharedMaterials.Length; i++)
                    {
                        var m = renderer.sharedMaterials[i];
                        Debug.Log($"TRAFFIC MATERIAL {renderer.name}[{i}] {m.name} shader={m.shader.name} color={m.color} texture={m.mainTexture?.name}");
                    }
            }
            EditorApplication.Exit(0);
        }
    }
}
