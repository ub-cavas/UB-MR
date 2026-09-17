using System;
using System.IO;
using CAVAS.UB_MR.Tests;
using UB_MR.Redis_Networking;
using UB_MR.Redis_Networking.Editor;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace CAVAS.UB_MR.Tests
{
    public static class TrafficVehicleChecks
    {
        public static void Run()
        {
            try
            {
                CatalogAndColorChecks();
                if (!TrafficVehicleCatalogValidator.ValidateAll(null)) throw new Exception("Catalog validation failed.");
                Debug.Log("Traffic Editor checks PASSED: catalog validation, ordinal lookup, RGB parsing, material cache sharing and cleanup.");
                EditorApplication.Exit(0);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        public static void RunPlayMode() { PrepareScene(false); EditorApplication.EnterPlaymode(); }
        public static void RunGpu() { PrepareScene(true); EditorApplication.EnterPlaymode(); }

        public static void BuildPlayer()
        {
            try
            {
                PrepareScene(true);
                const string scenePath = "Assets/UB_MR/Tests/TrafficValidation.unity";
                EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene(), scenePath);
                // Ros2ForUnity's post-build copier requires fresh destinations for its metadata
                // and versioned libraries. This directory is owned exclusively by this test runner.
                const string outputDirectory = "/tmp/ubmr-traffic-player";
                if (Directory.Exists(outputDirectory)) Directory.Delete(outputDirectory, true);
                var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = new[] { scenePath }, locationPathName = "/tmp/ubmr-traffic-player/TrafficValidation",
                    target = BuildTarget.StandaloneLinux64, options = BuildOptions.Development
                });
                EditorApplication.Exit(report.summary.result == BuildResult.Succeeded ? 0 : 1);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        private static void PrepareScene(bool gpu)
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            var root = new GameObject("Traffic playback checks");
            var checks = root.AddComponent<TrafficPlaybackChecks>();
            checks.catalog = AssetDatabase.LoadAssetAtPath<TrafficVehicleCatalog>(TrafficVehicleCatalogValidator.CatalogPath);
            checks.fallback = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/UB_MR/Prefabs/Traffic/Jeep_Renegade_2016.prefab");
            checks.gpu = gpu;
            if (gpu)
            {
                var camera = new GameObject("Camera").AddComponent<Camera>();
                camera.tag = "MainCamera";
                camera.transform.position = new Vector3(8, 6, 10);
                camera.transform.LookAt(Vector3.zero);
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = new Color(0.18f, 0.21f, 0.25f);
                var light = new GameObject("Sun").AddComponent<Light>();
                light.type = LightType.Directional;
                light.intensity = 2;
                light.transform.rotation = Quaternion.Euler(40, -40, 0);
                RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
                RenderSettings.ambientLight = Color.gray;
            }
        }

        private static void CatalogAndColorChecks()
        {
            var catalog = ScriptableObject.CreateInstance<TrafficVehicleCatalog>();
            var prefab = new GameObject("Catalog test prefab");
            var source = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            try
            {
                SetEntries(catalog, prefab, "vehicle.audi.a2");
                Check(catalog.TryResolve("vehicle.audi.a2", out var found) && found == prefab, "Exact lookup");
                Check(!catalog.TryResolve("VEHICLE.AUDI.A2", out _) && !catalog.TryResolve(null, out _), "Ordinal/null lookup");
                SetEntries(catalog, prefab, "same", "same");
                Check(!catalog.Validate(out _) && !catalog.TryResolve("same", out _), "Duplicate catalog must fail closed");
                SetEntries(catalog, prefab, " ");
                Check(!catalog.Validate(out _), "Empty ID validation");
                SetEntries(catalog, null, "vehicle.missing");
                Check(!catalog.Validate(out _), "Missing prefab validation");
                foreach (string invalid in new[] { null, "", "-1,0,0", "256,0,0", "1,2", "1,2,3,4", "1.0,2,3", "r,2,3", "+1,2,3" })
                    Check(!TrafficVehicleAppearance.TryParseColor(invalid, out _), "Invalid RGB accepted: " + invalid);
                Check(TrafficVehicleAppearance.TryParseColor(" 0 , 128,255 ", out var rgb) && rgb.g == 128, "RGB whitespace");
                using var cache = new TrafficMaterialCache();
                Color authored = source.GetColor("_BaseColor");
                var a = cache.Acquire(source, "_BaseColor", rgb);
                var b = cache.Acquire(source, "_BaseColor", rgb);
                Check(a == b && cache.Count == 1 && source.GetColor("_BaseColor") == authored, "Material sharing/source mutation");
                cache.Release(source, "_BaseColor", rgb);
                Check(cache.Count == 1 && b != null, "Released material still in use");
                cache.Release(source, "_BaseColor", rgb);
                Check(cache.Count == 0 && b == null, "Material cache leak");
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(catalog);
                UnityEngine.Object.DestroyImmediate(prefab);
                UnityEngine.Object.DestroyImmediate(source);
            }
        }

        private static void SetEntries(TrafficVehicleCatalog catalog, GameObject prefab, params string[] ids)
        {
            var settings = new SerializedObject(catalog);
            var entries = settings.FindProperty("entries");
            entries.arraySize = ids.Length;
            for (int i = 0; i < ids.Length; i++)
            {
                entries.GetArrayElementAtIndex(i).FindPropertyRelative("blueprintId").stringValue = ids[i];
                entries.GetArrayElementAtIndex(i).FindPropertyRelative("prefab").objectReferenceValue = prefab;
            }
            settings.ApplyModifiedPropertiesWithoutUndo();
        }
        private static void Check(bool value, string message) { if (!value) throw new Exception(message); }
    }
}
