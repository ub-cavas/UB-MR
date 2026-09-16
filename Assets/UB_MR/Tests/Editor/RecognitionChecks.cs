using System;
using System.Reflection;
using CAVAS.UB_MR.Config;
using CAVAS.UB_MR.DT.Sensors;
using Newtonsoft.Json;
using UnityEditor;
using UnityEngine;
using UnityEngine.Assertions;

namespace CAVAS.UB_MR.Tests
{
    /// <summary>Run with Unity -batchmode -executeMethod CAVAS.UB_MR.Tests.RecognitionChecks.Run.</summary>
    public static class RecognitionChecks
    {
        public static void Run()
        {
            try
            {
                ConfigRoundTrip();
                ClockScheduling();
                GeometryAndLifecycle();
                Debug.Log("UB-MR recognition checks PASSED: config, clock, geometry, messages, registry, SDF mode.");
                EditorApplication.Exit(0);
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                EditorApplication.Exit(1);
            }
        }

        public static void RunGpu()
        {
            ComputeBuffer input = null, output = null;
            Texture3D texture = null;
            ComputeShader shader = null;
            try
            {
                Check(SystemInfo.supportsComputeShaders, "Compute shaders unavailable.");
                shader = UnityEngine.Object.Instantiate(Resources.Load<ComputeShader>("Scripts/SDFRaymarch"));
                const int resolution = 32;
                var distances = new float[resolution * resolution * resolution];
                for (int z = 0; z < resolution; z++)
                for (int y = 0; y < resolution; y++)
                for (int x = 0; x < resolution; x++)
                {
                    var p = new Vector3((x + 0.5f) / resolution, (y + 0.5f) / resolution, (z + 0.5f) / resolution);
                    distances[x + resolution * (y + resolution * z)] = (p - Vector3.one * 0.5f).magnitude - 0.25f;
                }
                texture = new Texture3D(resolution, resolution, resolution, TextureFormat.RFloat, false)
                    { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
                texture.SetPixelData(distances, 0);
                texture.Apply();
                var points = new[] { new Vector4(0.1f, 0.1f, 10, 0), new Vector4(10, 0.1f, 0.1f, 0) };
                input = new ComputeBuffer(2, 16);
                output = new ComputeBuffer(2, 16);
                input.SetData(points);
                int kernel = shader.FindKernel("SDFRaymarch");
                shader.SetBuffer(kernel, "_Points", input);
                shader.SetBuffer(kernel, "_ModifiedPoints", output);
                shader.SetTexture(kernel, "_SDF", texture);
                var worldToTexture = Matrix4x4.Translate(Vector3.one * 0.5f) *
                    Matrix4x4.TRS(new Vector3(0, 0, 5), Quaternion.identity, Vector3.one * 2).inverse;
                shader.SetMatrix("_WorldToSDFSpace", worldToTexture);
                shader.SetMatrix("_SDFToWorldSpace", worldToTexture.inverse);
                shader.SetMatrix("_LocalToWorldRS", Matrix4x4.identity);
                shader.SetMatrix("_WorldToLocalRS", Matrix4x4.identity);
                shader.SetVector("_RayOriginWorld", Vector3.zero);
                shader.SetFloat("_Margin", 0);
                shader.SetFloat("_MaxDistance", 100);
                shader.SetFloat("_HitThreshold", 0.001f);
                shader.SetInt("_MaxIterations", 128);
                shader.Dispatch(kernel, 1, 1, 1);
                var result = new Vector4[2];
                output.GetData(result);
                Check(result[0].z > 4.4f && result[0].z < 4.7f && result[0].w == 1,
                    "GPU SDF ray did not hit the virtual sphere: " + result[0]);
                Check(result[1] == points[1], "GPU changed a ray that missed the virtual sphere.");
                Debug.Log("UB-MR GPU checks PASSED: SDF hit shortens scan ray; miss preserves original point.");
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                EditorApplication.Exit(1);
                return;
            }
            finally
            {
                input?.Release();
                output?.Release();
                UnityEngine.Object.DestroyImmediate(texture);
                UnityEngine.Object.DestroyImmediate(shader);
            }
            // Also rerun the non-graphics checks after final runtime changes.
            Run();
        }

        static void Check(bool condition, string message)
        {
            if (!condition) throw new Exception(message);
        }

        static void ConfigRoundTrip()
        {
            var legacy = JsonConvert.DeserializeObject<Config.Agent>("{\"name\":\"legacy\",\"sensors\":{}}");
            Assert.AreEqual(VirtualObjectRecognitionMode.LidarModification, legacy.recognition.mode);
            Assert.AreEqual(30f, legacy.recognition.publishRateHz);
            Assert.AreEqual(1000f, legacy.recognition.detectionRadiusMeters);
            legacy.recognition.mode = VirtualObjectRecognitionMode.BoundingBoxInjection;
            legacy.recognition.useSimTime = true;
            legacy.recognition.boundingBoxTopic = "/test/virtual_objects";
            legacy.recognition.publishRateHz = 20;
            legacy.recognition.detectionRadiusMeters = 123;
            string json = JsonConvert.SerializeObject(legacy);
            Check(json.Contains("BoundingBoxInjection"), "Mode must serialize by name.");
            var restored = JsonConvert.DeserializeObject<Config.Agent>(json);
            Assert.AreEqual(VirtualObjectRecognitionMode.BoundingBoxInjection, restored.recognition.mode);
            Check(restored.recognition.useSimTime, "Clock setting lost.");
            Assert.AreEqual(20f, restored.recognition.publishRateHz);
            Assert.AreEqual(123f, restored.recognition.detectionRadiusMeters);
            Assert.AreEqual("/test/virtual_objects", restored.recognition.boundingBoxTopic);
            Check(restored.recognition.TryValidate(out _), "Valid settings rejected.");
            restored.recognition.boundingBoxTopic = "/invalid topic";
            Check(!restored.recognition.TryValidate(out _), "Invalid ROS topic accepted.");
            restored.recognition.boundingBoxTopic = "/objects";
            foreach (float invalid in new[] { 0f, -1f, float.NaN, float.PositiveInfinity })
            {
                restored.recognition.publishRateHz = invalid;
                Check(!restored.recognition.TryValidate(out _), "Invalid rate accepted.");
            }
        }

        static void ClockScheduling()
        {
            var schedule = new DetectionSchedule(10f);
            Check(!schedule.ShouldPublish(0), "Zero ROS time must wait.");
            Check(schedule.ShouldPublish(1_000_000_001L), "First sample missing.");
            Check(!schedule.ShouldPublish(1_000_000_001L), "Paused clock produced a duplicate.");
            Check(!schedule.ShouldPublish(1_050_000_001L), "Rate limit ignored.");
            Check(schedule.ShouldPublish(1_100_000_001L), "Scheduled sample missing.");
            Check(schedule.ShouldPublish(500_000_001L), "Backward clock jump not reset.");
            Check(!schedule.ShouldPublish(500_000_001L), "Duplicate after clock reset.");
            Check(schedule.ShouldPublish(5_000_000_001L), "Forward jump not handled.");
        }

        static void SetField(object target, string field, object value) => target.GetType()
            .GetField(field, BindingFlags.Instance | BindingFlags.NonPublic).SetValue(target, value);

        static void Near(Vector3 expected, Vector3 actual, string context)
            => Check(Vector3.Distance(expected, actual) < 0.0001f, context + $": {expected} != {actual}");

        static void GeometryAndLifecycle()
        {
            var ego = new GameObject("ego");
            var root = new GameObject("virtual");
            var child = new GameObject("collider");
            VirtualBoundingBoxDetector.ClearVirtualObjectDatabase();
            try
            {
                ego.transform.SetPositionAndRotation(new Vector3(10, 0, 20), Quaternion.Euler(0, 90, 0));
                child.transform.SetParent(root.transform, false);
                child.transform.localPosition = new Vector3(0, 1, 0);
                root.transform.SetPositionAndRotation(new Vector3(13, 0, 20), Quaternion.Euler(0, 45, 0));
                var collider = child.AddComponent<BoxCollider>();
                collider.center = new Vector3(0, 0.5f, 0);
                collider.size = new Vector3(2, 2, 4);
                var obj = root.AddComponent<VirtualObject>();
                SetField(obj, "boundingBox", collider);
                VirtualBoundingBoxDetector.AddVirtualObjectToDatabase(obj);
                Check(obj.TryGetBoundingBox(out var center, out var rotation, out var size), "Box missing.");
                Near(new Vector3(13, 1.5f, 20), center, "Child collider center");
                Near(new Vector3(2, 2, 4), size, "Rotation must not inflate dimensions");
                var stamp = new builtin_interfaces.msg.Time { Sec = 42, Nanosec = 123456789 };
                var message = VirtualBoundingBoxDetector.BuildMessage(ego.transform, ego.transform, 10, stamp);
                Assert.AreEqual(1, message.Objects.Length);
                Assert.AreEqual("base_link", message.Header.Frame_id);
                Assert.AreEqual(123456789u, message.Header.Stamp.Nanosec);
                var detection = message.Objects[0];
                Near(new Vector3(3, 0, 1.5f), new Vector3((float)detection.Kinematics.Pose_with_covariance.Pose.Position.X,
                    (float)detection.Kinematics.Pose_with_covariance.Pose.Position.Y,
                    (float)detection.Kinematics.Pose_with_covariance.Pose.Position.Z), "Rotated ego ROS pose");
                Assert.AreEqual(4.0, detection.Shape.Dimensions.X);
                Assert.AreEqual(2.0, detection.Shape.Dimensions.Y);
                Assert.AreEqual(1f, detection.Existence_probability);
                Check(detection.Kinematics.Has_position_covariance && !detection.Kinematics.Has_twist,
                    "Kinematic validity flags incorrect.");
                Assert.AreEqual(autoware_perception_msgs.msg.DetectedObjectKinematics.AVAILABLE,
                    detection.Kinematics.Orientation_availability);
                Assert.AreEqual(autoware_perception_msgs.msg.ObjectClassification.CAR, detection.Classification[0].Label);
                Assert.AreEqual(0, VirtualBoundingBoxDetector.BuildMessage(ego.transform, ego.transform, 1, stamp).Objects.Length);
                float boundary = Vector3.Distance(center, ego.transform.position);
                Assert.AreEqual(1, VirtualBoundingBoxDetector.BuildMessage(ego.transform, ego.transform, boundary + 0.0001f, stamp).Objects.Length);
                root.transform.localScale = new Vector3(2, 3, 4);
                obj.TryGetBoundingBox(out _, out _, out size);
                Near(new Vector3(4, 6, 16), size, "Scaled box");
                child.transform.localRotation = Quaternion.Euler(0, 35, 0);
                obj.TryGetBoundingBox(out center, out rotation, out size);
                // Every transformed collider corner must be enclosed even with parent-induced shear.
                for (int i = 0; i < 8; i++)
                {
                    var corner = collider.center + Vector3.Scale(collider.size * 0.5f,
                        new Vector3((i & 1) == 0 ? -1 : 1, (i & 2) == 0 ? -1 : 1, (i & 4) == 0 ? -1 : 1));
                    var local = Quaternion.Inverse(rotation) * (child.transform.TransformPoint(corner) - center);
                    Check(Mathf.Abs(local.x) <= size.x / 2 + 0.0001f && Mathf.Abs(local.y) <= size.y / 2 + 0.0001f &&
                        Mathf.Abs(local.z) <= size.z / 2 + 0.0001f, "Sheared box corner outside bounds.");
                }
                root.transform.SetParent(ego.transform, true);
                Assert.AreEqual(0, VirtualBoundingBoxDetector.BuildMessage(ego.transform, ego.transform, 100, stamp).Objects.Length);
                root.transform.SetParent(null, true);
                obj.enabled = false;
                Assert.AreEqual(0, VirtualBoundingBoxDetector.BuildMessage(ego.transform, ego.transform, 100, stamp).Objects.Length);
                obj.enabled = true;
                VirtualBoundingBoxDetector.RemoveVirtualObjectFromDatabase(obj);
                Assert.AreEqual(0, VirtualBoundingBoxDetector.BuildMessage(ego.transform, ego.transform, 100, stamp).Objects.Length);
                VirtualBoundingBoxDetector.AddVirtualObjectToDatabase(obj);
                UnityEngine.Object.DestroyImmediate(root);
                Assert.AreEqual(0, VirtualBoundingBoxDetector.BuildMessage(ego.transform, ego.transform, 100, stamp).Objects.Length);

                ConfigurationManager.SetActiveAgent(new Config.Agent { recognition = new VirtualObjectRecognitionSettings
                    { mode = VirtualObjectRecognitionMode.BoundingBoxInjection } });
                root = new GameObject("box-only object");
                var sdf = root.AddComponent<MeshToSDF>();
                obj = root.AddComponent<VirtualObject>();
                SetField(obj, "meshToSDF", sdf);
                typeof(VirtualObject).GetMethod("Awake", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(obj, null);
                typeof(VirtualObject).GetMethod("Start", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(obj, null);
                Check(!sdf.enabled, "Box mode must disable MeshToSDF.");
                Check(typeof(VirtualObject).GetField("ownedSdf", BindingFlags.NonPublic | BindingFlags.Instance)
                    .GetValue(obj) == null, "Box mode allocated an SDF texture.");
            }
            finally
            {
                ConfigurationManager.SetActiveAgent(null);
                VirtualBoundingBoxDetector.ClearVirtualObjectDatabase();
                UnityEngine.Object.DestroyImmediate(root);
                UnityEngine.Object.DestroyImmediate(ego);
            }
        }
    }
}
