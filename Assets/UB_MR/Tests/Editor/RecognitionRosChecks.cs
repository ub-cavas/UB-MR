using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using CAVAS.UB_MR.Config;
using CAVAS.UB_MR.ROS2;
using UnityEditor.SceneManagement;
using System.Threading;
using CAVAS.UB_MR.DT.Sensors;
using CAVAS.UB_MR.DT.Sensors.Lidar;
using ROS2;
using sensor_msgs.msg;
using UnityEditor;
using UnityEngine;

namespace CAVAS.UB_MR.Tests
{
    public static class RecognitionRosChecks
    {
        const string CombinedKey = "UBMR.CombinedRecognitionCheck";
        static DT.Agent combinedAgent;
        static GameObject moduleObject, sdfObject, agentObject, virtualObject;
        static Texture3D sphere;
        static ROS2Node combinedNode;
        static IPublisher<PointCloud2> scanPublisher;
        static ISubscription<PointCloud2> scanSubscription;
        static ISubscription<autoware_perception_msgs.msg.DetectedObjects> boxSubscription;
        static PointCloud2 combinedScan;
        static autoware_perception_msgs.msg.DetectedObjects combinedBoxes;
        static double combinedDeadline, publishAfter;

        [InitializeOnLoadMethod]
        static void RegisterCombinedCheck()
        {
            EditorApplication.playModeStateChanged += state =>
            {
                if (state == PlayModeStateChange.EnteredPlayMode && SessionState.GetBool(CombinedKey, false))
                {
                    combinedDeadline = EditorApplication.timeSinceStartup + 20;
                    EditorApplication.update += CombinedTick;
                }
            };
        }

        /// <summary>Graphics + isolated ROS required: real Agent.Setup/Update/LateUpdate and DDS outputs.</summary>
        public static void RunCombined()
        {
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
            SessionState.SetBool(CombinedKey, true);
            EditorApplication.EnterPlaymode();
        }

        static FieldInfo AgentField(string name) => typeof(DT.Agent).GetField(name, BindingFlags.Instance | BindingFlags.NonPublic);

        static void CombinedTick()
        {
            try
            {
                if (combinedAgent == null)
                {
                    if (!SystemInfo.supportsComputeShaders) throw new Exception("Combined check requires compute graphics.");
                    var config = new Config.Agent { name = "combined_runtime_check", isDynamic = false,
                        recognition = new VirtualObjectRecognitionSettings { mode = VirtualObjectRecognitionMode.LidarModificationAndBoundingBoxInjection,
                            boundingBoxTopic = "/ub_mr_test/combined_boxes" } };
                    config.sensors.Add("top", new Config.Sensor { name = "top", type = SensorType.LiDAR, topic = "/ub_mr_test/combined_scan" });
                    ConfigurationManager.SetActiveAgent(config);
                    moduleObject = new GameObject("combined test module");
                    var module = moduleObject.AddComponent<Module>();
                    module.enabled = false;
                    if (!module.UsesLidarModification) throw new Exception("Combined Module disabled SDF refresh.");
                    const int resolution = 32;
                    sphere = new Texture3D(resolution, resolution, resolution, TextureFormat.RFloat, false)
                        { filterMode = FilterMode.Bilinear, wrapMode = TextureWrapMode.Clamp };
                    var distances = new float[resolution * resolution * resolution];
                    for (int z = 0; z < resolution; z++)
                    for (int y = 0; y < resolution; y++)
                    for (int x = 0; x < resolution; x++)
                        distances[x + resolution * (y + resolution * z)] =
                            (new Vector3((x + .5f) / resolution, (y + .5f) / resolution, (z + .5f) / resolution) - Vector3.one * .5f).magnitude - .25f;
                    sphere.SetPixelData(distances, 0); sphere.Apply();
                    sdfObject = new GameObject("known SDF sphere");
                    sdfObject.transform.position = new Vector3(0, 0, 5); sdfObject.transform.localScale = Vector3.one * 2;
                    sdfObject.AddComponent<SDFTexture>().sdf = sphere;
                    typeof(Module).GetMethod("RefreshSDFList", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(module, null);
                    if (module.GetSDFs().Count != 1) throw new Exception("Combined Module did not refresh actual SDF.");
                    virtualObject = new GameObject("virtual box");
                    virtualObject.transform.position = new Vector3(0, 0, 5);
                    var collider = virtualObject.AddComponent<BoxCollider>();
                    var obj = virtualObject.AddComponent<VirtualObject>();
                    typeof(VirtualObject).GetField("boundingBox", BindingFlags.Instance | BindingFlags.NonPublic).SetValue(obj, collider);
                    combinedNode = ROS2_Bridge.ROS_CORE.CreateNode("combined_runtime_receiver");
                    scanPublisher = combinedNode.CreatePublisher<PointCloud2>("/ub_mr_test/combined_scan");
                    scanSubscription = combinedNode.CreateSubscription<PointCloud2>("/ub_mr_test/combined_scan_modified", msg => combinedScan = msg);
                    boxSubscription = combinedNode.CreateSubscription<autoware_perception_msgs.msg.DetectedObjects>("/ub_mr_test/combined_boxes", msg => combinedBoxes = msg);
                    // Verify concrete initialization across all modes on the real prefab, then keep combined alive.
                    foreach (VirtualObjectRecognitionMode mode in Enum.GetValues(typeof(VirtualObjectRecognitionMode)))
                    {
                        config.recognition.mode = mode;
                        agentObject = UnityEngine.Object.Instantiate(Resources.Load<GameObject>("Prefabs/Agent"));
                        combinedAgent = agentObject.AddComponent<DT.Agent>();
                        combinedAgent.enabled = false;
                        combinedAgent.Setup(config, module);
                        var sensors = (IEnumerable)AgentField("sensors").GetValue(combinedAgent);
                        object modifier = null;
                        foreach (var entry in sensors) modifier = entry.GetType().GetProperty("Item2").GetValue(entry);
                        if ((modifier is LidarModifier) != config.recognition.UsesLidarModification ||
                            (modifier is LidarPassthrough) == config.recognition.UsesLidarModification ||
                            (AgentField("mVirtualBoundingBoxDetector").GetValue(combinedAgent) != null) != config.recognition.UsesBoundingBoxInjection ||
                            (AgentField("detectionClock").GetValue(combinedAgent) != null) != config.recognition.UsesBoundingBoxInjection ||
                            (AgentField("lidarModifierComputeShader").GetValue(combinedAgent) != null) != config.recognition.UsesLidarModification)
                            throw new Exception("Concrete runtime capability selection failed: " + mode);
                        if (mode == VirtualObjectRecognitionMode.LidarModificationAndBoundingBoxInjection) break;
                        combinedAgent.Teardown(); combinedAgent.Teardown();
                        UnityEngine.Object.Destroy(agentObject);
                    }
                    publishAfter = EditorApplication.timeSinceStartup + 1;
                }
                if (EditorApplication.timeSinceStartup > combinedDeadline) throw new Exception("Combined scan/box DDS delivery timed out.");
                if (EditorApplication.timeSinceStartup < publishAfter) return;
                // Lincoln-sized scan (> the Agent's 60k default buffers): known hit/miss rays sit at the end so
                // the check fails if oversized scans are dropped or only the first buffer-length points are modified.
                const int LargeScanPoints = 156_499;
                int hitOffset = (LargeScanPoints - 2) * 12;
                var coordinates = new[] { 10f, -.1f, .1f, .1f, -10f, .1f };
                var data = new byte[LargeScanPoints * 12]; Buffer.BlockCopy(coordinates, 0, data, hitOffset, coordinates.Length * 4);
                scanPublisher.Publish(new PointCloud2 { Header = new std_msgs.msg.Header { Frame_id = "test_lidar" },
                    Height = 1, Width = LargeScanPoints, Point_step = 12, Row_step = LargeScanPoints * 12, Data = data,
                    Fields = new[] { new PointField { Name = "x", Offset = 0, Datatype = PointField.FLOAT32, Count = 1 },
                        new PointField { Name = "y", Offset = 4, Datatype = PointField.FLOAT32, Count = 1 },
                        new PointField { Name = "z", Offset = 8, Datatype = PointField.FLOAT32, Count = 1 } } });
                typeof(DT.Agent).GetMethod("Update", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(combinedAgent, null);
                typeof(DT.Agent).GetMethod("LateUpdate", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(combinedAgent, null);
                if (combinedScan == null || combinedBoxes == null || combinedBoxes.Objects.Length != 1) return;
                if (combinedScan.Width != LargeScanPoints || combinedScan.Data.Length != data.Length)
                    throw new Exception($"Large scan was resized: {combinedScan.Width} points.");
                float hit = BitConverter.ToSingle(combinedScan.Data, hitOffset);
                if (hit < 4.3f || hit > 4.8f || BitConverter.ToSingle(combinedScan.Data, hitOffset + 12) != .1f ||
                    BitConverter.ToSingle(combinedScan.Data, hitOffset + 16) != -10f)
                    throw new Exception("Real combined LidarModifier failed known hit/miss on a large scan: " + hit);
                Debug.Log($"UB-MR combined runtime PASSED: all concrete mode selections, actual Module SDF, {LargeScanPoints}-point Agent GPU scan and direct box DDS outputs, repeated teardown.");
                FinishCombined(0);
            }
            catch (Exception exception) { Debug.LogException(exception); FinishCombined(1); }
        }

        static void FinishCombined(int status)
        {
            EditorApplication.update -= CombinedTick;
            SessionState.SetBool(CombinedKey, false);
            combinedAgent?.Teardown(); combinedAgent?.Teardown();
            if (combinedNode != null)
            {
                if (scanSubscription != null) combinedNode.RemoveSubscription<PointCloud2>(scanSubscription);
                if (boxSubscription != null) combinedNode.RemoveSubscription<autoware_perception_msgs.msg.DetectedObjects>(boxSubscription);
                if (scanPublisher != null) combinedNode.RemovePublisher<PointCloud2>(scanPublisher);
                ROS2_Bridge.ROS_CORE.RemoveNode(combinedNode);
            }
            ConfigurationManager.SetActiveAgent(null);
            UnityEngine.Object.Destroy(agentObject); UnityEngine.Object.Destroy(virtualObject);
            UnityEngine.Object.Destroy(sdfObject); UnityEngine.Object.Destroy(moduleObject); UnityEngine.Object.Destroy(sphere);
            EditorApplication.Exit(status);
        }

        public static void Run()
        {
            try
            {
                var core = new ROS2UnityCore();
                if (!core.Ok()) throw new Exception("ROS initialization failed.");
                var node = core.CreateNode("ub_mr_recognition_test");
                var qos = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);
                var passthrough = new LidarPassthrough("/ub_mr_test/scan", node, qos);
                var publisher = node.CreatePublisher<PointCloud2>("/ub_mr_test/scan");
                PointCloud2 received = null;
                using var delivered = new AutoResetEvent(false);
                var subscription = node.CreateSubscription<PointCloud2>("/ub_mr_test/scan_modified", message =>
                {
                    received = message;
                    delivered.Set();
                });
                var original = new PointCloud2
                {
                    Header = new std_msgs.msg.Header { Frame_id = "test_lidar", Stamp = new builtin_interfaces.msg.Time
                        { Sec = 456, Nanosec = 987654321 } },
                    Height = 2, Width = 2, Point_step = 16, Row_step = 36,
                    Is_bigendian = true, Is_dense = false,
                    Fields = new[] { new PointField { Name = "x", Offset = 0, Datatype = PointField.FLOAT32, Count = 1 },
                        new PointField { Name = "ring", Offset = 12, Datatype = PointField.UINT16, Count = 1 } },
                    Data = Enumerable.Range(0, 72).Select(i => (byte)(i * 3)).ToArray()
                };
                var deadline = DateTime.UtcNow.AddSeconds(10);
                while (DateTime.UtcNow < deadline && received == null)
                {
                    publisher.Publish(original);
                    Thread.Sleep(30);
                    passthrough.Publish();
                    delivered.WaitOne(30);
                }
                if (received == null) throw new Exception("Passthrough DDS delivery timed out.");
                if (!received.Data.SequenceEqual(original.Data) || received.Header.Frame_id != original.Header.Frame_id ||
                    received.Header.Stamp.Sec != original.Header.Stamp.Sec || received.Header.Stamp.Nanosec != original.Header.Stamp.Nanosec ||
                    received.Height != original.Height || received.Width != original.Width || received.Point_step != original.Point_step ||
                    received.Row_step != original.Row_step || received.Is_bigendian != original.Is_bigendian || received.Is_dense != original.Is_dense ||
                    received.Fields.Length != original.Fields.Length || received.Fields[1].Name != "ring" ||
                    received.Fields[1].Offset != 12 || received.Fields[1].Datatype != PointField.UINT16)
                    throw new Exception("Passthrough changed the scan or metadata.");
                passthrough.CleanUp();
                passthrough.CleanUp();
                passthrough.Publish();

                var clock = new DetectionClock(node, true, 30);
                if (clock.TrySample(out _)) throw new Exception("Simulation clock published before receiving time.");
                var clockPublisher = node.CreatePublisher<rosgraph_msgs.msg.Clock>("/clock");
                var tick = new rosgraph_msgs.msg.Clock { Clock_ = new builtin_interfaces.msg.Time { Sec = 10, Nanosec = 123456789 } };
                builtin_interfaces.msg.Time stamp = null;
                deadline = DateTime.UtcNow.AddSeconds(10);
                while (DateTime.UtcNow < deadline && stamp == null)
                {
                    clockPublisher.Publish(tick);
                    Thread.Sleep(30);
                    clock.TrySample(out stamp);
                }
                if (stamp == null || stamp.Sec != 10 || stamp.Nanosec != 123456789)
                    throw new Exception("Simulation clock stamp was not preserved.");
                if (clock.TrySample(out _)) throw new Exception("Paused clock published twice.");
                foreach (int seconds in new[] { 5, 50 })
                {
                    stamp = null;
                    tick.Clock_.Sec = seconds;
                    deadline = DateTime.UtcNow.AddSeconds(5);
                    while (DateTime.UtcNow < deadline && stamp == null)
                    {
                        clockPublisher.Publish(tick); Thread.Sleep(30); clock.TrySample(out stamp);
                    }
                    if (stamp == null || stamp.Sec != seconds || clock.TrySample(out _))
                        throw new Exception("ROS clock jump/pause scheduling failed.");
                }
                clock.CleanUp();
                clock.CleanUp();
                node.RemoveSubscription<PointCloud2>(subscription);
                node.RemovePublisher<PointCloud2>(publisher);
                node.RemovePublisher<rosgraph_msgs.msg.Clock>(clockPublisher);
                core.RemoveNode(node);
                Debug.Log("UB-MR ROS checks PASSED: byte-exact PointCloud2 passthrough, simulation clock, idempotent cleanup.");
                EditorApplication.Exit(0);
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                EditorApplication.Exit(1);
            }
        }
    }
}
