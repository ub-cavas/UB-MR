using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Security.Cryptography;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UB_MR.Redis_Networking;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Object = UnityEngine.Object;

namespace CAVAS.UB_MR.Tests
{
    /// <summary>
    /// Unity -batchmode -nographics -projectPath UB-MR
    /// -executeMethod CAVAS.UB_MR.Tests.CarlaMapFrameChecks.Run -logFile /tmp/carla-map-frame.log
    /// Uses only temporary, inactive objects in an unsaved empty scene; never starts ROS or CARLA.
    /// </summary>
    [InitializeOnLoad]
    public static class CarlaMapFrameChecks
    {
        const string Key = "UBMR.CarlaMapFrameChecks";
        const string ScenePath = "Assets/UB_MR/Modules/UB-Service-Center-Loop/UB-Service-Center-Loop.unity";
        const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
        static readonly Quaternion Baseline = Quaternion.Euler(-90f, -90f, -180f);

        static CarlaMapFrameChecks()
        {
            EditorApplication.playModeStateChanged += state =>
            {
                if (state == PlayModeStateChange.EnteredPlayMode && SessionState.GetBool(Key, false))
                    EditorApplication.delayCall += RunInPlayMode;
            };
        }

        public static void Run()
        {
            try
            {
                if (Application.isPlaying) throw new Exception("Start these checks in edit mode.");
                SessionState.SetString(Key + ".scene", Hash(ScenePath));
                SessionState.SetString(Key + ".meta", Hash(ScenePath + ".meta"));
                SessionState.SetString(Key + ".scenes", SceneFiles());
                using (var fixture = new Fixture())
                {
                    Check(CarlaMapFrame.GetOrCreate(fixture.Module) == null, "Frame created in edit mode.");
                    Check(fixture.Module.GetComponent<CarlaMapFrame>() == null, "Edit-mode scene was mutated.");
                }
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
                SessionState.SetBool(Key, true);
                EditorApplication.EnterPlaymode();
            }
            catch (Exception e) { Finish(e); }
        }

        static void RunInPlayMode()
        {
            try
            {
                Geometry();
                MapUpdateTiming();
                RuntimeConsumers();
                OptionalCarlaSmoke();
                Check(Hash(ScenePath) == SessionState.GetString(Key + ".scene", ""), "Protected scene changed.");
                Check(Hash(ScenePath + ".meta") == SessionState.GetString(Key + ".meta", ""), "Protected scene meta changed.");
                Check(SceneFiles() == SessionState.GetString(Key + ".scenes", ""), "Scene assets were created or removed.");
                Finish(null);
            }
            catch (Exception e) { Finish(e); }
        }

        static void Geometry()
        {
            using var fixture = new Fixture();
            CarlaMapFrame frame = CarlaMapFrame.GetOrCreate(fixture.Module);
            Check(frame == CarlaMapFrame.GetOrCreate(fixture.Module), "Frame was not reused.");
            Check(fixture.Module.GetComponents<CarlaMapFrame>().Length == 1, "Duplicate shared frames.");
            Near(Get<Vector3>(frame, "originOffset"), Vector3.zero, "Default offset");
            fixture.Root.rotation = Baseline;

            foreach (float yaw in new[] { 0f, 90f, -90f, 180f, -180f, 359f })
            {
                Check(frame.TryCarlaPoseToUnityWorld(new Vector3(10, 20, 3), yaw, out Pose pose), "Baseline conversion failed.");
                Near(pose.position, new Vector3(20, 3, 10), "Axis permutation");
                Near(pose.rotation * Vector3.forward, Quaternion.Euler(0, yaw, 0) * Vector3.forward, "Cardinal heading");
                RoundTrip(frame, new Vector3(10, 20, 3), yaw);
            }

            // Quaternion stored on the protected scene's imported map root (x,y,z,w).
            fixture.Root.rotation = new Quaternion(0.50520855f, -0.49473667f, -0.49473667f, -0.50520855f);
            Check(frame.TryCarlaPoseToUnityWorld(new Vector3(100, 0, 0), 0, out Pose aligned), "Scene conversion failed.");
            Near(aligned.position, Quaternion.Euler(0, -1.2f, 0) * new Vector3(0, 0, 100), "Scene alignment");
            Heading(aligned.rotation.eulerAngles.y, -1.2f, "Scene heading");
            RoundTrip(frame, new Vector3(100, 0, 0), 0);

            fixture.Root.position = new Vector3(41, 8, -73);
            fixture.Root.rotation = Quaternion.Euler(7, 32, -4) * Baseline;
            fixture.Root.localScale = new Vector3(2, 3, 4); // Rigid frame ignores mesh scale.
            Set(frame, "originOffset", new Vector3(1.347f, 2, 5.916f));
            Check(CarlaMapFrame.GetOrCreate(fixture.Module) == frame, "Configured frame replaced.");
            Near(Get<Vector3>(frame, "originOffset"), new Vector3(1.347f, 2, 5.916f), "Configured offset overwritten");
            foreach (Vector3 location in new[] { Vector3.zero, new Vector3(300, -120, 2), new Vector3(-400, 250, -6) })
                foreach (float yaw in new[] { -179f, -90f, 0f, 42f, 179f })
                    RoundTrip(frame, location, yaw);

            Vector3 carlaPosition = new(130, -50, 4);
            Check(frame.TryCarlaPoseToUnityWorld(carlaPosition, 71, out Pose ego), "Initial pose failed.");
            Quaternion shiftRotation = Quaternion.Euler(-3, 83, 6);
            Vector3 shiftPosition = new(50, -3, 20);
            fixture.Root.SetPositionAndRotation(shiftPosition + shiftRotation * fixture.Root.position,
                shiftRotation * fixture.Root.rotation);
            Check(frame.TryUnityWorldToCarlaPose(shiftPosition + shiftRotation * ego.position,
                shiftRotation * ego.rotation, out Vector3 invariant, out float invariantYaw), "Rigid invariant failed.");
            Near(invariant, carlaPosition, "Rigid invariant position");
            Heading(invariantYaw, 71, "Rigid invariant heading");

            fixture.Root.SetPositionAndRotation(Vector3.zero, Baseline);
            Check(!frame.TryUnityWorldToCarlaPose(Vector3.zero, Quaternion.Euler(90, 0, 0), out _, out _),
                "Undefined vertical heading was accepted.");
            Check(frame.TryUnityWorldToCarlaPose(Vector3.zero, Quaternion.Euler(12, 43, 8), out _, out float planarYaw),
                "Tilted ego heading rejected.");
            Heading(planarYaw, 43, "Yaw-only tilted vehicle");
        }

        static void RuntimeConsumers()
        {
            using var fixture = new Fixture();
            fixture.Root.rotation = Baseline;
            GameObject network = fixture.NewInactive("Network checks");
            var receiver = network.AddComponent<TrafficReceiver>();
            var renderer = network.AddComponent<TrafficRenderer>();
            var publisher = network.AddComponent<EgoPublisher>();
            var prefab = fixture.NewInactive("Traffic prefab");
            var ego = fixture.NewInactive("Ego override").transform;
            Set(renderer, "receiver", receiver);
            Set(renderer, "vehiclePrefab", prefab);
            Set(renderer, "module", fixture.Module);
            Set(publisher, "module", fixture.Module);
            Set(publisher, "egoTransformOverride", ego);
            Set(publisher, "egoId", "map-frame-check");
            Set(publisher, "bridgeHost", "127.0.0.1");
            using var listener = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
            listener.Client.ReceiveTimeout = 2000;
            Set(publisher, "bridgePort", ((IPEndPoint)listener.Client.LocalEndPoint).Port);
            Invoke(publisher, "Awake");
            Invoke(renderer, "Awake");
            Invoke(renderer, "OnEnable");
            Invoke(publisher, "Start");
            try
            {
                Check(Get<CarlaMapFrame>(publisher, "_mapFrame") == Get<CarlaMapFrame>(renderer, "_mapFrame"),
                    "Consumers did not share the runtime frame.");
                Check(fixture.Module.GetComponents<CarlaMapFrame>().Length == 1, "Consumers created duplicate frames.");

                ReceiveTraffic(receiver, 10, 20, 3, 45);
                Invoke(renderer, "LateUpdate");
                var vehicles = Get<IDictionary>(renderer, "spawnedVehicles");
                GameObject vehicle = VehicleRoot(vehicles["traffic-check"]);
                Near(vehicle.transform.position, new Vector3(20, 3, 10), "Traffic initial pose");
                Check(vehicle.activeSelf, "Valid traffic remained hidden.");

                // No new traffic packet: alignment alone must move the cached vehicle.
                fixture.Root.SetPositionAndRotation(new Vector3(8, 2, -4), Quaternion.Euler(0, 30, 0) * Baseline);
                Invoke(renderer, "LateUpdate");
                Near(vehicle.transform.position, new Vector3(8, 2, -4) + Quaternion.Euler(0, 30, 0) * new Vector3(20, 3, 10),
                    "Cached traffic ignored map update");
                Heading(vehicle.transform.eulerAngles.y, 75, "Cached traffic heading");
                ego.SetPositionAndRotation(vehicle.transform.position, vehicle.transform.rotation);
                JObject packet = PublishAndReceive(publisher, listener);
                CheckPayload(packet, new Vector3(10, 20, 3), 45);
                Invoke(publisher, "LateUpdate");
                Check(!listener.Client.Poll(50000, SelectMode.SelectRead), "Publish rate limit was bypassed.");

                int unavailable = 0, recovered = 0;
                Application.LogCallback logs = (message, trace, type) =>
                {
                    if (message.StartsWith("[CarlaMapFrame] Map reference unavailable")) unavailable++;
                    if (message.StartsWith("[CarlaMapFrame] Map reference recovered")) recovered++;
                };
                Application.logMessageReceived += logs;
                try
                {
                    Set(fixture.Module, "map_root", null);
                    for (int i = 0; i < 2; i++)
                    {
                        Set(publisher, "_nextSendTime", 0f);
                        Invoke(publisher, "LateUpdate");
                        Invoke(renderer, "LateUpdate");
                    }
                    Check(!listener.Client.Poll(50000, SelectMode.SelectRead), "Published without a map.");
                    Check(!vehicle.activeSelf, "Traffic stayed visible without a map.");
                    Set(fixture.Module, "map_root", fixture.Root);
                    CheckPayload(PublishAndReceive(publisher, listener), new Vector3(10, 20, 3), 45);
                    Invoke(renderer, "LateUpdate");
                    Check(vehicle.activeSelf, "Traffic did not recover.");
                    Check(unavailable == 1 && recovered == 1, "Availability messages were not transition-only.");
                }
                finally { Application.logMessageReceived -= logs; }

                // A replaced module must be discovered on the next attempt.
                Object.DestroyImmediate(fixture.Module);
                Set(publisher, "_nextSendTime", 0f);
                Invoke(publisher, "LateUpdate");
                Check(!listener.Client.Poll(50000, SelectMode.SelectRead), "Published without a module.");
                // Activate a disabled Module so scene discovery finds it without starting ROS.
                var replacementObject = fixture.NewInactive("Replacement module");
                var replacement = replacementObject.AddComponent<Module>();
                replacement.enabled = false;
                Set(replacement, "map_root", fixture.Root);
                replacementObject.SetActive(true);
                CheckPayload(PublishAndReceive(publisher, listener), new Vector3(10, 20, 3), 45);
                Invoke(renderer, "LateUpdate");
                Check(Get<CarlaMapFrame>(publisher, "_mapFrame") == Get<CarlaMapFrame>(renderer, "_mapFrame"),
                    "Consumers did not recover onto the same module.");

                ReceiveTraffic(receiver, 12, -4, 1, -90);
                Invoke(renderer, "LateUpdate");
                ego.SetPositionAndRotation(vehicle.transform.position, vehicle.transform.rotation);
                CheckPayload(PublishAndReceive(publisher, listener), new Vector3(12, -4, 1), -90);
                // Exercise the existing receiver's despawn event.
                ReceiveTraffic(receiver);
                Check(vehicles.Count == 0, "Traffic despawn did not clear the replica.");
            }
            finally
            {
                foreach (var vehicle in Get<IDictionary>(renderer, "spawnedVehicles").Values)
                    Object.DestroyImmediate(VehicleRoot(vehicle));
                Get<IDictionary>(renderer, "spawnedVehicles").Clear();
                Invoke(renderer, "OnDisable");
                Invoke(publisher, "OnDestroy");
            }
        }

        static void MapUpdateTiming()
        {
            using var fixture = new Fixture();
            var panel = fixture.NewInactive("Map panel checks").AddComponent<MapPanel>();
            foreach (string field in new[] { "pos_x", "pos_y", "pos_z", "rot_x", "rot_y", "rot_z" })
            {
                GameObject input = fixture.NewInactive(field);
                input.AddComponent<RectTransform>();
                Set(panel, field, input.AddComponent<TMPro.TMP_InputField>());
            }
            Set(fixture.Module, "mapPanel", panel);
            panel.SetMapPosition(new Vector3(12, 3, -8));
            panel.SetMapRotation(new Vector3(-90, -91.2f, -180));
            Invoke(fixture.Module, "Update");
            Near(fixture.Root.position, new Vector3(12, 3, -8), "Map UI position");
            Check(Quaternion.Angle(fixture.Root.rotation, Quaternion.Euler(-90, -91.2f, -180)) < 0.01f,
                "Map rotation was not applied during Update.");
            Check(fixture.Module.HasMapRotationState, "Map UI state not initialized.");
            Near(fixture.Module.CurrentMapRotationEuler, new Vector3(-90, -91.2f, -180), "Map UI state");
            Check(typeof(Module).GetCustomAttribute<DefaultExecutionOrder>().order == -1000,
                "Module no longer updates before pose consumers.");
        }

        // Optional integration hook: an isolated CARLA verifier supplies poses.json and
        // acknowledges each rendered pose with <index>.ok in this temporary directory.
        // No server connections occur unless both environment variables are provided.
        static void OptionalCarlaSmoke()
        {
            string directory = Environment.GetEnvironmentVariable("UB_CARLA_FRAME_SMOKE_DIRECTORY");
            if (string.IsNullOrEmpty(directory)) return;
            int port = int.Parse(Environment.GetEnvironmentVariable("UB_CARLA_FRAME_SMOKE_UDP_PORT"));
            var cases = JArray.Parse(File.ReadAllText(Path.Combine(directory, "poses.json")));
            using var fixture = new Fixture();
            var ego = fixture.NewInactive("Smoke ego").transform;
            var publisher = fixture.NewInactive("Smoke publisher").AddComponent<EgoPublisher>();
            Set(publisher, "module", fixture.Module);
            Set(publisher, "egoTransformOverride", ego);
            Set(publisher, "egoId", "map-frame-smoke");
            Set(publisher, "bridgeHost", "127.0.0.1");
            Set(publisher, "bridgePort", port);
            Invoke(publisher, "Awake");
            Invoke(publisher, "Start");
            try
            {
                for (int i = 0; i < cases.Count; i++)
                {
                    JToken test = cases[i];
                    Vector3 location = test["location"].ToObject<Vector3>();
                    Vector3 translation = test["translation"].ToObject<Vector3>();
                    Quaternion alignment = Quaternion.Euler(0, (float)test["alignmentYaw"], 0);
                    fixture.Root.SetPositionAndRotation(translation, alignment * Baseline);
                    ego.SetPositionAndRotation(translation + alignment * new Vector3(location.y, location.z, location.x),
                        alignment * Quaternion.Euler(0, (float)test["yaw"], 0));
                    // Existing color field identifies fresh packets without changing the protocol.
                    Set(publisher, "vehicleColor", $"{i},0,0");
                    string acknowledgement = Path.Combine(directory, i + ".ok");
                    DateTime deadline = DateTime.UtcNow.AddSeconds(20);
                    do
                    {
                        Set(publisher, "_nextSendTime", 0f);
                        Invoke(publisher, "LateUpdate");
                        System.Threading.Thread.Sleep(100);
                    } while (!File.Exists(acknowledgement) && DateTime.UtcNow < deadline);
                    Check(File.Exists(acknowledgement), $"CARLA did not acknowledge smoke case {i}.");
                }
                Debug.Log($"CARLA end-to-end smoke acknowledged {cases.Count} map-relative poses.");
            }
            finally { Invoke(publisher, "OnDestroy"); }
        }

        static JObject PublishAndReceive(EgoPublisher publisher, UdpClient listener)
        {
            Set(publisher, "_nextSendTime", 0f);
            Invoke(publisher, "LateUpdate");
            IPEndPoint sender = new(IPAddress.Any, 0);
            return JObject.Parse(Encoding.UTF8.GetString(listener.Receive(ref sender)));
        }

        static void CheckPayload(JObject packet, Vector3 location, float yaw)
        {
            Check(string.Join(",", packet.Properties().Select(p => p.Name).OrderBy(p => p)) == "blueprint,color,id,location,yaw",
                "Wire fields changed.");
            Check((string)packet["id"] == "map-frame-check" && (string)packet["blueprint"] == "vehicle.lincoln.mkz_2017"
                && (string)packet["color"] == "0,0,0", "Vehicle identity changed.");
            Near(new Vector3((float)packet["location"]["x"], (float)packet["location"]["y"], (float)packet["location"]["z"]),
                location, "UDP location");
            Heading((float)packet["yaw"], yaw, "UDP yaw");
        }

        static void ReceiveTraffic(TrafficReceiver receiver, float x = 0, float y = 0, float z = 0, float? yaw = null)
        {
            var vehicles = yaw.HasValue ? new[] { new { id = "traffic-check", location = new { x, y, z }, yaw = yaw.Value } } : Array.Empty<object>();
            Type payloadType = typeof(TrafficReceiver).GetNestedType("TrafficPayload", BindingFlags.NonPublic);
            object payload = JsonConvert.DeserializeObject(JsonConvert.SerializeObject(new { vehicles }), payloadType);
            Invoke(receiver, "ProcessPayload", payload);
        }

        static void RoundTrip(CarlaMapFrame frame, Vector3 location, float yaw)
        {
            Check(frame.TryCarlaPoseToUnityWorld(location, yaw, out Pose pose), "Forward conversion failed.");
            Check(frame.TryUnityWorldToCarlaPose(pose.position, pose.rotation, out Vector3 result, out float resultYaw), "Inverse conversion failed.");
            Near(result, location, "Round-trip position");
            Heading(resultYaw, yaw, "Round-trip yaw");
            Check(resultYaw >= -180 && resultYaw < 180, "Yaw not normalized.");
        }

        sealed class Fixture : IDisposable
        {
            readonly List<GameObject> objects = new();
            public Module Module { get; }
            public Transform Root { get; }
            public Fixture()
            {
                Module = NewInactive("Map module checks").AddComponent<Module>();
                Root = NewInactive("Map root checks").transform;
                Set(Module, "map_root", Root);
            }
            public GameObject NewInactive(string name)
            {
                var go = new GameObject(name);
                go.SetActive(false);
                objects.Add(go);
                return go;
            }
            public void Dispose()
            {
                for (int i = objects.Count - 1; i >= 0; i--)
                    if (objects[i] != null) Object.DestroyImmediate(objects[i]);
            }
        }

        static GameObject VehicleRoot(object instance) => (GameObject)instance.GetType().GetField("root").GetValue(instance);
        static void Set(object target, string name, object value) => target.GetType().GetField(name, Private).SetValue(target, value);
        static T Get<T>(object target, string name) => (T)target.GetType().GetField(name, Private).GetValue(target);
        static void Invoke(object target, string name, params object[] args) => target.GetType().GetMethod(name, Private).Invoke(target, args);
        static void Near(Vector3 actual, Vector3 expected, string label) => Check(Vector3.Distance(actual, expected) <= 0.001f, $"{label}: {actual} != {expected}");
        static void Heading(float actual, float expected, string label) => Check(Mathf.Abs(Mathf.DeltaAngle(actual, expected)) <= 0.01f, $"{label}: {actual} != {expected}");
        static void Check(bool condition, string message) { if (!condition) throw new Exception(message); }
        static string Hash(string path) { using var sha = SHA256.Create(); return Convert.ToBase64String(sha.ComputeHash(File.ReadAllBytes(path))); }
        static string SceneFiles() => string.Join("\n", Directory.GetFiles("Assets", "*.unity", SearchOption.AllDirectories).OrderBy(p => p));
        static void Finish(Exception exception)
        {
            SessionState.SetBool(Key, false);
            if (exception != null) Debug.LogException(exception);
            else Debug.Log("UB-MR CARLA map frame checks PASSED: geometry, runtime sharing, cached traffic, UDP, recovery, scene preservation.");
            EditorApplication.Exit(exception == null ? 0 : 1);
        }
    }
}
