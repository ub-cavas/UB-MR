using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using CAVAS.UB_MR.Config;
using CAVAS.UB_MR.DT.Sensors;
using Newtonsoft.Json;
using UB_MR.Redis_Networking;
using UnityEngine;

namespace CAVAS.UB_MR.Tests
{
    /// <summary>Opt-in replay harness used by the Editor batch runner and its standalone test scene.</summary>
    public sealed class TrafficPlaybackChecks : MonoBehaviour
    {
        public TrafficVehicleCatalog catalog;
        public GameObject fallback;
        public bool gpu;
        private TrafficReceiver receiver;
        private TrafficRenderer renderer;
        private UdpClient sender;
        private int port;
        private GameObject network;
        private Transform mapRoot;
        private readonly List<RenderTexture> ownedTextures = new();
        private int sequence;

        private IEnumerator Start()
        {
            var stack = new Stack<IEnumerator>();
            stack.Push(RunChecks());
            while (stack.Count > 0)
            {
                var checks = stack.Peek();
                object next = null;
                bool done;
                try { done = !checks.MoveNext(); if (!done) next = checks.Current; }
                catch (Exception error) { Debug.LogException(error); Finish(1); yield break; }
                if (done) { stack.Pop(); continue; }
                if (next is IEnumerator nested) { stack.Push(nested); continue; }
                yield return next;
            }
            Debug.Log("Traffic playback checks PASSED: UDP selection, colors, lifecycle, coordinates, perception, 30-actor resource cycles" + (gpu ? ", GPU SDF hits." : "."));
            Finish(0);
        }

        private void Finish(int code)
        {
            sender?.Dispose();
            if (network != null) network.SetActive(false);
            ConfigurationManager.SetActiveAgent(null);
#if UNITY_EDITOR
            UnityEditor.EditorApplication.Exit(code);
#else
            Application.Quit(code);
#endif
        }

        private IEnumerator RunChecks()
        {
            ConfigurationManager.SetActiveAgent(new Config.Agent { recognition = new VirtualObjectRecognitionSettings
                { mode = gpu ? VirtualObjectRecognitionMode.LidarModification : VirtualObjectRecognitionMode.BoundingBoxInjection } });
            VirtualBoundingBoxDetector.ClearVirtualObjectDatabase();
            network = new GameObject("Traffic test network");
            network.SetActive(false);
            receiver = network.AddComponent<TrafficReceiver>();
            Set(receiver, "listenPort", 0);
            renderer = network.AddComponent<TrafficRenderer>();
            Set(renderer, "receiver", receiver);
            Set(renderer, "vehicleCatalog", catalog);
            Set(renderer, "vehiclePrefab", fallback);
            mapRoot = new GameObject("Traffic test map frame").transform;
            mapRoot.rotation = Quaternion.Euler(-90, -90, -180);
            var module = network.AddComponent<Module>();
            module.enabled = false; // Bind the map without starting ROS.
            Set(module, "map_root", mapRoot);
            Set(renderer, "module", module);
            network.SetActive(true);
            yield return null;
            port = ((IPEndPoint)Get<UdpClient>(receiver, "_udpClient").Client.LocalEndPoint).Port;
            sender = new UdpClient();
            var data = new[] { Vehicle("a", "vehicle.audi.a2", "255,0,0"), Vehicle("b", "vehicle.audi.a2", "0,0,255"),
                Vehicle("m", "vehicle.ford.mustang", "0,255,0"), Vehicle("u", "vehicle.unknown.test", "255,0,0"),
                Vehicle("empty", "", null), Vehicle("missing", null, null) };
            yield return Send(data);
            Check(Count == 6, "Mixed snapshot instance count");
            Check(Root("a").GetComponent<TrafficVehicleAppearance>() != null, "Audi selection");
            Check(Root("m").transform.Find("Alignment/Mustang_Textured") != null, "Mustang selection");
            Check(Root("u").GetComponent<TrafficVehicleAppearance>() == null, "Unknown must use Jeep fallback");
            var red = Paint(Root("a"));
            var blue = Paint(Root("b"));
            Check(red != blue && red.GetColor("_BaseColor").r == 1 && blue.GetColor("_BaseColor").b == 1, "Independent paint");
            var source = Paint(catalog.Entries[0].prefab);
            Check(source != red && source != blue, "Source material mutated");
            var untouched = NonPaint(Root("m"));
            var oldAudi = Root("a");
            yield return Send(data);
            Check(Root("a") == oldAudi && Paint(Root("a")) == red, "Repeated snapshot duplicated objects/materials");
            data[0].color = " 0, 255, 0 ";
            yield return Send(data);
            Check(Paint(Root("b")) == blue && Paint(Root("a")).GetColor("_BaseColor").g == 1, "Color update isolation");
            Check(untouched.SequenceEqual(NonPaint(Root("m"))), "Non-paint materials changed");
            data[0].color = "256,0,0";
            yield return Send(data);
            Check(Paint(Root("a")) == source, "Invalid color must restore authored material");
            data[0].color = null;
            yield return Send(data);
            Check(Paint(Root("a")) == source, "Missing color must restore authored material");
            data[0].blueprint = "vehicle.ford.mustang";
            yield return Send(data);
            Check(oldAudi == null || !oldAudi.activeInHierarchy, "Replaced object still active");
            Check(Root("a").transform.Find("Alignment/Mustang_Textured") != null && Count == 6, "Blueprint replacement");
            foreach (float yaw in new[] { 0f, 90f, 180f, 270f })
            {
                data[0].yaw = yaw;
                data[0].location = new TrafficReceiver.LocationData { x = 4, y = 3, z = 2 };
                yield return Send(data);
                Near(Root("a").transform.position, new Vector3(3, 2, 4), "Position basis");
                Check(Quaternion.Angle(Root("a").transform.rotation, Quaternion.Euler(0, yaw, 0)) < 0.01f, "Yaw conversion");
            }
            mapRoot.rotation = Quaternion.Euler(0, 30, 0) * Quaternion.Euler(-90, -90, -180);
            data[0].yaw = 0;
            yield return Send(data);
            Near(Root("a").transform.position, Quaternion.Euler(0, 30, 0) * new Vector3(3, 2, 4), "Map yaw position");
            Check(Quaternion.Angle(Root("a").transform.rotation, Quaternion.Euler(0, 30, 0)) < 0.01f, "Map yaw orientation");
            mapRoot.rotation = Quaternion.Euler(-90, -90, -180);
            var blueVehicle = Root("b");
            Set(module, "map_root", null);
            yield return null;
            Check(!blueVehicle.activeSelf && Detections == 0, "Missing map must hide traffic");
            Set(module, "map_root", mapRoot);
            yield return null;
            Check(blueVehicle.activeSelf && Paint(blueVehicle).GetColor("_BaseColor").b == 1,
                "Map recovery must restore vehicle paint without a new packet");
            Check(Detections == 6, "Perception registration");
            foreach (var obj in FindObjectsByType<VirtualObject>(FindObjectsSortMode.None))
            {
                Check(obj.Classification == VirtualObjectClassification.Car, "Classification");
                Check(obj.TryGetBoundingBox(out _, out _, out var size) && size.z > 3, "Vehicle bounding box");
                if (!gpu) Check(!obj.GetComponentInChildren<MeshToSDF>().enabled, "SDF enabled in bounding-box mode");
            }
            renderer.enabled = false;
            Check(Detections == 0, "Disable must immediately remove perception registrations");
            yield return null;
            Check(FindObjectsByType<VirtualObject>(FindObjectsSortMode.None).Length == 0, "Disable left instances");
            data[1].color = "0,200,200";
            yield return Send(data);
            Check(Count == 0 && Detections == 0, "Disabled renderer processed a snapshot");
            renderer.enabled = true;
            yield return null; // Poses and visibility are restored in LateUpdate.
            Check(Count == 6 && Detections == 6, "Re-enable must reconcile KnownVehicles");
            Check(Mathf.Abs(Paint(Root("b")).GetColor("_BaseColor").g - 200f / 255f) < 0.001f,
                "Re-enable used stale appearance");
            yield return Send(Array.Empty<TrafficReceiver.VehicleData>());
            Check(Count == 0 && Detections == 0, "Empty snapshot must despawn all");
            // Missing fallback and invalid catalog should log once and skip/fallback without exceptions.
            Set(renderer, "vehiclePrefab", null);
            yield return Send(new[] { Vehicle("skipped", "vehicle.missing-fallback", null) });
            Check(Count == 0, "Missing fallback should skip actor");
            Set(renderer, "vehiclePrefab", fallback);
            yield return Send(Array.Empty<TrafficReceiver.VehicleData>());
            renderer.enabled = false;
            Set(renderer, "vehicleCatalog", null);
            renderer.enabled = true;
            yield return Send(new[] { Vehicle("catalog-fallback", "vehicle.audi.a2", "255,0,0") });
            Check(Count == 1 && Root("catalog-fallback").GetComponent<TrafficVehicleAppearance>() == null,
                "Absent catalog must use fallback");
            renderer.enabled = false;
            Set(renderer, "vehicleCatalog", catalog);
            renderer.enabled = true;
            yield return null;
            Check(Root("catalog-fallback").GetComponent<TrafficVehicleAppearance>() != null,
                "Catalog recovery failed");
            yield return Send(Array.Empty<TrafficReceiver.VehicleData>());
            for (int cycle = 0; cycle < 3; cycle++)
            {
                var fleet = Enumerable.Range(0, 30).Select(i => Vehicle("cycle" + i,
                    i % 2 == 0 ? "vehicle.audi.a2" : "vehicle.ford.mustang", $"{cycle * 80},{i * 8},150")).ToArray();
                yield return Send(fleet);
                Check(Count == 30 && Detections == 30, "30-actor spawn");
                if (gpu)
                {
                    yield return new WaitForSeconds(0.5f);
                    var frame = RenderFrame();
                    frame.Release(); Destroy(frame);
                    foreach (var obj in FindObjectsByType<VirtualObject>(FindObjectsSortMode.None))
                        ownedTextures.Add(Get<RenderTexture>(obj, "ownedSdf"));
                }
                yield return Send(Array.Empty<TrafficReceiver.VehicleData>());
                yield return null;
                Check(Count == 0 && Cache.Count == 0 && Detections == 0, "Resource cycle did not return to baseline");
                Check(ownedTextures.All(t => t == null), "SDF texture leaked");
            }
            if (gpu)
            {
                yield return Send(new[] { Vehicle("audi", "vehicle.audi.a2", "255,0,0"), Vehicle("mustang", "vehicle.ford.mustang", "0,80,255") });
                Root("mustang").transform.position = new Vector3(4, 0, 0);
                yield return new WaitForSeconds(1);
                var camera = Camera.main;
                camera.transform.position = new Vector3(9, 6, 10);
                camera.transform.LookAt(new Vector3(2, 0.6f, 0));
                var target = RenderFrame();
                var previous = RenderTexture.active;
                RenderTexture.active = target;
                var picture = new Texture2D(1200, 700, TextureFormat.RGB24, false);
                picture.ReadPixels(new Rect(0, 0, 1200, 700), 0, 0);
                picture.Apply();
                System.IO.File.WriteAllBytes("/tmp/ubmr-traffic-preview.png", picture.EncodeToPNG());
                RenderTexture.active = previous;
                camera.targetTexture = null;
                target.Release();
                Destroy(target); Destroy(picture);
                CheckSdfHit(Root("audi"));
                CheckSdfHit(Root("mustang"));
                yield return Send(Array.Empty<TrafficReceiver.VehicleData>());
            }
        }

        private static RenderTexture RenderFrame()
        {
            // Batch Play Mode has no visible Game view; explicitly request a complete URP frame.
            var target = new RenderTexture(1200, 700, 24);
            UnityEngine.Rendering.RenderPipeline.SubmitRenderRequest(Camera.main,
                new UnityEngine.Rendering.RenderPipeline.StandardRequest { destination = target });
            return target;
        }

        private IEnumerator Send(TrafficReceiver.VehicleData[] vehicles)
        {
            string json = JsonConvert.SerializeObject(new { timestamp = ++sequence, vehicles });
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            sender.Send(bytes, bytes.Length, new IPEndPoint(IPAddress.Loopback, port));
            float deadline = Time.realtimeSinceStartup + 5;
            // Always allow the receive thread and main-thread queue at least one frame.
            yield return null;
            while (receiver.KnownVehicles.Count != vehicles.Length || vehicles.Any(v =>
                !receiver.KnownVehicles.TryGetValue(v.id, out var known) || known.blueprint != v.blueprint
                || known.color != v.color || known.yaw != v.yaw || known.location.x != v.location.x))
            {
                Check(Time.realtimeSinceStartup < deadline, "UDP snapshot timeout");
                yield return null;
            }
            // The snapshot is consumed in Update; wait for its map-relative LateUpdate pose.
            yield return null;
        }

        private void CheckSdfHit(GameObject root)
        {
            var obj = root.GetComponent<VirtualObject>();
            var sdf = obj.GetComponentInChildren<SDFTexture>();
            var box = root.GetComponent<BoxCollider>();
            Check(sdf.sdf is RenderTexture rt && rt.IsCreated(), "SDF texture was not created");
            var readback = UnityEngine.Rendering.AsyncGPUReadback.Request(sdf.sdf, 0);
            readback.WaitForCompletion();
            Check(!readback.hasError, "SDF GPU readback failed");
            float minimum = float.PositiveInfinity, maximum = float.NegativeInfinity;
            for (int layer = 0; layer < readback.layerCount; layer++)
                foreach (ushort bits in readback.GetData<ushort>(layer))
                {
                    float distance = Mathf.HalfToFloat(bits);
                    minimum = Mathf.Min(minimum, distance); maximum = Mathf.Max(maximum, distance);
                }
            Check(minimum < 0 && maximum > 0, "SDF must contain both interior and exterior distances: " + root.name);
            var origin = root.transform.TransformPoint(box.center - Vector3.forward * (box.size.z * 0.5f + 2));
            // Slightly oblique rays avoid the existing raymarcher's reciprocal-of-zero AABB edge case.
            var endpoint = root.transform.TransformPoint(box.center + Vector3.forward * (box.size.z * 0.5f + 2)
                + new Vector3(0.05f, 0.025f, 0));
            var miss = endpoint + Vector3.up * 20;
            var shader = Instantiate(Resources.Load<ComputeShader>("Scripts/SDFRaymarch"));
            using var input = new ComputeBuffer(2, 16);
            using var output = new ComputeBuffer(2, 16);
            Vector3 direction = endpoint - origin;
            Vector3 missDirection = miss - origin;
            input.SetData(new[] { new Vector4(direction.x, direction.y, direction.z, 0),
                new Vector4(missDirection.x, missDirection.y, missDirection.z, 0) });
            int kernel = shader.FindKernel("SDFRaymarch");
            shader.SetBuffer(kernel, "_Points", input); shader.SetBuffer(kernel, "_ModifiedPoints", output);
            shader.SetTexture(kernel, "_SDF", sdf.sdf);
            shader.SetMatrix("_WorldToSDFSpace", sdf.worldToSDFTexCoords); shader.SetMatrix("_SDFToWorldSpace", sdf.sdflocalToWorld);
            shader.SetMatrix("_LocalToWorldRS", Matrix4x4.identity); shader.SetMatrix("_WorldToLocalRS", Matrix4x4.identity);
            shader.SetVector("_RayOriginWorld", origin); shader.SetFloat("_Margin", 0); shader.SetFloat("_MaxDistance", 100);
            shader.SetFloat("_HitThreshold", 0.05f); shader.SetInt("_MaxIterations", 256);
            shader.Dispatch(kernel, 1, 1, 1);
            var result = new Vector4[2]; output.GetData(result);
            Check(result[0].w == 1 && ((Vector3)result[0]).magnitude < direction.magnitude, $"GPU ray did not hit {root.name}: {result[0]}, direction={direction}, origin={origin}");
            Check(result[1].w == 0, "GPU miss changed " + root.name);
            Destroy(shader);
        }

        private int Count => Get<IDictionary>(renderer, "spawnedVehicles").Count;
        private TrafficMaterialCache Cache => Get<TrafficMaterialCache>(renderer, "materials");
        private int Detections => VirtualBoundingBoxDetector.BuildMessage(transform, transform, 10000, new builtin_interfaces.msg.Time()).Objects.Length;
        private GameObject Root(string id) => GameObject.Find("Vehicle_" + id);
        private static Material Paint(GameObject root)
        {
            var binding = root.GetComponent<TrafficVehicleAppearance>().PaintBindings[0];
            return binding.renderer.sharedMaterials[binding.materialIndex];
        }
        private static Material[] NonPaint(GameObject root)
        {
            var bindings = root.GetComponent<TrafficVehicleAppearance>().PaintBindings;
            return root.GetComponentsInChildren<Renderer>().SelectMany(r => r.sharedMaterials.Where((m, i) =>
                !bindings.Any(b => b.renderer == r && b.materialIndex == i))).ToArray();
        }
        private static TrafficReceiver.VehicleData Vehicle(string id, string blueprint, string color) =>
            new() { id = id, blueprint = blueprint, color = color, location = new TrafficReceiver.LocationData() };
        private static void Set(object target, string field, object value) => target.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic).SetValue(target, value);
        private static T Get<T>(object target, string field) => (T)target.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic).GetValue(target);
        private static void Check(bool value, string message) { if (!value) throw new Exception(message); }
        private static void Near(Vector3 actual, Vector3 expected, string message) => Check(Vector3.Distance(actual, expected) < 0.001f, message);
    }
}
