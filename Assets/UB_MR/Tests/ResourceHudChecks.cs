#if UNITY_EDITOR
using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using CAVAS.UB_MR.Telemetry;
using CAVAS.UB_MR.UI;
using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Tests
{
    public static class ResourceHudChecks
    {
        sealed class FixedGpu : IGpuProvider
        {
            readonly bool multiple;
            public FixedGpu(bool multiple = true) => this.multiple = multiple;
            public Task<GpuSnapshot> SampleAsync(CancellationToken cancellation)
            {
                var devices = new[]
                {
                    new GpuDeviceSnapshot("GPU-one", "NVIDIA GeForce RTX 5080", 16384, 6144, 2048),
                    new GpuDeviceSnapshot("GPU-two", "Second GPU", 16384, 2048, 0)
                };
                return Task.FromResult(new GpuSnapshot(multiple ? devices : new[] { devices[0] }, MonotonicClock.Instance.Seconds));
            }
        }

        // Unity -batchmode -nographics -executeMethod CAVAS.UB_MR.Tests.ResourceHudChecks.Run
        public static void Run()
        {
            try
            {
                ResourceTelemetryChecks.Run().GetAwaiter().GetResult();
                ResourceNetworkChecks.Run().GetAwaiter().GetResult();
                ResourceLidarChecks.Run();
                CheckHud(false);
                Debug.Log("UB-MR resource checks PASSED: telemetry, network, LiDAR queues, HUD and lifecycle.");
                EditorApplication.Exit(0);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        // Requires a graphics device. Writes two rendered fixture images into /tmp.
        public static void RunVisual()
        {
            try
            {
                CheckHud(true, true);
                CheckHud(true);
                Debug.Log("UB-MR resource HUD visual checks PASSED: 1280x720 and 1920x1080.");
                EditorApplication.Exit(0);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }

        static void CheckHud(bool render, bool overview = false)
        {
            var root = new GameObject("Resource HUD checks");
            var clock = new ResourceTelemetryChecks.Clock { Now = MonotonicClock.Instance.Seconds };
            using var collector = new ResourceTelemetry(clock);
            StatPanel panel = null;
            Task completion = Task.CompletedTask;
            try
            {
                for (int i = 0; i < (overview ? 1 : 6); i++)
                {
                    var sensor = collector.RegisterLidar(overview ? "Lincoln MKZ · /lidar/front" : "Ego · /lidar/" + i, i == 5);
                    double now = collector.Clock.Seconds;
                    if (i != 5) sensor.RecordProcessing(now - .012);
                    sensor.RecordPublished(now - .028, i == 5);
                }
                collector.SensorPayload.AddReceived(14 * 1048576);
                collector.SensorPayload.AddSent(13 * 1048576);
                clock.Now += 1;
                panel = StatPanel.Create(root.transform, collector, new FixedGpu(!overview)); completion = panel.Completion;
                Thread.Sleep(150);
                panel.Toggle(); panel.Toggle(); Canvas.ForceUpdateCanvases();
                ResourceTelemetryChecks.Check(panel.IsExpanded, "HUD must start expanded");
                var scroll = panel.GetComponentInChildren<ScrollRect>();
                ResourceTelemetryChecks.Check(scroll != null && !scroll.horizontal && scroll.vertical && scroll.verticalScrollbar != null,
                    "HUD needs a vertical scroll view and scrollbar");
                if (scroll.content.rect.height > scroll.viewport.rect.height + .5f)
                    ResourceTelemetryChecks.Check(scroll.verticalNormalizedPosition > .99f, "HUD must open at the top");
                var canvases = root.GetComponentsInChildren<Canvas>();
                ResourceTelemetryChecks.Check(canvases.Length == 1, "Duplicate HUD canvas");
                if (render)
                {
                    Capture(panel, 1280, 720, !overview);
                    Capture(panel, 1920, 1080, !overview);
                }
                int rowsBefore = collector.SensorSnapshots().Count;
                panel.Toggle(); ResourceTelemetryChecks.Check(!panel.IsExpanded, "HUD did not collapse");
                ResourceTelemetryChecks.Check(panel.transform.Find("Panel").GetComponent<RectTransform>().rect.width < 200,
                    "Collapsed Resources button is not compact");
                collector.RegisterLidar("New sensor while collapsed", true);
                panel.Toggle(); Canvas.ForceUpdateCanvases();
                ResourceTelemetryChecks.Check(panel.IsExpanded && collector.SensorSnapshots().Count == rowsBefore + 1,
                    "Collapsed HUD stopped collecting sensors");
                bool found = false;
                foreach (var label in panel.GetComponentsInChildren<TextMeshProUGUI>())
                    if (label.text.Contains("New sensor while collapsed")) found = true;
                ResourceTelemetryChecks.Check(found, "Reopened HUD did not show new sensor");
                panel.Shutdown(); panel.Shutdown();
            }
            finally { UnityEngine.Object.DestroyImmediate(root); }
            ResourceTelemetryChecks.Check(completion.Wait(3000), "Destroying HUD did not stop GPU worker");
        }

        static void Capture(StatPanel panel, int width, int height, bool crowded)
        {
            var cameraObject = new GameObject("HUD capture camera");
            var camera = cameraObject.AddComponent<Camera>();
            var target = new RenderTexture(width, height, 24);
            var previousActive = RenderTexture.active;
            Texture2D image = null;
            var canvas = panel.GetComponent<Canvas>();
            try
            {
                target.Create(); camera.targetTexture = target;
                camera.clearFlags = CameraClearFlags.SolidColor; camera.backgroundColor = new Color(.22f, .26f, .30f);
                camera.nearClipPlane = .01f; camera.farClipPlane = 100;
                canvas.renderMode = RenderMode.ScreenSpaceCamera; canvas.worldCamera = camera; canvas.planeDistance = 1;
                panel.GetComponent<CanvasScaler>().SendMessage("Update");
                panel.Toggle(); panel.Toggle(); Canvas.ForceUpdateCanvases();
                if (crowded)
                    ResourceTelemetryChecks.Check(panel.GetComponentInChildren<ScrollRect>().content.rect.height >
                        panel.GetComponentInChildren<ScrollRect>().viewport.rect.height, "Many sensors should scroll");
                foreach (var label in panel.GetComponentsInChildren<TextMeshProUGUI>())
                {
                    label.ForceMeshUpdate();
                    ResourceTelemetryChecks.Check(!label.isTextOverflowing, "HUD text clipped: " + label.text);
                }
                camera.Render(); RenderTexture.active = target;
                image = new Texture2D(width, height, TextureFormat.RGB24, false);
                image.ReadPixels(new Rect(0, 0, width, height), 0, 0); image.Apply();
                File.WriteAllBytes($"/tmp/ubmr-resource-hud-{width}x{height}{(crowded ? "-many-sensors" : "")}.png", image.EncodeToPNG());
            }
            finally
            {
                canvas.renderMode = RenderMode.ScreenSpaceOverlay; canvas.worldCamera = null;
                RenderTexture.active = previousActive; camera.targetTexture = null;
                UnityEngine.Object.DestroyImmediate(image); UnityEngine.Object.DestroyImmediate(target);
                UnityEngine.Object.DestroyImmediate(cameraObject);
            }
        }
    }
}
#endif
