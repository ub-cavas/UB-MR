using System;
using System.Collections.Generic;
using System.Globalization;
using CAVAS.UB_MR.Telemetry;
using TMPro;
using UB_MR.Redis_Networking;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI
{
    // A simulation overlay, independent of UI_Manager's mutually exclusive menu panels.
    public sealed class StatPanel : MonoBehaviour
    {
        const float Width = 384, CardWidth = 348, HeaderHeight = 64;
        static readonly Color Ink = new(.91f, .95f, .98f);
        static readonly Color Muted = new(.57f, .65f, .73f);
        static readonly Color OwnColor = new(.30f, .82f, .94f);
        static readonly Color OtherColor = new(.94f, .70f, .37f);
        static readonly Color LidarColor = new(.72f, .65f, .98f);
        static readonly Color NetworkColor = new(.42f, .84f, .69f);
        static readonly Color CardColor = new(.065f, .085f, .115f, .98f);
        static readonly Color LineColor = new(.15f, .19f, .24f);
        readonly List<TextMeshProUGUI> labels = new();
        readonly List<Image> shapes = new();
        ResourceTelemetry telemetry;
        GpuMonitor gpu;
        RectTransform panel, content, header, decorations, headerAccent;
        GameObject body;
        TextMeshProUGUI title, subtitle, toggleLabel;
        Canvas canvas;
        ScrollRect scroll;
        float savedScroll = 1;
        double nextRefresh;
        bool stopped;
        int labelCount, shapeCount;
        public bool IsExpanded { get; private set; } = true;
        public System.Threading.Tasks.Task Completion => gpu?.Completion ?? System.Threading.Tasks.Task.CompletedTask;

        public static StatPanel Create(Transform parent, ResourceTelemetry telemetry, IGpuProvider provider = null)
        {
            var root = new GameObject("Resource HUD", typeof(RectTransform), typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            root.transform.SetParent(parent, false);
            var view = root.AddComponent<StatPanel>();
            view.Initialize(telemetry, provider);
            return view;
        }

        void Initialize(ResourceTelemetry metrics, IGpuProvider provider)
        {
            telemetry = metrics;
            bool linux = Application.platform == RuntimePlatform.LinuxPlayer || Application.platform == RuntimePlatform.LinuxEditor;
            gpu = new GpuMonitor(provider ?? new NvidiaSmiProvider(linux, telemetry.Clock), telemetry.Clock);
            canvas = GetComponent<Canvas>(); canvas.renderMode = RenderMode.ScreenSpaceOverlay; canvas.sortingOrder = 100;
            var scaler = GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1280, 720); scaler.matchWidthOrHeight = 1;
            if (FindFirstObjectByType<EventSystem>() == null)
            {
                var events = new GameObject("Resource HUD Events", typeof(EventSystem), typeof(InputSystemUIInputModule));
                events.transform.SetParent(transform, false);
            }

            panel = Rect("Panel", transform);
            panel.anchorMin = panel.anchorMax = panel.pivot = Vector2.one;
            panel.anchoredPosition = new Vector2(-18, -18);
            var background = panel.gameObject.AddComponent<Image>(); background.color = new Color(.035f, .047f, .065f, .97f);
            var shadow = panel.gameObject.AddComponent<Shadow>();
            shadow.effectColor = new Color(0, 0, 0, .30f); shadow.effectDistance = new Vector2(3, -4);
            var border = panel.gameObject.AddComponent<Outline>();
            border.effectColor = new Color(.55f, .68f, .8f, .12f); border.effectDistance = Vector2.one;

            header = Rect("Resources", panel); Place(header, 0, 0, Width, HeaderHeight);
            var buttonImage = header.gameObject.AddComponent<Image>(); buttonImage.color = new Color(.06f, .082f, .11f);
            var button = header.gameObject.AddComponent<Button>(); button.targetGraphic = buttonImage;
            var colors = button.colors;
            colors.highlightedColor = new Color(1.15f, 1.15f, 1.15f); colors.pressedColor = new Color(.85f, .85f, .85f);
            button.colors = colors; button.onClick.AddListener(Toggle);
            headerAccent = Rect("Accent", header); Place(headerAccent, 0, 0, Width, 2);
            var accent = headerAccent.gameObject.AddComponent<Image>(); accent.color = OwnColor; accent.raycastTarget = false;
            subtitle = Label(header); subtitle.text = "RUNTIME  /  TELEMETRY"; subtitle.fontSize = 9;
            subtitle.characterSpacing = 1.8f; subtitle.color = Muted; Place(subtitle.rectTransform, 16, 12, 270, 14);
            title = Label(header); title.text = "Resources"; title.fontSize = 22; title.fontStyle = FontStyles.Bold;
            Place(title.rectTransform, 16, 28, 275, 28);
            toggleLabel = Label(header); toggleLabel.text = "−"; toggleLabel.fontSize = 22;
            toggleLabel.color = Muted; toggleLabel.alignment = TextAlignmentOptions.Center;
            Place(toggleLabel.rectTransform, Width - 42, 21, 28, 30);

            var bodyRect = Rect("Scroll view", panel); body = bodyRect.gameObject;
            bodyRect.anchorMin = Vector2.zero; bodyRect.anchorMax = Vector2.one;
            bodyRect.offsetMin = new Vector2(14, 12); bodyRect.offsetMax = new Vector2(-14, -74);
            scroll = body.AddComponent<ScrollRect>(); scroll.horizontal = false;
            scroll.movementType = ScrollRect.MovementType.Clamped; scroll.scrollSensitivity = 28;
            var viewport = Rect("Viewport", bodyRect); Stretch(viewport, 0, 0, -8, 0);
            viewport.gameObject.AddComponent<RectMask2D>();
            var hitArea = viewport.gameObject.AddComponent<Image>(); hitArea.color = Color.clear;
            content = Rect("Content", viewport);
            content.anchorMin = new Vector2(0, 1); content.anchorMax = Vector2.one; content.pivot = new Vector2(0, 1);
            content.anchoredPosition = Vector2.zero;
            decorations = Rect("Card backgrounds", content); Stretch(decorations, 0, 0, 0, 0);
            scroll.viewport = viewport; scroll.content = content;

            var track = Rect("Scrollbar", bodyRect);
            track.anchorMin = new Vector2(1, 0); track.anchorMax = Vector2.one;
            track.offsetMin = new Vector2(-3, 0); track.offsetMax = Vector2.zero;
            var trackImage = track.gameObject.AddComponent<Image>(); trackImage.color = new Color(.10f, .13f, .17f);
            var handle = Rect("Handle", track); Stretch(handle, 0, 0, 0, 0);
            var handleImage = handle.gameObject.AddComponent<Image>(); handleImage.color = new Color(.29f, .38f, .47f);
            var scrollbar = track.gameObject.AddComponent<Scrollbar>();
            scrollbar.handleRect = handle; scrollbar.targetGraphic = handleImage;
            scrollbar.direction = Scrollbar.Direction.BottomToTop; scrollbar.value = 1;
            scroll.verticalScrollbar = scrollbar;
            Refresh(); Canvas.ForceUpdateCanvases(); scroll.verticalNormalizedPosition = 1;
        }

        static RectTransform Rect(string name, Transform parent)
        {
            var rect = (RectTransform)new GameObject(name, typeof(RectTransform)).transform;
            rect.SetParent(parent, false); return rect;
        }
        static void Stretch(RectTransform rect, float left, float bottom, float right, float top)
        {
            rect.anchorMin = Vector2.zero; rect.anchorMax = Vector2.one;
            rect.offsetMin = new Vector2(left, bottom); rect.offsetMax = new Vector2(right, top);
        }
        static void Place(RectTransform rect, float x, float y, float width, float height)
        {
            rect.anchorMin = rect.anchorMax = rect.pivot = new Vector2(0, 1);
            rect.anchoredPosition = new Vector2(x, -y); rect.sizeDelta = new Vector2(width, height);
        }
        static TextMeshProUGUI Label(Transform parent)
        {
            var label = Rect("Label", parent).gameObject.AddComponent<TextMeshProUGUI>();
            label.font = TMP_Settings.defaultFontAsset; label.color = Ink; label.raycastTarget = false;
            label.richText = false; label.textWrappingMode = TextWrappingModes.Normal;
            return label;
        }
        void Box(float x, float y, float width, float height, Color color)
        {
            if (shapes.Count <= shapeCount)
            {
                var shape = Rect("Surface", decorations).gameObject.AddComponent<Image>();
                shape.raycastTarget = false; shapes.Add(shape);
            }
            var image = shapes[shapeCount++]; image.gameObject.SetActive(true); image.color = color;
            Place(image.rectTransform, x, y, width, height);
        }
        float Text(string text, float x, float y, float width, float size, Color color,
            bool bold = false, bool right = false, float height = 0)
        {
            if (labels.Count <= labelCount) labels.Add(Label(content));
            var label = labels[labelCount++]; label.gameObject.SetActive(true); label.text = text;
            label.fontSize = size; label.color = color; label.fontStyle = bold ? FontStyles.Bold : FontStyles.Normal;
            label.alignment = right ? TextAlignmentOptions.TopRight : TextAlignmentOptions.TopLeft;
            label.enableAutoSizing = height > 0;
            label.fontSizeMin = Math.Min(size, 10); label.fontSizeMax = size;
            float measured = height > 0 ? height : label.GetPreferredValues(text, width, 0).y + 2;
            Place(label.rectTransform, x, y, width, measured);
            return measured;
        }
        float Section(string name, string note, float y, Color accent)
        {
            Box(0, y + 4, 3, 11, accent);
            Text(name, 11, y, 215, 11, accent, true);
            Text(note, 224, y, 124, 10, Muted, right: true);
            return y + 25;
        }
        static string Number(double? number, string format = "0.00")
            => number?.ToString(format, CultureInfo.InvariantCulture) ?? "—";
        void TimingRow(string name, TimingSnapshot timing, float y, bool bypass = false)
        {
            Text(name, 14, y, 168, 12, Muted);
            Text(bypass ? "Bypass" : Number(timing.LatestMs), 184, y, 62, 14, bypass ? LidarColor : Ink, right: true, height: 21);
            Text(bypass ? "—" : Number(timing.AverageMs), 260, y, 74, 14, Muted, right: true, height: 21);
        }
        void RateRow(string name, string unit, TrafficSnapshot value, double divisor, float y)
        {
            Text(name, 14, y, 170, 12, Ink);
            Text(unit, 14, y + 17, 170, 10, Muted);
            Text(value.Available ? Number(value.ReceivedBytesPerSecond / divisor) : "—", 184, y + 3, 62, 16,
                OwnColor, right: true, height: 22);
            Text(value.Available ? Number(value.SentBytesPerSecond / divisor) : "—", 260, y + 3, 74, 16,
                NetworkColor, right: true, height: 22);
        }

        public void Toggle()
        {
            if (IsExpanded)
            {
                savedScroll = scroll.verticalNormalizedPosition; IsExpanded = false; body.SetActive(false);
            }
            else IsExpanded = true;
            subtitle.gameObject.SetActive(IsExpanded);
            title.fontSize = IsExpanded ? 22 : 15;
            Place(title.rectTransform, 16, IsExpanded ? 28 : 10, IsExpanded ? 275 : 110, IsExpanded ? 28 : 24);
            toggleLabel.text = IsExpanded ? "−" : "+";
            float width = IsExpanded ? Width : 168;
            header.sizeDelta = new Vector2(width, IsExpanded ? HeaderHeight : 42);
            headerAccent.sizeDelta = new Vector2(width, 2);
            Place(toggleLabel.rectTransform, width - 42, IsExpanded ? 21 : 5, 28, 30);
            Refresh();
            if (IsExpanded)
            {
                body.SetActive(true); Canvas.ForceUpdateCanvases(); scroll.verticalNormalizedPosition = savedScroll;
            }
        }
        void Update()
        {
            if (stopped || telemetry == null || telemetry.Clock.Seconds < nextRefresh) return;
            Refresh();
        }
        void Refresh()
        {
            nextRefresh = telemetry.Clock.Seconds + .25;
            TrafficSnapshot payload = telemetry.SensorPayload.Snapshot();
            RedisSnapshot redis = ServerConnection.Instance != null ? ServerConnection.Instance.TelemetrySnapshot : default;
            if (!IsExpanded) { panel.sizeDelta = new Vector2(168, 42); return; }
            labelCount = shapeCount = 0;
            float y = Section("GPU MEMORY", "MiB", 2, OwnColor);
            string processName = Application.isEditor ? "UNITY EDITOR" : "UB-MR";
            var graphics = gpu.Snapshot;
            if (!graphics.Available || telemetry.Clock.Seconds - graphics.SampleTime > 3)
            {
                string reason = graphics.UnavailableReason ?? "GPU sample is stale";
                float reasonHeight = Text(reason, 14, y + 35, 320, 11, Muted);
                Box(0, y, CardWidth, reasonHeight + 49, CardColor);
                Text("Unavailable", 14, y + 12, 320, 16, OtherColor);
                y += reasonHeight + 61;
            }
            else for (int i = 0; i < graphics.Devices.Count; i++)
            {
                var device = graphics.Devices[i];
                float nameHeight = Text($"GPU {i} / {device.Name}", 14, y + 11, 320, 12, Ink, true);
                float top = y + nameHeight + 18;
                Box(0, y, CardWidth, nameHeight + 109, CardColor);
                Text(Number(device.ProcessMiB, "#,##0"), 14, top, 150, 27, OwnColor, right: false, height: 34);
                Text(Number(device.OtherMiB, "#,##0"), 183, top + 3, 151, 23, OtherColor, right: false, height: 30);
                Text(device.ProcessMiB.HasValue ? processName : "APPLICATION UNAVAILABLE", 14, top + 35, 155, 9, Muted);
                Text("OTHER / SYSTEM", 183, top + 35, 151, 9, Muted);
                float barY = top + 56;
                Box(14, barY, 320, 6, LineColor);
                if (device.TotalMiB > 0 && device.ProcessMiB.HasValue && device.OtherMiB.HasValue)
                {
                    float own = Mathf.Clamp01((float)(device.ProcessMiB.Value / device.TotalMiB.Value));
                    float other = Mathf.Clamp((float)(device.OtherMiB.Value / device.TotalMiB.Value), 0, 1 - own);
                    if (own > 0) Box(14, barY, 320 * own, 6, OwnColor);
                    if (other > 0) Box(14 + 320 * own, barY, 320 * other, 6, OtherColor);
                }
                Text($"{Number(device.UsedMiB, "#,##0")} / {Number(device.TotalMiB, "#,##0")} used · incl. reserved", 14, barY + 13, 275, 10, Muted);
                string percentage = device.TotalMiB > 0 && device.UsedMiB.HasValue
                    ? (100 * device.UsedMiB.Value / device.TotalMiB.Value).ToString("0", CultureInfo.InvariantCulture) + "%" : "—";
                Text(percentage, 284, barY + 11, 50, 12, Ink, true, true, 19);
                y += nameHeight + 121;
            }

            y = Section("LiDAR LATENCY", "milliseconds", y + 3, LidarColor);
            var sensors = telemetry.SensorSnapshots();
            if (sensors.Count == 0)
            {
                Box(0, y, CardWidth, 44, CardColor);
                Text("No LiDAR sensors", 14, y + 13, 320, 12, Muted); y += 56;
            }
            foreach (var sensor in sensors)
            {
                float nameHeight = Text(sensor.Name, 14, y + 11, 320, 12, Ink, true);
                float top = y + nameHeight + 18;
                Box(0, y, CardWidth, nameHeight + 91, CardColor);
                string state = !sensor.ReceiveToPublish.Available ? "WAITING" : sensor.ReceiveToPublish.Inactive ? "INACTIVE" : "";
                if (state.Length > 0) Text(state, 14, top, 168, 9, OtherColor);
                Text("LATEST", 184, top, 62, 9, Muted, right: true);
                Text("AVG · 10 S", 260, top, 74, 9, Muted, right: true);
                TimingRow("Processing", sensor.Processing, top + 19, sensor.Bypass);
                TimingRow("Receive → publish", sensor.ReceiveToPublish, top + 44);
                y += nameHeight + 103;
            }

            y = Section("NETWORK", "1 s rates", y + 3, NetworkColor);
            Box(0, y, CardWidth, 171, CardColor);
            Text("THROUGHPUT", 14, y + 12, 166, 9, Muted);
            Text("RX", 184, y + 12, 62, 9, OwnColor, right: true);
            Text("TX", 260, y + 12, 74, 9, NetworkColor, right: true);
            RateRow("ROS sensor payload", "MiB/s", payload, 1048576, y + 32);
            RateRow("Redis", "KiB/s", redis.Traffic, 1024, y + 72);
            Box(14, y + 113, 320, 1, LineColor);
            Text("SERVER RTT", 14, y + 125, 166, 9, Muted);
            Text("latest / avg · 10 s", 183, y + 125, 151, 9, Muted, right: true);
            Text(redis.Connected ? !redis.RoundTrip.Available ? "Waiting for probe" :
                redis.RoundTrip.Inactive ? "Inactive · ms" : "Redis · ms" : "Disconnected",
                14, y + 143, 160, 11, redis.Connected ? Muted : OtherColor);
            Text(redis.Connected ? Number(redis.RoundTrip.LatestMs) + " / " + Number(redis.RoundTrip.AverageMs) : "— / —",
                174, y + 142, 160, 15, Ink, right: true, height: 22);
            y += 181;
            y += Text("— Unavailable or waiting for samples.\nSensor payload excludes network overhead. LiDAR ends at local publish; server RTT uses Redis PING/PONG.",
                4, y, 338, 10, Muted) + 4;
            for (int i = labelCount; i < labels.Count; i++) labels[i].gameObject.SetActive(false);
            for (int i = shapeCount; i < shapes.Count; i++) shapes[i].gameObject.SetActive(false);
            content.sizeDelta = new Vector2(0, y);
            float availableHeight = canvas.pixelRect.height / Mathf.Max(.01f, canvas.scaleFactor) - 36;
            panel.sizeDelta = new Vector2(Width, Mathf.Min(Mathf.Max(100, availableHeight), Mathf.Min(660, y + 86)));
        }
        public void Shutdown()
        {
            if (stopped) return; stopped = true; gpu?.Dispose();
        }
        void OnDestroy() => Shutdown();
    }
}
