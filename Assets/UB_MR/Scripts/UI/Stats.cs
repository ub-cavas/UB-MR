using UnityEngine;
using TMPro;
using UnityEngine.UI;
using CAVAS.UI;
using System;
using System.Text;
using UB_MR.Redis_Networking;

namespace CAVAS.UB_MR.UI
{
     
    public class StatPanel : Panel
    {
        private const float RefreshIntervalSeconds = 0.25f;

        [SerializeField] private TextMeshProUGUI diagnosticsText;
        [SerializeField] private TrafficReceiver trafficReceiver;

        private readonly StringBuilder _builder = new StringBuilder(512);
        private float _smoothedFrameSeconds;
        private float _nextRefreshTime;
        private bool _layoutReady;
        private static Button _hudToggleButton;
        private static TextMeshProUGUI _hudToggleText;

        public static StatPanel FindOrCreate()
        {
            foreach (StatPanel existing in Resources.FindObjectsOfTypeAll<StatPanel>())
            {
                if (existing is not null && existing.gameObject.scene.IsValid())
                {
                    existing.EnsureLayout();
                    return existing;
                }
            }

            Canvas canvas = FindUsableCanvas();
            if (canvas is null)
            {
                GameObject canvasObject = new GameObject("UB-MR Diagnostics Canvas", typeof(RectTransform), typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
                canvas = canvasObject.GetComponent<Canvas>();
                canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            }

            GameObject panelObject = new GameObject("StatsPanel", typeof(RectTransform), typeof(CanvasRenderer), typeof(Image));
            panelObject.transform.SetParent(canvas.transform, false);

            StatPanel panel = panelObject.AddComponent<StatPanel>();
            panel.EnsureLayout();
            panel.gameObject.SetActive(false);
            return panel;
        }

        public static void EnsureHudToggle(Action onClick)
        {
            StatPanel panel = FindOrCreate();
            Canvas canvas = panel.GetComponentInParent<Canvas>();
            if (canvas is null)
                return;

            Button toggleButton = FindHudToggleButton();
            if (toggleButton is null)
                toggleButton = CreateHudToggleButton(canvas.transform);

            toggleButton.onClick.RemoveAllListeners();
            toggleButton.onClick.AddListener(() => onClick?.Invoke());
            _hudToggleButton = toggleButton;
            _hudToggleText = toggleButton.GetComponentInChildren<TextMeshProUGUI>(true);
            UpdateHudToggleState(panel.gameObject.activeInHierarchy);
        }

        public static void UpdateHudToggleState(bool panelVisible)
        {
            if (_hudToggleButton is null)
                _hudToggleButton = FindHudToggleButton();
            if (_hudToggleButton is null)
                return;

            if (_hudToggleText is null)
                _hudToggleText = _hudToggleButton.GetComponentInChildren<TextMeshProUGUI>(true);

            if (_hudToggleText is not null)
                _hudToggleText.text = panelVisible ? "Diagnostics On" : "Diagnostics";

            Image image = _hudToggleButton.GetComponent<Image>();
            if (image is not null)
                image.color = panelVisible
                    ? new Color(0.10f, 0.42f, 0.33f, 0.92f)
                    : new Color(0.02f, 0.03f, 0.04f, 0.82f);

            ColorBlock colors = _hudToggleButton.colors;
            colors.normalColor = panelVisible
                ? new Color(0.10f, 0.42f, 0.33f, 0.92f)
                : new Color(0.02f, 0.03f, 0.04f, 0.82f);
            colors.highlightedColor = panelVisible
                ? new Color(0.14f, 0.52f, 0.42f, 0.96f)
                : new Color(0.13f, 0.18f, 0.22f, 0.95f);
            colors.pressedColor = new Color(0.08f, 0.28f, 0.24f, 0.95f);
            colors.selectedColor = colors.highlightedColor;
            _hudToggleButton.colors = colors;
        }

        void Start()
        {
            EnsureLayout();
            UpdateStatsGUI(force: true);
        }

        void Update()
        {
            UpdateStatsGUI();
        }

        public override void LoadPanel()
        {
            EnsureLayout();
            base.LoadPanel();
            SetTitle("Diagnostics");
            UpdateStatsGUI(force: true);
        }

        public void AddStatistic()
        {
            //TODO: Add new stat UI setup
        }

        public void UpdateStatsGUI(bool force = false)
        {
            EnsureLayout();

            float unscaledDeltaTime = Time.unscaledDeltaTime;
            if (unscaledDeltaTime > 0f)
                _smoothedFrameSeconds = Mathf.Lerp(_smoothedFrameSeconds <= 0f ? unscaledDeltaTime : _smoothedFrameSeconds, unscaledDeltaTime, 0.1f);

            if (!force && Time.unscaledTime < _nextRefreshTime)
                return;

            _nextRefreshTime = Time.unscaledTime + RefreshIntervalSeconds;

            if (trafficReceiver is null)
                trafficReceiver = FindFirstObjectByType<TrafficReceiver>();

            float fps = _smoothedFrameSeconds > 0f ? 1f / _smoothedFrameSeconds : 0f;
            float frameMs = _smoothedFrameSeconds * 1000f;

            _builder.Clear();
            _builder.AppendLine($"FPS: {fps:0.0}  Frame: {frameMs:0.0} ms");

            if (trafficReceiver is null)
            {
                _builder.AppendLine("Traffic: receiver not found");
                diagnosticsText.text = _builder.ToString();
                return;
            }

            TrafficReceiver.TrafficDiagnosticsSnapshot traffic = trafficReceiver.GetDiagnosticsSnapshot();
            string age = traffic.LastPacketAgeSeconds >= 0f ? $"{traffic.LastPacketAgeSeconds * 1000f:0} ms" : "none";
            string frame = traffic.LastServerFrame.HasValue ? traffic.LastServerFrame.Value.ToString() : "n/a";
            string trafficState = !traffic.HasTraffic
                ? "waiting"
                : traffic.LastPacketAgeSeconds > 1f ? "stale" : "live";

            _builder.AppendLine($"Traffic: {trafficState}  Vehicles: {traffic.VehicleCount}");
            _builder.AppendLine($"UDP: {traffic.PacketsPerSecond:0.0} pkt/s  {FormatBytes(traffic.BytesPerSecond)}/s");
            _builder.AppendLine($"Last: {age}  Port: {traffic.ListenPort}");
            _builder.AppendLine($"Peer: {traffic.RemoteEndpoint}");
            _builder.AppendLine($"Frame: {frame}  Missed: {traffic.MissedFrames}  Dup: {traffic.DuplicateFrames}");
            _builder.AppendLine($"Jitter: {traffic.JitterSeconds * 1000f:0.0} ms  Bad packets: {traffic.MalformedPackets}");

            diagnosticsText.text = _builder.ToString();
        }

        //TODO: Implement generic update procedures

        private void EnsureLayout()
        {
            if (_layoutReady && diagnosticsText is not null)
                return;

            RectTransform rectTransform = GetComponent<RectTransform>();
            rectTransform.anchorMin = new Vector2(0f, 1f);
            rectTransform.anchorMax = new Vector2(0f, 1f);
            rectTransform.pivot = new Vector2(0f, 1f);
            rectTransform.anchoredPosition = new Vector2(12f, -12f);
            rectTransform.sizeDelta = new Vector2(440f, 300f);

            Image background = GetComponent<Image>();
            if (background is null)
                background = gameObject.AddComponent<Image>();
            background.color = new Color(0.02f, 0.03f, 0.04f, 0.78f);
            background.raycastTarget = false;

            Transform oldVelocityGroup = transform.Find("Velocity");
            if (oldVelocityGroup is not null)
                oldVelocityGroup.gameObject.SetActive(false);

            TextMeshProUGUI titleText = FindDirectText("Panel_Title (TMP)") ?? FindDirectText("Title (TMP)");
            if (titleText is null)
                titleText = CreateText("Panel_Title (TMP)", 24f);
            SetTextRect(titleText, new Vector2(16f, -12f), new Vector2(408f, 34f));
            titleText.text = "Diagnostics";
            titleText.fontSize = 24f;
            titleText.alignment = TextAlignmentOptions.Left;
            titleText.fontStyle = FontStyles.Bold;
            titleText.raycastTarget = false;

            diagnosticsText = diagnosticsText ?? FindDirectText("Diagnostics (TMP)");
            if (diagnosticsText is null)
                diagnosticsText = CreateText("Diagnostics (TMP)", 18f);
            SetTextRect(diagnosticsText, new Vector2(16f, -54f), new Vector2(408f, 230f));
            diagnosticsText.fontSize = 18f;
            diagnosticsText.alignment = TextAlignmentOptions.TopLeft;
            diagnosticsText.fontStyle = FontStyles.Normal;
            diagnosticsText.enableWordWrapping = false;
            diagnosticsText.raycastTarget = false;

            _layoutReady = true;
        }

        private TextMeshProUGUI FindDirectText(string childName)
        {
            Transform child = transform.Find(childName);
            return child is not null ? child.GetComponent<TextMeshProUGUI>() : null;
        }

        private static Canvas FindUsableCanvas()
        {
            foreach (Canvas candidate in Resources.FindObjectsOfTypeAll<Canvas>())
            {
                if (candidate is null || !candidate.gameObject.scene.IsValid() || !candidate.isActiveAndEnabled)
                    continue;
                if (candidate.transform.lossyScale.sqrMagnitude <= 0.0001f)
                    continue;

                return candidate;
            }

            return null;
        }

        private static Button FindHudToggleButton()
        {
            foreach (Button candidate in Resources.FindObjectsOfTypeAll<Button>())
            {
                if (candidate is null || !candidate.gameObject.scene.IsValid())
                    continue;
                if (candidate.name == "Diagnostics Toggle Button")
                    return candidate;
            }

            return null;
        }

        private static Button CreateHudToggleButton(Transform parent)
        {
            GameObject buttonObject = new GameObject("Diagnostics Toggle Button", typeof(RectTransform), typeof(CanvasRenderer), typeof(Image), typeof(Button));
            buttonObject.transform.SetParent(parent, false);

            RectTransform buttonRect = buttonObject.GetComponent<RectTransform>();
            buttonRect.anchorMin = new Vector2(1f, 1f);
            buttonRect.anchorMax = new Vector2(1f, 1f);
            buttonRect.pivot = new Vector2(1f, 1f);
            buttonRect.anchoredPosition = new Vector2(-12f, -12f);
            buttonRect.sizeDelta = new Vector2(164f, 36f);

            Image image = buttonObject.GetComponent<Image>();
            image.color = new Color(0.02f, 0.03f, 0.04f, 0.82f);

            Button button = buttonObject.GetComponent<Button>();
            ColorBlock colors = button.colors;
            colors.normalColor = new Color(0.02f, 0.03f, 0.04f, 0.82f);
            colors.highlightedColor = new Color(0.13f, 0.18f, 0.22f, 0.95f);
            colors.pressedColor = new Color(0.08f, 0.28f, 0.24f, 0.95f);
            colors.selectedColor = colors.highlightedColor;
            colors.colorMultiplier = 1f;
            button.colors = colors;

            GameObject textObject = new GameObject("Label (TMP)", typeof(RectTransform), typeof(CanvasRenderer), typeof(TextMeshProUGUI));
            textObject.transform.SetParent(buttonObject.transform, false);

            RectTransform textRect = textObject.GetComponent<RectTransform>();
            textRect.anchorMin = Vector2.zero;
            textRect.anchorMax = Vector2.one;
            textRect.offsetMin = Vector2.zero;
            textRect.offsetMax = Vector2.zero;

            TextMeshProUGUI text = textObject.GetComponent<TextMeshProUGUI>();
            text.text = "Diagnostics";
            text.fontSize = 16f;
            text.fontStyle = FontStyles.Bold;
            text.alignment = TextAlignmentOptions.Center;
            text.color = Color.white;
            text.raycastTarget = false;

            return button;
        }

        private TextMeshProUGUI CreateText(string objectName, float fontSize)
        {
            GameObject textObject = new GameObject(objectName, typeof(RectTransform), typeof(CanvasRenderer), typeof(TextMeshProUGUI));
            textObject.transform.SetParent(transform, false);

            TextMeshProUGUI text = textObject.GetComponent<TextMeshProUGUI>();
            text.fontSize = fontSize;
            text.color = Color.white;
            text.text = string.Empty;
            return text;
        }

        private void SetTextRect(TextMeshProUGUI text, Vector2 anchoredPosition, Vector2 sizeDelta)
        {
            RectTransform textRect = text.GetComponent<RectTransform>();
            textRect.anchorMin = new Vector2(0f, 1f);
            textRect.anchorMax = new Vector2(0f, 1f);
            textRect.pivot = new Vector2(0f, 1f);
            textRect.anchoredPosition = anchoredPosition;
            textRect.sizeDelta = sizeDelta;
        }

        private string FormatBytes(float bytesPerSecond)
        {
            if (bytesPerSecond >= 1024f * 1024f)
                return $"{bytesPerSecond / (1024f * 1024f):0.00} MB";
            if (bytesPerSecond >= 1024f)
                return $"{bytesPerSecond / 1024f:0.0} KB";
            return $"{bytesPerSecond:0} B";
        }
    }
}
