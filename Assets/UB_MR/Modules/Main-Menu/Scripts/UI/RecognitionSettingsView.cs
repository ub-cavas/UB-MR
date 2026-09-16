using System.Globalization;
using CAVAS.UB_MR.Config;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Modules.MainMenu
{
    /// <summary>Uses the existing menu's control templates so shipped scenes need no manual wiring.</summary>
    public sealed class RecognitionSettingsView
    {
        readonly GameObject root;
        readonly TMP_Dropdown mode;
        readonly TMP_Dropdown clock;
        readonly TMP_InputField topic;
        readonly TMP_InputField rate;
        readonly TMP_InputField radius;
        readonly TextMeshProUGUI error;
        Config.Agent agent;

        public RecognitionSettingsView(Transform parent, SensorEditPanel templates, Button buttonTemplate)
        {
            root = new GameObject("Virtual object recognition", typeof(RectTransform), typeof(Image));
            root.transform.SetParent(parent, false);
            var panel = (RectTransform)root.transform;
            panel.anchorMin = Vector2.zero;
            panel.anchorMax = Vector2.one;
            panel.offsetMin = Vector2.zero;
            panel.offsetMax = Vector2.zero;
            root.GetComponent<Image>().color = new Color(0.08f, 0.10f, 0.13f, 1f);
            var font = buttonTemplate.GetComponentInChildren<TextMeshProUGUI>(true).font;
            Label("Virtual object recognition", font, 40, 30, 820, 65, 32);
            Label("Saved with this agent. Changes apply to the next session.", font, 40, 105, 820, 55, 23);
            Label("Recognition mode", font, 40, 205, 270, 56, 24);
            mode = Dropdown(templates.RecognitionDropdownTemplate, 205,
                new[] { "LiDAR modification", "Direct bounding boxes" });
            Label("ROS clock", font, 40, 285, 270, 56, 24);
            clock = Dropdown(templates.RecognitionDropdownTemplate, 285,
                new[] { "System time", "Simulation /clock" });
            Label("Bounding-box topic", font, 40, 365, 270, 56, 24);
            topic = Input(templates.RecognitionInputTemplate, 365);
            Label("Publication rate (Hz)", font, 40, 445, 270, 56, 24);
            rate = Input(templates.RecognitionInputTemplate, 445);
            Label("Detection radius (m)", font, 40, 525, 270, 56, 24);
            radius = Input(templates.RecognitionInputTemplate, 525);
            Label("Direct boxes include all virtual objects within range, including occluded objects. " +
                "Autoware still tracks and predicts their motion.", font, 40, 615, 820, 85, 23);
            error = Label("", font, 40, 705, 820, 55, 22);
            error.color = new Color(1f, 0.65f, 0.5f);
            var cancel = MakeButton(buttonTemplate, "Cancel", 40);
            cancel.onClick.AddListener(Hide);
            var save = MakeButton(buttonTemplate, "Save settings", 465);
            save.onClick.AddListener(Save);
            mode.onValueChanged.AddListener(_ => RefreshMode());
            Hide();
        }

        static void Place(RectTransform rect, float x, float y, float width, float height)
        {
            rect.anchorMin = rect.anchorMax = new Vector2(0, 1);
            rect.pivot = new Vector2(0, 1);
            rect.anchoredPosition = new Vector2(x, -y);
            rect.sizeDelta = new Vector2(width, height);
            rect.localScale = Vector3.one;
        }

        TextMeshProUGUI Label(string text, TMP_FontAsset font, float x, float y, float width, float height, float size)
        {
            var obj = new GameObject(text, typeof(RectTransform), typeof(TextMeshProUGUI));
            obj.transform.SetParent(root.transform, false);
            var label = obj.GetComponent<TextMeshProUGUI>();
            label.font = font;
            label.text = text;
            label.fontSize = size;
            label.color = Color.white;
            label.raycastTarget = false;
            Place(label.rectTransform, x, y, width, height);
            return label;
        }

        TMP_Dropdown Dropdown(TMP_Dropdown template, float y, string[] options)
        {
            var control = Object.Instantiate(template, root.transform);
            control.onValueChanged = new TMP_Dropdown.DropdownEvent();
            control.ClearOptions();
            control.AddOptions(new System.Collections.Generic.List<string>(options));
            Place((RectTransform)control.transform, 330, y, 530, 56);
            control.captionText.enableAutoSizing = true;
            control.captionText.fontSizeMin = 18;
            control.captionText.fontSizeMax = 25;
            control.gameObject.SetActive(true);
            return control;
        }

        TMP_InputField Input(TMP_InputField template, float y)
        {
            var control = Object.Instantiate(template, root.transform);
            control.onValueChanged = new TMP_InputField.OnChangeEvent();
            control.onEndEdit = new TMP_InputField.SubmitEvent();
            control.contentType = TMP_InputField.ContentType.Standard;
            control.text = "";
            Place((RectTransform)control.transform, 330, y, 530, 56);
            control.textComponent.fontSize = 25;
            control.gameObject.SetActive(true);
            return control;
        }

        Button MakeButton(Button template, string text, float x)
        {
            var button = Object.Instantiate(template, root.transform);
            button.onClick = new Button.ButtonClickedEvent();
            button.GetComponentInChildren<TextMeshProUGUI>(true).text = text;
            Place((RectTransform)button.transform, x, 790, 395, 70);
            return button;
        }

        public void Show(Config.Agent selectedAgent)
        {
            agent = selectedAgent;
            var settings = agent.recognition ?? new VirtualObjectRecognitionSettings();
            mode.SetValueWithoutNotify((int)settings.mode);
            clock.SetValueWithoutNotify(settings.useSimTime ? 1 : 0);
            topic.text = settings.boundingBoxTopic;
            rate.text = settings.publishRateHz.ToString(CultureInfo.InvariantCulture);
            radius.text = settings.detectionRadiusMeters.ToString(CultureInfo.InvariantCulture);
            error.text = "";
            RefreshMode();
            root.SetActive(true);
            root.transform.SetAsLastSibling();
        }

        void RefreshMode()
        {
            bool boxes = mode.value == (int)VirtualObjectRecognitionMode.BoundingBoxInjection;
            topic.interactable = rate.interactable = radius.interactable = clock.interactable = boxes;
        }

        void Save()
        {
            if (!float.TryParse(rate.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float hz) ||
                !float.TryParse(radius.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float meters))
            {
                error.text = "Enter numeric rate and radius values.";
                return;
            }
            var settings = new VirtualObjectRecognitionSettings
            {
                mode = (VirtualObjectRecognitionMode)mode.value, useSimTime = clock.value == 1,
                boundingBoxTopic = topic.text.Trim(), publishRateHz = hz, detectionRadiusMeters = meters
            };
            if (!settings.TryValidate(out string message)) { error.text = message; return; }
            agent.recognition = settings;
            ConfigurationManager.SaveToJSON(agent);
            Hide();
        }

        public void Hide() => root.SetActive(false);
    }
}
