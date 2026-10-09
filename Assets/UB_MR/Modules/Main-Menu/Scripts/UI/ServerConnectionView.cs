using System.Globalization;
using TMPro;
using UB_MR.Redis_Networking;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Modules.MainMenu
{
    public sealed class ServerConnectionView
    {
        readonly GameObject root;
        readonly TMP_InputField host, port, channel, password;
        readonly TextMeshProUGUI status;
        readonly ServerConnection connection;
        string validationError;

        public ServerConnectionView(Transform parent, TMP_InputField inputTemplate, Button buttonTemplate,
            ServerConnection connection)
        {
            this.connection = connection;
            root = new GameObject("Server Connection", typeof(RectTransform), typeof(Image));
            root.transform.SetParent(parent, false);
            var rect = (RectTransform)root.transform;
            rect.anchorMin = Vector2.zero; rect.anchorMax = Vector2.one;
            rect.offsetMin = rect.offsetMax = Vector2.zero;
            root.GetComponent<Image>().color = new Color(.08f, .10f, .13f);
            var font = buttonTemplate.GetComponentInChildren<TextMeshProUGUI>(true).font;
            Label("Server connection", font, 40, 30, 820, 60, 36);
            Label("Receive traffic and share your ego vehicle's position.", font, 40, 105, 820, 50, 23);
            Label("Server address", font, 40, 180, 820, 40, 25);
            host = Input(inputTemplate, "Server address", 40, 225, 820);
            Label("Port", font, 40, 310, 200, 40, 25);
            port = Input(inputTemplate, "Port", 40, 355, 200);
            port.contentType = TMP_InputField.ContentType.IntegerNumber;
            Label("Channel", font, 280, 310, 580, 40, 25);
            channel = Input(inputTemplate, "Channel", 280, 355, 580);
            Label("Password", font, 40, 440, 820, 40, 25);
            password = Input(inputTemplate, "Password", 40, 485, 820);
            password.contentType = TMP_InputField.ContentType.Password;
            Label("Address and port are remembered. Password is kept for this run only.", font,
                40, 560, 820, 60, 22);
            status = Label("", font, 40, 635, 820, 60, 23);
            Button connect = Button(buttonTemplate, "Connect", 40, 720, 395);
            connect.onClick.AddListener(Connect);
            Button disconnect = Button(buttonTemplate, "Disconnect", 465, 720, 395);
            disconnect.onClick.AddListener(() => { validationError = null; connection.Disconnect(); Refresh(); });
            Button back = Button(buttonTemplate, "Back", 40, 805, 820);
            back.onClick.AddListener(Hide);
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
            var go = new GameObject("Label", typeof(RectTransform), typeof(TextMeshProUGUI));
            go.transform.SetParent(root.transform, false);
            var label = go.GetComponent<TextMeshProUGUI>();
            label.font = font; label.text = text; label.fontSize = size;
            label.color = Color.white; label.raycastTarget = false;
            Place(label.rectTransform, x, y, width, height);
            return label;
        }

        TMP_InputField Input(TMP_InputField template, string name, float x, float y, float width)
        {
            var field = Object.Instantiate(template, root.transform);
            field.name = name;
            field.onValueChanged = new TMP_InputField.OnChangeEvent();
            field.onEndEdit = new TMP_InputField.SubmitEvent();
            field.onSubmit = new TMP_InputField.SubmitEvent();
            field.contentType = TMP_InputField.ContentType.Standard;
            field.lineType = TMP_InputField.LineType.SingleLine;
            field.characterLimit = 256;
            field.textComponent.fontSize = 25;
            if (field.placeholder is TMP_Text placeholder) placeholder.text = "";
            Place((RectTransform)field.transform, x, y, width, 60);
            field.gameObject.SetActive(true);
            return field;
        }

        Button Button(Button template, string text, float x, float y, float width)
        {
            var button = Object.Instantiate(template, root.transform);
            button.name = text;
            button.onClick = new Button.ButtonClickedEvent();
            button.GetComponentInChildren<TextMeshProUGUI>(true).text = text;
            Place((RectTransform)button.transform, x, y, width, 60);
            return button;
        }

        public void Show()
        {
            var settings = connection.Settings;
            host.text = settings.host;
            port.text = settings.port.ToString(CultureInfo.InvariantCulture);
            channel.text = settings.channel;
            password.text = settings.password ?? "";
            validationError = null;
            root.SetActive(true);
            root.transform.SetAsLastSibling();
            Refresh();
        }

        void Connect()
        {
            if (!int.TryParse(port.text, out int number))
                validationError = "Enter a port number between 1 and 65535.";
            else
            {
                var settings = new ServerSettings
                {
                    host = host.text.Trim(), port = number, channel = channel.text.Trim(), password = password.text
                };
                connection.Connect(settings, out validationError);
            }
            Refresh();
        }

        public void Refresh()
        {
            if (!root.activeSelf) return;
            status.text = validationError ?? (connection.IsConnected
                ? connection.HasTraffic ? $"Connected · {connection.VehicleCount} traffic vehicles" : "Connected · waiting for traffic"
                : connection.Status);
            status.color = validationError != null ? new Color(1f, .65f, .5f) :
                connection.IsConnected ? new Color(.5f, 1f, .7f) : Color.white;
        }

        public void Hide() => root.SetActive(false);
    }
}
