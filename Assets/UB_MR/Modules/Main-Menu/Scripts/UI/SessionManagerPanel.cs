using CAVAS.UI;
using TMPro;
using UB_MR.Redis_Networking;
using UnityEngine;
using UnityEngine.UI;


namespace CAVAS.UB_MR.Modules.MainMenu
{
    public class SessionManagerPanel : Panel
    {
        [SerializeField] Button newSessionButton;
        [SerializeField] Button loadSessionButton;
        [SerializeField] Button exitButton;
        [Header("Panels")]
        [SerializeField] Panel agentSelectionPanel;
        [SerializeField] Panel confirmationPanel;

        Button serverButton;
        TextMeshProUGUI serverStatus;
        ServerConnectionView serverView;
        ServerConnection serverConnection;

        public override void LoadPanel()
        {
            newSessionButton.onClick.AddListener(NewSession);
            loadSessionButton.onClick.AddListener(LoadSession);
            base.LoadPanel();
            serverConnection = ServerConnection.GetOrCreate();
            if (serverButton == null)
            {
                var template = FindFirstObjectByType<SensorEditPanel>(FindObjectsInactive.Include);
                serverButton = Instantiate(newSessionButton, newSessionButton.transform.parent);
                serverButton.name = "Server connection";
                serverButton.onClick = new Button.ButtonClickedEvent();
                serverButton.GetComponentInChildren<TextMeshProUGUI>(true).text = "Server connection";
                ((RectTransform)newSessionButton.transform).anchoredPosition = new Vector2(0, 180);
                ((RectTransform)loadSessionButton.transform).anchoredPosition = new Vector2(0, 70);
                ((RectTransform)serverButton.transform).anchoredPosition = new Vector2(0, -40);
                ((RectTransform)exitButton.transform).anchoredPosition = new Vector2(0, -150);
                var statusObject = new GameObject("Server status", typeof(RectTransform), typeof(TextMeshProUGUI));
                statusObject.transform.SetParent(transform, false);
                serverStatus = statusObject.GetComponent<TextMeshProUGUI>();
                serverStatus.font = serverButton.GetComponentInChildren<TextMeshProUGUI>(true).font;
                serverStatus.fontSize = 23;
                serverStatus.alignment = TextAlignmentOptions.Center;
                serverStatus.raycastTarget = false;
                serverStatus.rectTransform.anchorMin = serverStatus.rectTransform.anchorMax = new Vector2(.5f, .5f);
                serverStatus.rectTransform.anchoredPosition = new Vector2(0, -290);
                serverStatus.rectTransform.sizeDelta = new Vector2(780, 120);
                serverView = new ServerConnectionView(transform, template.RecognitionInputTemplate, newSessionButton, serverConnection);
                serverButton.onClick.AddListener(serverView.Show);
            }
        }

        public override void UnloadPanel()
        {
            serverView?.Hide();
            newSessionButton.onClick.RemoveAllListeners();
            loadSessionButton.onClick.RemoveAllListeners();
            base.UnloadPanel();
        }

        void Update()
        {
            if (serverConnection == null || serverStatus == null) return;
            serverStatus.text = $"{serverConnection.Settings.host}:{serverConnection.Settings.port}\n{serverConnection.Status}";
            serverView.Refresh();
        }

        void NewSession()
        {
            UI_Manager.LoadPanel(agentSelectionPanel);
        }

        void LoadSession()
        {
            //TODO: Read a config file and set ConfigurationManager variables
            UI_Manager.LoadPanel(confirmationPanel);
        }
    }
}
