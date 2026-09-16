using CAVAS.UB_MR.Config;
using CAVAS.UI;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Modules.MainMenu
{
    public class AgentEditMenu : ScrollPanel
    {
        [SerializeField] Button loadAgentButton;

        [Header("Panels")]
        [SerializeField] AgentSelectionPanel agentSelectionPanel;
        [SerializeField] Panel mapPanel;
        [SerializeField] SensorEditPanel sensorEditPanel;
        Config.Agent agent;
        Button recognitionButton;
        RecognitionSettingsView recognitionView;

        public void SetAgent(Config.Agent inAgent)
        {
            agent = inAgent;
        }

        public override void LoadPanel()
        {
            base.LoadPanel();

            if (recognitionButton == null)
            {
                recognitionButton = Instantiate(loadAgentButton, loadAgentButton.transform.parent);
                recognitionButton.name = "Recognition settings";
                recognitionButton.onClick = new Button.ButtonClickedEvent();
                var label = recognitionButton.GetComponentInChildren<TMPro.TextMeshProUGUI>(true);
                label.text = "Recognition settings";
                label.enableAutoSizing = true;
                label.fontSizeMin = 20;
                label.fontSizeMax = 30;
                var rect = (RectTransform)recognitionButton.transform;
                rect.sizeDelta = new Vector2(360, 90);
                rect.anchoredPosition = new Vector2(-200, -350);
                var loadRect = (RectTransform)loadAgentButton.transform;
                loadRect.sizeDelta = new Vector2(360, 90);
                loadRect.anchoredPosition = new Vector2(200, -350);
                recognitionView = new RecognitionSettingsView(transform, sensorEditPanel, loadAgentButton);
                recognitionButton.onClick.AddListener(() => recognitionView.Show(agent));
            }
            loadAgentButton.onClick.AddListener(LoadAgent);
            // Reassign function of back button
            GetBackButton().onClick.RemoveAllListeners();
            GetBackButton().onClick.AddListener(ChangeAgent);

            if (agent != null)
            {
                SetTitle(agent.name);
                RemoveAllButtons();
                foreach (string name in agent.sensors.Keys)
                    AddButton(name);
            }
        }

        public override void UnloadPanel()
        {
            recognitionView?.Hide();
            loadAgentButton.onClick.RemoveAllListeners();

            base.UnloadPanel();
        }

        
        protected override void OnAddClicked()
        {
            sensorEditPanel.SetSensor(null, agent);
            UI_Manager.LoadPanel(sensorEditPanel);
        }

        protected override void OnEditClicked()
        {
            sensorEditPanel.SetSensor(agent.sensors[GetActiveButtonLabel()], agent);
            UI_Manager.LoadPanel(sensorEditPanel);
        }

        protected override void OnRemoveClicked()
        {
            agent.sensors.Remove(GetActiveButtonLabel());
            ConfigurationManager.SaveToJSON(agent);
            UI_Manager.LoadPanel(this);
        }

        void LoadAgent()
        {
            ConfigurationManager.SetActiveAgent(agent);
            UI_Manager.LoadPanel(mapPanel);
        }

        void ChangeAgent()
        {
            UI_Manager.LoadPanel(agentSelectionPanel);
        }
    }
}
