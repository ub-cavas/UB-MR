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

        public void SetAgent(Config.Agent inAgent)
        {
            agent = inAgent;
        }

        public override void LoadPanel()
        {
            base.LoadPanel();

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
