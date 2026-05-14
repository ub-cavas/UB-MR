using CAVAS.UB_MR.Config;
using CAVAS.UI;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Modules.MainMenu
{
    public class AgentSelectionPanel : ScrollPanel
    {
        [SerializeField] Button loadAgentButton;
        [SerializeField] MapSelectionPanel mapSelectionPanel;
        [SerializeField] AgentCreatePanel createPanel;
        [SerializeField] AgentEditMenu editPanel;

        public override void LoadPanel()
        {
            base.LoadPanel();
            loadAgentButton.onClick.AddListener(LoadAgent);

            RemoveAllButtons();
            foreach (Config.Agent agent in ConfigurationManager.GetAllAgents())
                AddButton(agent.name);

            SetTitle("Select Agent");
        }

        public override void UnloadPanel()
        {
            loadAgentButton.onClick.RemoveAllListeners();

            base.UnloadPanel();
        }

        protected override void OnAddClicked()
        {
            UI_Manager.LoadPanel(createPanel);
        }

        protected override void OnEditClicked()
        {
            Config.Agent agent = ConfigurationManager.LoadFromJSON(GetActiveButtonLabel());
            if (agent is not null)
            {
                editPanel.SetAgent(agent);
                UI_Manager.LoadPanel(editPanel);
            }
                
        }

        protected override void OnRemoveClicked()
        {
            Config.Agent agent = ConfigurationManager.LoadFromJSON(GetActiveButtonLabel());
            ConfigurationManager.RemoveAgent(agent.name);
            UI_Manager.LoadPanel(this);
        }

        void LoadAgent()
        {
            Config.Agent agent = ConfigurationManager.LoadFromJSON(GetActiveButtonLabel());
            ConfigurationManager.SetActiveAgent(agent);
            UI_Manager.LoadPanel(mapSelectionPanel);
        }
    }
}
