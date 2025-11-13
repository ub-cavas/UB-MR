using CAVAS.UB_MR.Config;
using CAVAS.UI;
using UnityEngine;

namespace CAVAS.UB_MR.UI
{
    public class AgentSelectionPanel : ScrollPanel
    {
        [SerializeField] AgentCreatePanel createPanel;
        [SerializeField] AgentEditMenu editPanel;

        public override void LoadPanel()
        {
            foreach (Config.Agent agent in ConfigurationManager.GetAllAgents())
                AddButton(agent.name);
        }

        protected override void OnAddClicked()
        {
            UI_Manager.LoadPanel(createPanel);
        }

        protected override void OnEditClicked()
        {
            Config.Agent agent = ConfigurationManager.LoadFromJSON(GetSelectedButtonLabel());
            if (agent is not null)
            {
                editPanel.SetAgent(agent);
                UI_Manager.LoadPanel(editPanel);
            }
                
        }

        protected override void OnRemoveClicked()
        {
            
        }
    }
}
