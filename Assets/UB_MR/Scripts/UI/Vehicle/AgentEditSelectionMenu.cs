using UnityEngine;
using System;
using TMPro;
using UnityEngine.UI;
using CAVAS.UB_MR.Config;

namespace CAVAS.UB_MR.UI.Vehicle
{
    public class AgentEditSelectionMenu : AgentSelection
    {
        [SerializeField] AgentSetupMenu agentSetupMenu;
        public Action<Agent> OnAgentSelected;
        protected override void OnButtonClick()
        {
            Button button = TryGetSelectedButton();
            if (button is not null)
            {
                string agentName = button.GetComponentInChildren<TextMeshProUGUI>().text;
                Agent agent = ConfigurationManager.LoadFromJSON(agentName);
                agentSetupMenu.OpenEditAgentMenu(agent);
                this.gameObject.SetActive(false);
            }
        }
    }
}
