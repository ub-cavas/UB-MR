using System;
using CAVAS.UB_MR.UI.Vehicle;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

namespace CAVAS.UB_MR.UI.Vehicle
{
    public class AgentSelectionMenu : AgentSelection
    {
        public Action<string> OnAgentSelected;
        protected override void OnButtonClick()
        {
            Button button = TryGetSelectedButton();
            if (button is not null)
            {
                string agentName = button.GetComponentInChildren<TextMeshProUGUI>().text;
                OnAgentSelected?.Invoke(agentName);
            }
        }
    }
}
