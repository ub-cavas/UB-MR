using System;
using CAVAS.UB_MR.Config;
using CAVAS.UI;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Modules.MainMenu
{
    public class AgentCreatePanel : Panel
    {
        [SerializeField] TMP_InputField nameInputField;
        [SerializeField] TMP_Dropdown modelDropdown;
        [SerializeField] Button saveButton;
        [Header("Panels")]
        [SerializeField] AgentSelectionPanel agentSelectionPanel;

        public override void LoadPanel()
        {
            saveButton.onClick.AddListener(CreateAgent);
            base.LoadPanel();
        }

        public override void UnloadPanel()
        {
            saveButton.onClick.RemoveAllListeners();
            base.UnloadPanel();
        }

        void CreateAgent()
        {
            name = nameInputField.text;
            Config.Agent agent = ConfigurationManager.LoadFromJSON(name);
            if (agent is null)
            {
                Config.Agent newAgent = new Config.Agent();
                newAgent.name = name;
                // Visual Model: TODO - Fix dropdown in Unity editor
                string label = modelDropdown.options[modelDropdown.value].text;
                if (Enum.TryParse(label, ignoreCase: true, out VisualModel visualModel))
                    newAgent.model = visualModel;
                else
                    Debug.LogWarning($"'{label}' is not a valid Vehicle Model");
                ConfigurationManager.SaveToJSON(newAgent);
            }
            UI_Manager.LoadPanel(agentSelectionPanel);
        }        
    }
}
