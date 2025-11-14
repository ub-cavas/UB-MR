using CAVAS.UB_MR.Config;
using CAVAS.UI;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI
{
    public class AgentCreatePanel : Panel
    {
        [SerializeField] TMP_InputField nameInputField;
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
                ConfigurationManager.SaveToJSON(newAgent);
            }
            UI_Manager.LoadPanel(agentSelectionPanel);
        }        
    }
}
