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

        public override void LoadPanel()
        {
            base.LoadPanel();
            saveButton.onClick.AddListener(CreateAgent);
        }

        public override void UnloadPanel()
        {
            base.UnloadPanel();
            saveButton.onClick.RemoveAllListeners();
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
            Back();
        }        
    }
}
