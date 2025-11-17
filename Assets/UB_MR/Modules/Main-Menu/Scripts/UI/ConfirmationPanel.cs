using System;
using CAVAS.UB_MR.Config;
using CAVAS.UI;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Modules.MainMenu
{
    public class ConfirmationPanel : Panel
    {
        [SerializeField] Button agentButton;
        [SerializeField] Button mapButton;
        [SerializeField] Button startButton;

        [Header("Panels")]
        [SerializeField] MapSelectionPanel mapSelectionPanel;
        [SerializeField] AgentSelectionPanel agentSelectionPanel;

        public override void LoadPanel()
        {
            base.LoadPanel();

            // Update the button labels
            Tuple<Config.Agent, string> config = ConfigurationManager.GetConfiguration();
            if (config.Item1 is null)
                agentButton.GetComponentInChildren<TextMeshProUGUI>().text = "Change Agent";
            else
                agentButton.GetComponentInChildren<TextMeshProUGUI>().text = config.Item1.name;

            if (config.Item2 is null)
                mapButton.GetComponentInChildren<TextMeshProUGUI>().text = "Change Map";
            else
                mapButton.GetComponentInChildren<TextMeshProUGUI>().text = config.Item2;

            agentButton.onClick.AddListener(SelectAgent);
            mapButton.onClick.AddListener(SelectMap);
            startButton.onClick.AddListener(StartMR);
        }

        public override void UnloadPanel()
        {
            agentButton.onClick.RemoveAllListeners();
            mapButton.onClick.RemoveAllListeners();
            startButton.onClick.RemoveAllListeners();
            base.UnloadPanel();
        }

        void SelectAgent()
        {
            UI_Manager.LoadPanel(agentSelectionPanel);
        }

        void SelectMap()
        {
            UI_Manager.LoadPanel(mapSelectionPanel);
        }

        void StartMR()
        {
            Tuple<Config.Agent, string> config = ConfigurationManager.GetConfiguration();
            SceneManager.LoadScene(config.Item2);
        }
    }
}
