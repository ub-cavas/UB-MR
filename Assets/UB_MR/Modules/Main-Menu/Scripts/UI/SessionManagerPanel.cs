using CAVAS.UI;
using UnityEngine;
using UnityEngine.UI;


namespace CAVAS.UB_MR.Modules.MainMenu
{
    public class SessionManagerPanel : Panel
    {
        [SerializeField] Button newSessionButton;
        [SerializeField] Button loadSessionButton;
        [SerializeField] Button exitButton;
        [Header("Panels")]
        [SerializeField] Panel agentSelectionPanel;
        [SerializeField] Panel confirmationPanel;

        public override void LoadPanel()
        {
            newSessionButton.onClick.AddListener(NewSession);
            loadSessionButton.onClick.AddListener(LoadSession);
            exitButton.onClick.AddListener(ExitApp);
            base.LoadPanel();
        }

        public override void UnloadPanel()
        {
            newSessionButton.onClick.RemoveAllListeners();
            loadSessionButton.onClick.RemoveAllListeners();
            exitButton.onClick.RemoveAllListeners();
            base.UnloadPanel();
        }

        void NewSession()
        {
            UI_Manager.LoadPanel(agentSelectionPanel);
        }

        void LoadSession()
        {
            //TODO: Read a config file and set ConfigurationManager variables
            UI_Manager.LoadPanel(confirmationPanel);
        }

        void ExitApp()
        {
            Application.Quit();
        }
    }
}
