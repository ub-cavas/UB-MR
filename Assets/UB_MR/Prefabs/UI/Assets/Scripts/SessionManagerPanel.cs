using CAVAS.UI;
using UnityEngine;
using UnityEngine.UI;


namespace CAVAS.UB_MR.UI
{
    public class SessionManagerPanel : Panel
    {
        [SerializeField] Button newSessionButton;
        [SerializeField] Button loadSessionButton;
        [SerializeField] Button exitButton;
        [SerializeField] Panel agentSelectionPanel;

        public override void LoadPanel()
        {
            loadSessionButton.onClick.AddListener(LoadSession);
            base.LoadPanel();
            
        }

        public override void UnloadPanel()
        {
            loadSessionButton.onClick.RemoveAllListeners();
            base.UnloadPanel();
            
        }

        void LoadSession()
        {
            UI_Manager.LoadPanel(agentSelectionPanel);
        }
    }
}
