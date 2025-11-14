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
            newSessionButton.onClick.AddListener(NewSession);
            base.LoadPanel();
            
        }

        public override void UnloadPanel()
        {
            newSessionButton.onClick.RemoveAllListeners();
            base.UnloadPanel();
            
        }

        void NewSession()
        {
            UI_Manager.LoadPanel(agentSelectionPanel);
        }
    }
}
