using CAVAS.UB_MR.Config;
using CAVAS.UI;
using composition_interfaces.srv;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace CAVAS.UB_MR.Modules.MainMenu
{
    public class MapSelectionPanel : ScrollPanel
    {
        [SerializeField] Button confirmButton;

        [SerializeField] ConfirmationPanel confirmationPanel;

        public override void LoadPanel()
        {
            base.LoadPanel();
            SetTitle("Map Selection");

            confirmButton.onClick.AddListener(Confirm);

            //Placeholder data
            RemoveAllButtons();
            AddButton("UB Service Center");
            AddButton("UB North Campus");
            AddButton("ITU Ayazaga");
        }

        public override void UnloadPanel()
        {
            confirmButton.onClick.RemoveAllListeners();

            base.UnloadPanel();
        }

        protected override void OnAddClicked()
        {
            throw new System.NotImplementedException();
        }

        protected override void OnEditClicked()
        {
            throw new System.NotImplementedException();
        }

        protected override void OnRemoveClicked()
        {
            throw new System.NotImplementedException();
        }

        void Confirm()
        {
            //string mapName = GetActiveButtonLabel(); //TODO: lookup for correct scene name
            string mapName = "service_center_loop";
            ConfigurationManager.SetActiveMap(mapName);
            UI_Manager.LoadPanel(confirmationPanel);
        }
    }
}
