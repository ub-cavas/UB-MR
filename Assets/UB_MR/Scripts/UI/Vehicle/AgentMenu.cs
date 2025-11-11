using UnityEngine;
using UnityEngine.UI;
using System;

namespace CAVAS.UB_MR.UI
{
    public class AgentMenu : MonoBehaviour
    {
        [SerializeField] Button addVehicleButton;
        [SerializeField] Button loadVehicleButton;
        [SerializeField] Button editVehicleButton;

        [Header("Menus")]
        [SerializeField] GameObject baseMenu;
        [SerializeField] GameObject agentConfigMenu;
        [SerializeField] GameObject agentSelectionMenu;
        [SerializeField] GameObject agentEditSelectionMenu;

        void OnEnable()
        {
            addVehicleButton.onClick.AddListener(AddVehicle);
            loadVehicleButton.onClick.AddListener(LoadVehicle);
            editVehicleButton.onClick.AddListener(EditVehicle);
        }

        void OnDisable()
        {
            addVehicleButton.onClick.RemoveListener(AddVehicle);
            loadVehicleButton.onClick.RemoveListener(LoadVehicle);
            editVehicleButton.onClick.RemoveListener(EditVehicle);
        }

        public void ExitApplication()
        {
            Application.Quit();
        }

        void AddVehicle()
        {
            agentConfigMenu.SetActive(true);
            baseMenu.SetActive(false);
        }

        void EditVehicle()
        {
            agentEditSelectionMenu.SetActive(true);
            baseMenu.SetActive(false);
        }

        void LoadVehicle()
        {
            agentSelectionMenu.SetActive(true);
            baseMenu.SetActive(false);
        }
    }

}
