using UnityEngine;
using UnityEngine.UI;
using System;

namespace CAVAS.UB_MR.UI
{
    public class VehicleMenu : MonoBehaviour
    {
        [SerializeField] Button addVehicleButton;
        [SerializeField] Button loadVehicleButton;
        [SerializeField] Button editVehicleButton;

        [Header("Menus")]
        [SerializeField] GameObject vehicleConfigMenu;
        [SerializeField] GameObject vehicleSelectionMenu;

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
            vehicleConfigMenu.SetActive(true);
            //this.gameObject.SetActive(false);
        }

        void EditVehicle()
        {
            vehicleSelectionMenu.SetActive(true);
            this.gameObject.SetActive(false);
        }

        void LoadVehicle()
        {
            vehicleSelectionMenu.SetActive(true);
            this.gameObject.SetActive(false);
        }
    }

}
