using UnityEngine;
using UnityEngine.UI;
using System;

namespace CAVAS.UB_MR.UI
{
    public class MainMenu : MonoBehaviour
    {
        [SerializeField] Button addVehicleButton;
        [SerializeField] Button loadVehicleButton;
        [SerializeField] Button editVehicleButton;

        public void Awake()
        {
            
        }

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

        }

        void EditVehicle()
        {

        }

        void LoadVehicle()
        { 

        }
    }

}
