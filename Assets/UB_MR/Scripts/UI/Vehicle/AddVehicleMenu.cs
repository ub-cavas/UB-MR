using UnityEngine;
using CAVAS.UB_MR.Config;
using UnityEngine.UI;


namespace CAVAS.UB_MR.UI.Vehicle
{
    public class VehicleSetup : MonoBehaviour
    {
        [SerializeField] Button confirmButton;
        public string vehicle_name;

        void OnEnable()
        {
            confirmButton.onClick.AddListener(SaveVehicle);
        }

        void OnDisable()
        {
            confirmButton.onClick.RemoveAllListeners();
        }

        public void SaveVehicle()
        {
            Config.Vehicle vehicle = new Config.Vehicle();
            vehicle.vehicle_name = this.vehicle_name;
            JSONManager.SaveToJSON(vehicle);
        }
    }
}
