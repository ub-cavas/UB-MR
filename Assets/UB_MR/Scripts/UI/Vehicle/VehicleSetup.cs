using UnityEngine;
using CAVAS.UB_MR.Config;


namespace CAVAS.UB_MR.UI.Vehicle
{
    public class VehicleSetup : MonoBehaviour
    {
        public string vehicle_name;


        public void SaveVehicle()
        {
            Config.Vehicle vehicle = new Config.Vehicle();
            vehicle.vehicle_name = this.vehicle_name;
            ConfigManager.Save(vehicle);
        }
    }
}
