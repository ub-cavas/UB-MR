using CAVAS.UB_MR.Config;
using UnityEngine;

namespace CAVAS.UB_MR.UI.Vehicle
{
    public abstract class AgentSelection : Button_Scroller
    {
        void Start()
        {
            LoadAllVehicles();
        }

        void LoadAllVehicles()
        {
            foreach (Config.Agent vehicle in ConfigurationManager.GetAllAgents())
                AddElement(vehicle.name);
        }        
    }
}
