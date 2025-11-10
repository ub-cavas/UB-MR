using UnityEngine;

namespace CAVAS.UB_MR.UI.Vehicle
{
    public class VehicleSelection : Button_Scroller
    {
        void Start()
        {
            AddElement("Lincoln MKZ");
        }

        protected override void OnButtonClick()
        {
            //TODO: Load the vehicle into memory
        }

        
    }
}
