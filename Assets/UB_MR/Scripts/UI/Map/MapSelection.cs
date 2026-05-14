using UnityEngine;

namespace CAVAS.UB_MR.UI.Map
{
    public class MapSelection : Button_Scroller
    {
        void Start()
        {
            AddElement("UB Service Center");
        }

        protected override void OnButtonClick()
        {
            //TODO: Load the respective map in the background
        }
    }
}
