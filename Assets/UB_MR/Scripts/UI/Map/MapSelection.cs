using UnityEngine;

namespace CAVAS.UB_MR.UI.Map
{
    public class MapSelection : Button_Scroller
    {
        protected override void Start()
        {
            base.Start();
            for (int i = 0; i < 3; i++)
                base.AddElement();
        }

        protected override void OnButtonClick()
        {
            print("Test");
        }
    }
}
