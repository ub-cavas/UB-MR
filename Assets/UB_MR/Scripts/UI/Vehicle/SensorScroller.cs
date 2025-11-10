using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

namespace CAVAS.UB_MR.UI.Vehicle
{
    public class SensorScroller : Button_Scroller
    {
        public Action<string> OnSensorClicked;

        protected override void OnButtonClick()
        {
            Button selected = TryGetSelectedButton();
            if (selected is not null)
            {
                string sensorName = selected.GetComponentInChildren<TextMeshProUGUI>().text;
                OnSensorClicked?.Invoke(sensorName);
            }
            
        }
    }

}

