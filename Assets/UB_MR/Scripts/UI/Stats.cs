using UnityEngine;
using TMPro;
using CAVAS.UB_MR.DT;
using CAVAS.UI;

namespace CAVAS.UB_MR.UI
{
     
    public class StatPanel : Panel
    {
        void Update()
        {
            UpdateStatsGUI();
        }

        public void AddStatistic()
        {
            //TODO: Add new stat UI setup
        }

        public void UpdateStatsGUI()
        {
           
            Vector3 linear = new Vector3(0, 0, 0);
            string display = string.Format("Linear: ({0:0.00}, {1:0.00}, {2:0.00})m/s", linear.x, linear.y, linear.z);
            //this.mLinearVelocityText.text = display;
        }

        //TODO: Implement generic update procedures
    }
}
