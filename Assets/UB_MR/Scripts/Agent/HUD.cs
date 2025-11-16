using UnityEngine;
using CAVAS.UB_MR.DT;
using CAVAS.UI;
using System;
using Unity.Cinemachine;
using CAVAS.UB_MR.UI;

namespace CAVAS.UB_MR.Agent
{
    public class HUD
    {
        public Action OnNextSpectatorCamera;
        public Action OnPrevSpectatorCamera;
        public StatPanel statPanel;

        public void AddStat()
        {
            //TODO: 
        }

        public void ToggleStats()
        {
            if (statPanel.gameObject.activeInHierarchy)
                UI_Manager.UnloadPanel(statPanel);
            else
                UI_Manager.LoadPanel(statPanel);
        }
    }


       /* void ToggleHUD()
        {
            // Stat Panels
            foreach (Stats statPanel in this.mStatPanels)
            {
                if (statPanel != null)
                {
                    statPanel.gameObject.SetActive(!statPanel.gameObject.activeSelf);
                }
            }
        }

        public void DashCam()
        {
            if (this.mAutonomousVehicle != null)
            {
                this.mAutonomousVehicle.EnableDashCam(true);
            }
        }

        public void FollowCam()
        {
            if (this.mAutonomousVehicle != null)
            {
                this.mAutonomousVehicle.EnableFollowCam(true);
            }
        }

        public void ToggleEnvironmentVisibility(bool inVisible)
        {
            this.mAutonomousVehicle.SetLayerCulling(Camera.main, "Environment", inVisible);
        }*/
}
