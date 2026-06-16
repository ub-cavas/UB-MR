using System;
using CAVAS.UB_MR.UI;

namespace CAVAS.UB_MR.DT
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

        public void EnsureStatsPanel()
        {
            if (statPanel is null)
                statPanel = StatPanel.FindOrCreate();
            StatPanel.EnsureHudToggle(ToggleStats);
        }

        public void ToggleStats()
        {
            EnsureStatsPanel();
            if (statPanel is null)
                return;

            if (statPanel.gameObject.activeInHierarchy)
                UI_Manager.UnloadPanel(statPanel);
            else
                UI_Manager.LoadPanel(statPanel);

            StatPanel.UpdateHudToggleState(statPanel.gameObject.activeInHierarchy);
        }
    }
}
