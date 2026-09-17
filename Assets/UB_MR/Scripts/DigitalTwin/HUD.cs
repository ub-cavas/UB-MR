using System;
using CAVAS.UB_MR.UI;

namespace CAVAS.UB_MR.DT
{
    public class HUD
    {
        public Action OnNextSpectatorCamera;
        public Action OnPrevSpectatorCamera;
        public StatPanel statPanel;

        public void ToggleStats()
        {
            if (statPanel != null) statPanel.Toggle();
        }
    }
}
