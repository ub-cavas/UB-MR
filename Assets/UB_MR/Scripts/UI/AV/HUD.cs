using UnityEngine;
using UnityEngine.UI;
using CAVAS.UB_MR.DT;

namespace CAVAS.UB_MR.UI.AV
{
    public class HUD : MonoBehaviour
    {
        Stats[] mStatPanels;

        // Start is called once before the first execution of Update after the MonoBehaviour is created
        void Start()
        {
            this.mStatPanels = this.GetComponentsInChildren<Stats>(true);
          
        }

        void ToggleHUD()
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

       
    }
}
