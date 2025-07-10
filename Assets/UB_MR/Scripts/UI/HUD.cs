using UnityEngine;
using UnityEngine.UI;
using CAVAS.UB_MR.DT;

namespace CAVAS.UB_MR.UI
{
    public class HUD : MonoBehaviour
    {
        [SerializeField] Button mHUD_Button;
        Stats[] mStatPanels;

        // Start is called once before the first execution of Update after the MonoBehaviour is created
        void Start()
        {
            this.mStatPanels = this.GetComponentsInChildren<Stats>(true);

            this.mHUD_Button.onClick.AddListener(this.ToggleHUD);
            this.mHUD_Button.onClick.AddListener(this.ToggleGhost);
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

        void ToggleGhost()
        {
            DT_Reflect ghost = Object.FindFirstObjectByType<DT_Reflect>(FindObjectsInactive.Include);
            if (ghost != null)
            {
                ghost.gameObject.SetActive(!ghost.gameObject.activeSelf);
            }
        }
    }
}
