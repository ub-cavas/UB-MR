using UnityEngine;
using UnityEngine.UI;
using System;

namespace CAVAS.UB_MR.UI
{
    public class MainMenu : MonoBehaviour
    {
        public static event Action OnOpenMainMenu;
        public static event Action OnOpenNetworkMenu;
        public static event Action OnOpenStatsMenu;

        [SerializeField] GameObject mPanel;

        public void ExitApplication()
        {
            Application.Quit();
        }

        public void OpenNetworkMenu()
        {
            OnOpenNetworkMenu?.Invoke();
            TogglePanel();
        }

        public void OpenStatsMenu()
        {
            OnOpenStatsMenu?.Invoke();
        }

        public void TogglePanel()
        {
            this.mPanel.SetActive(!this.mPanel.activeInHierarchy);
            if (this.mPanel.activeInHierarchy)
            {
                OnOpenMainMenu?.Invoke();
            }

        }
    }

}
