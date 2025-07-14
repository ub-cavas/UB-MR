using UnityEngine;
using CAVAS.UB_MR.UI.Network;

namespace CAVAS.UB_MR.UI
{
    public class MainMenu : MonoBehaviour
    {
        NetworkUI mNetworkUI;

        void Awake()
        {
            this.mNetworkUI = GetComponentInChildren<NetworkUI>(true);
        }

        // Start is called once before the first execution of Update after the MonoBehaviour is created
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }

        public void ExitApplication()
        {
            Application.Quit();
        }
    }

}
