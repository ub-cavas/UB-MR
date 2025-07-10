using UnityEngine;
using Unity.Netcode;


namespace CAVAS.UB_MR.UI.Network
{
    public class NetworkUI : MonoBehaviour
    {
        public void StartHost()
        {
            NetworkManager.Singleton.StartHost();
        }

        public void StartClient()
        {
            NetworkManager.Singleton.StartClient();
        }
    }
}
