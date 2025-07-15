using UnityEngine;
using Unity.Netcode;
using CAVAS.UB_MR.UI;


namespace CAVAS.UB_MR.UI.Network
{
    public class NetworkUI : NetworkBehaviour
    {
        [SerializeField] GameObject mSpectatorPrefab;
        [SerializeField] GameObject mPredictiveDigitalTwinPrefab;
        [SerializeField] GameObject mGPSDigitalTwinPrefab;
        [Space]
        [SerializeField] bool mAutoDisable = true;
        [SerializeField] RectTransform mRoleSelectionPanel;
        [SerializeField] RectTransform mConnectionPanel;

        Canvas mCanvas;

        void Awake()
        {
            this.mCanvas = GetComponent<Canvas>();
            
            if (mAutoDisable)
                this.mCanvas.enabled = false;
        }

        void OnEnable()
        {
            MainMenu.OnOpenNetworkMenu += EnableMenu;
            MainMenu.OnOpenMainMenu += DisableMenu;
            
        }

        void OnDisable()
        {
            MainMenu.OnOpenNetworkMenu -= EnableMenu;
            MainMenu.OnOpenMainMenu -= DisableMenu;
            
        }

        public void StartHost()
        {
            NetworkManager.Singleton.StartHost();
        }

        public void StartClient()
        {
            NetworkManager.Singleton.StartClient();
        }

        void DisableMenu()
        {
            this.mCanvas.enabled = false;
        }

        void EnableMenu()
        {
            this.mCanvas.enabled = true;

            if (NetworkManager.Singleton.IsConnectedClient && !HasActiveChild(this.mCanvas.gameObject))
            {
                this.mConnectionPanel.gameObject.SetActive(true);
            }
            else if (!NetworkManager.Singleton.IsConnectedClient && !HasActiveChild(this.mCanvas.gameObject))
            {
                this.mRoleSelectionPanel.gameObject.SetActive(true);
            }
        }

        public bool HasActiveChild(GameObject parent)
        {
            if (parent == null || parent.transform.childCount == 0)
                return false;

            for (int i = 0; i < parent.transform.childCount; i++)
            {
                if (parent.transform.GetChild(i).gameObject.activeInHierarchy)
                {
                    return true;
                }
            }
            return false;
        }

        public void Disconnect()
        {
            NetworkManager.Singleton.Shutdown();
        }

        public void SpawnSpectator()
        {
            SpawnSpectatorServerRpc();
        }

        public void SpawnDigitalTwin(bool inPredictive)
        {
            if (inPredictive)
            {
                SpawnPredictiveDigitalTwinServerRpc();
            }
            else
            {
                SpawnGPSDigitalTwinServerRpc();
            }
        }
        

        [ServerRpc(RequireOwnership = false)]
        void SpawnSpectatorServerRpc(ServerRpcParams rpcParams = default)
        {
            ulong callerClientId = rpcParams.Receive.SenderClientId;
            NetworkSpawn(mSpectatorPrefab, callerClientId);
        }

        [ServerRpc(RequireOwnership = false)]
        public void SpawnPredictiveDigitalTwinServerRpc(ServerRpcParams rpcParams = default)
        {
            ulong callerClientId = rpcParams.Receive.SenderClientId;
            NetworkSpawn(mPredictiveDigitalTwinPrefab, callerClientId);
        }

        [ServerRpc(RequireOwnership = false)]
        public void SpawnGPSDigitalTwinServerRpc(ServerRpcParams rpcParams = default)
        {
            ulong callerClientId = rpcParams.Receive.SenderClientId;
            NetworkSpawn(mGPSDigitalTwinPrefab, callerClientId);
        }

        
        void NetworkSpawn(GameObject prefab, ulong client_id)
        {
            //Debug.Log("Spawning " + prefab.name + " for client " + client_id);
            var instance = Instantiate(prefab);
            var instanceNetworkObject = instance.GetComponent<NetworkObject>();
            instanceNetworkObject.SpawnWithOwnership(client_id);
        }
    }
}
