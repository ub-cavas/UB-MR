using Unity.Netcode;
using System.Collections.Generic;
using UnityEngine;
using CAVAS.UB_MR.DT;

namespace CAVAS.UB_MR
{
    public class ObservableObjectTracker : MonoBehaviour
    {
        private List<NetworkObject> observableNetworkObjects = new List<NetworkObject>();
        private HashSet<ulong> trackedObjectIds = new HashSet<ulong>();
        private NetworkManager networkManager;
        public List<NetworkObject> ObservableNetworkObjects => observableNetworkObjects;

        void Start()
        {
            networkManager = NetworkManager.Singleton;

            if (networkManager != null)
            {
                networkManager.OnClientConnectedCallback += OnClientConnected;
                networkManager.OnClientDisconnectCallback += OnClientDisconnected;

                // Initialize with existing objects if we're already connected
                if (networkManager.IsClient || networkManager.IsServer)
                {
                    InitializeExistingObjects();
                }

            }
        }

        void OnEnable()
        {
            DigitalTwin.OnSpawn += ForceUpdate;
        }

        void OnDisable()
        {
            DigitalTwin.OnSpawn -= ForceUpdate;
        }

        void OnDestroy()
        {
            if (networkManager != null)
            {
                networkManager.OnClientConnectedCallback -= OnClientConnected;
                networkManager.OnClientDisconnectCallback -= OnClientDisconnected;
            }
        }

        private void OnClientConnected(ulong clientId)
        {
            // Refresh the list when we connect
            if (clientId == NetworkManager.Singleton.LocalClientId)
            {
                InitializeExistingObjects();
            }
            ForceUpdate();
        }

        private void OnClientDisconnected(ulong clientId)
        {
            // Objects should be automatically despawned, but we can do cleanup here if needed
        }

        private void InitializeExistingObjects()
        {
            observableNetworkObjects.Clear();
            trackedObjectIds.Clear();

            // Check all currently spawned objects
            foreach (var networkObject in NetworkManager.Singleton.SpawnManager.SpawnedObjects.Values)
            {
                if (networkObject.TryGetComponent<Observable>(out _))
                {
                    observableNetworkObjects.Add(networkObject);
                    trackedObjectIds.Add(networkObject.NetworkObjectId);
                }
            }
        }

        private void CheckForNewOrRemovedObjects()
        {
            var currentSpawnedObjects = NetworkManager.Singleton.SpawnManager.SpawnedObjects.Values;
            var currentObservableIds = new HashSet<ulong>();

            // Check for new objects
            foreach (var networkObject in currentSpawnedObjects)
            {
                if (networkObject.TryGetComponent<Observable>(out _))
                {
                    currentObservableIds.Add(networkObject.NetworkObjectId);

                    // If this is a new object, add it to our list
                    if (!trackedObjectIds.Contains(networkObject.NetworkObjectId))
                    {
                        observableNetworkObjects.Add(networkObject);
                        trackedObjectIds.Add(networkObject.NetworkObjectId);
                    }
                }
            }

            // Check for removed objects
            for (int i = observableNetworkObjects.Count - 1; i >= 0; i--)
            {
                var networkObject = observableNetworkObjects[i];

                // If the object no longer exists or no longer has the Observable component
                if (networkObject == null || !currentObservableIds.Contains(networkObject.NetworkObjectId))
                {
                    trackedObjectIds.Remove(networkObject.NetworkObjectId);
                    observableNetworkObjects.RemoveAt(i);
                }
            }
        }

        // Optional: Get observable objects by client ID
        public List<NetworkObject> GetObservableObjectsForClient(ulong clientId)
        {
            var clientObjects = new List<NetworkObject>();

            foreach (var networkObject in observableNetworkObjects)
            {
                if (networkObject != null && networkObject.OwnerClientId == clientId)
                {
                    clientObjects.Add(networkObject);
                }
            }

            return clientObjects;
        }

        void ForceUpdate()
        {
            CheckForNewOrRemovedObjects();
        }
    }
}
