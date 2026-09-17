using System.Collections.Generic;
using CAVAS.UB_MR;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    public class TrafficRenderer : MonoBehaviour
    {
        [SerializeField] private TrafficReceiver receiver;
        [SerializeField] private GameObject vehiclePrefab;
        [SerializeField] private Module module;

        private CarlaMapFrame _mapFrame;
        private bool _waitingForModule;
        private readonly Dictionary<string, GameObject> spawnedVehicles = new();

        void Awake()
        {
            ResolveMapFrame();
        }

        void OnEnable()
        {
            ResolveMapFrame();
            receiver.OnSpawnVehicle += HandleSpawn;
            receiver.OnDespawnVehicle += HandleDespawn;
            // Reconcile messages received while rendering was disabled.
            foreach (var data in receiver.KnownVehicles.Values)
                HandleSpawn(data);
        }

        void OnDisable()
        {
            receiver.OnSpawnVehicle -= HandleSpawn;
            receiver.OnDespawnVehicle -= HandleDespawn;
            foreach (GameObject vehicle in spawnedVehicles.Values)
                Destroy(vehicle);
            spawnedVehicles.Clear();
        }

        void LateUpdate()
        {
            CarlaMapFrame frame = ResolveMapFrame();
            foreach (var entry in spawnedVehicles)
            {
                if (!receiver.KnownVehicles.TryGetValue(entry.Key, out var data)) continue;
                GameObject vehicle = entry.Value;
                Vector3 location = new(data.location.x, data.location.y, data.location.z);
                if (frame == null || !frame.TryCarlaPoseToUnityWorld(location, data.yaw, out Pose pose))
                {
                    vehicle.SetActive(false);
                    continue;
                }

                // Always reapply cached poses: alignment can change without a new packet.
                vehicle.transform.SetPositionAndRotation(pose.position, pose.rotation);
                vehicle.SetActive(true);
            }
        }

        private void HandleSpawn(TrafficReceiver.VehicleData data)
        {
            if (spawnedVehicles.ContainsKey(data.id)) return;

            GameObject go = Instantiate(vehiclePrefab);
            go.name = $"Vehicle_{data.id}";
            go.SetActive(false); // Remain hidden until a valid map-relative pose is applied.
            spawnedVehicles[data.id] = go;
        }

        private void HandleDespawn(TrafficReceiver.VehicleData data)
        {
            if (!spawnedVehicles.TryGetValue(data.id, out GameObject go)) return;

            Destroy(go);
            spawnedVehicles.Remove(data.id);
        }

        private CarlaMapFrame ResolveMapFrame()
        {
            if (!Application.isPlaying) return null;
            if (_mapFrame != null && module != null) return _mapFrame;

            if (module == null)
                module = FindFirstObjectByType<Module>();
            if (module == null)
            {
                if (!_waitingForModule)
                    Debug.LogWarning("[TrafficRenderer] Waiting for a Module before rendering poses.", this);
                _waitingForModule = true;
                return null;
            }

            if (_waitingForModule)
                Debug.Log("[TrafficRenderer] Module acquired; pose conversion available.", this);
            _waitingForModule = false;
            _mapFrame = CarlaMapFrame.GetOrCreate(module);
            return _mapFrame;
        }
    }
}
