using System.Collections.Generic;
using CAVAS.UB_MR;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    public class TrafficRenderer : MonoBehaviour
    {
        [SerializeField] private TrafficReceiver receiver;
        [SerializeField] private GameObject vehiclePrefab;
        [SerializeField] private TrafficVehicleCatalog vehicleCatalog;
        [SerializeField] private Module module;
        private CarlaMapFrame _mapFrame;
        private bool _waitingForModule;

        private sealed class VehicleInstance
        {
            public GameObject root;
            public string blueprint;
            public TrafficVehicleAppearance appearance;
            public string color;
        }

        private readonly Dictionary<string, VehicleInstance> spawnedVehicles = new();
        private readonly HashSet<string> warnedBlueprints = new(System.StringComparer.Ordinal);
        private readonly HashSet<string> missingFallbackBlueprints = new(System.StringComparer.Ordinal);
        private TrafficMaterialCache materials;
        private TrafficReceiver subscribedReceiver;
        private bool warnedCatalog;

        void Awake()
        {
            ResolveMapFrame();
        }

        void OnEnable()
        {
            ResolveMapFrame();
            if (receiver == null)
            {
                Debug.LogError("TrafficRenderer requires a TrafficReceiver.", this);
                return;
            }
            string error = "No vehicle catalog assigned.";
            if ((vehicleCatalog == null || !vehicleCatalog.Initialize(out error)) && !warnedCatalog)
            {
                Debug.LogError($"TrafficRenderer: {error} Using fallback vehicles.", this);
                warnedCatalog = true;
            }
            materials = new TrafficMaterialCache();
            subscribedReceiver = receiver;
            subscribedReceiver.OnSpawnVehicle += HandleSpawn;
            subscribedReceiver.OnVehicleUpdate += HandleUpdate;
            subscribedReceiver.OnDespawnVehicle += HandleDespawn;
            foreach (var data in subscribedReceiver.KnownVehicles.Values) HandleSpawn(data);
        }

        void OnDisable()
        {
            if (subscribedReceiver != null)
            {
                subscribedReceiver.OnSpawnVehicle -= HandleSpawn;
                subscribedReceiver.OnVehicleUpdate -= HandleUpdate;
                subscribedReceiver.OnDespawnVehicle -= HandleDespawn;
            }
            subscribedReceiver = null;
            foreach (var instance in spawnedVehicles.Values) DestroyInstance(instance);
            spawnedVehicles.Clear();
            materials?.Dispose();
            materials = null;
        }

        void LateUpdate()
        {
            if (subscribedReceiver == null) return;
            CarlaMapFrame frame = ResolveMapFrame();
            foreach (var entry in spawnedVehicles)
            {
                if (!subscribedReceiver.KnownVehicles.TryGetValue(entry.Key, out var data) || data.location == null) continue;
                GameObject vehicle = entry.Value.root;
                if (vehicle == null) continue;
                Vector3 location = new(data.location.x, data.location.y, data.location.z);
                if (frame == null || !frame.TryCarlaPoseToUnityWorld(location, data.yaw, out Pose pose))
                {
                    vehicle.SetActive(false);
                    continue;
                }

                // Always reapply cached poses: alignment can change without a new packet.
                vehicle.transform.SetPositionAndRotation(pose.position, pose.rotation);
                if (!vehicle.activeSelf)
                    entry.Value.appearance?.ApplyColor(entry.Value.color, materials);
                vehicle.SetActive(true);
            }
        }

        private void HandleSpawn(TrafficReceiver.VehicleData data)
        {
            if (data == null || string.IsNullOrEmpty(data.id) || data.location == null) return;
            if (spawnedVehicles.ContainsKey(data.id)) { HandleUpdate(data); return; }
            bool mapped = vehicleCatalog != null && vehicleCatalog.TryResolve(data.blueprint, out _);
            GameObject prefab = vehiclePrefab;
            if (mapped) vehicleCatalog.TryResolve(data.blueprint, out prefab);
            else
            {
                string key = data.blueprint ?? "";
                if (warnedBlueprints.Add(key))
                    Debug.LogWarning($"TrafficRenderer: unmapped blueprint '{key}'; using fallback.", this);
                if (prefab == null && missingFallbackBlueprints.Add(key))
                    Debug.LogError($"TrafficRenderer: no fallback for '{key}'; actor skipped.", this);
            }
            if (prefab == null) return;
            var instance = new VehicleInstance
            {
                root = Instantiate(prefab), blueprint = data.blueprint, color = data.color
            };
            instance.root.name = $"Vehicle_{data.id}";
            instance.root.SetActive(false); // Wait for a valid map-relative pose in LateUpdate.
            // Fallback appearance remains authored, even if it has paint bindings.
            if (mapped) instance.appearance = instance.root.GetComponent<TrafficVehicleAppearance>();
            instance.appearance?.ApplyColor(data.color, materials);
            spawnedVehicles.Add(data.id, instance);
        }

        private void HandleUpdate(TrafficReceiver.VehicleData data)
        {
            if (data == null || string.IsNullOrEmpty(data.id) || data.location == null) return;
            if (!spawnedVehicles.TryGetValue(data.id, out var instance)) { HandleSpawn(data); return; }
            if (instance.root == null || instance.blueprint != data.blueprint)
            {
                DestroyInstance(instance);
                spawnedVehicles.Remove(data.id);
                HandleSpawn(data);
                return;
            }
            if (instance.color != data.color)
            {
                instance.appearance?.ApplyColor(data.color, materials);
                instance.color = data.color;
            }
        }

        private void HandleDespawn(TrafficReceiver.VehicleData data)
        {
            if (data == null || string.IsNullOrEmpty(data.id)
                || !spawnedVehicles.TryGetValue(data.id, out var instance)) return;
            DestroyInstance(instance);
            spawnedVehicles.Remove(data.id);
        }

        private static void DestroyInstance(VehicleInstance instance)
        {
            if (instance.root == null) return;
            instance.root.SetActive(false);
            instance.appearance?.ReleaseColor();
            TrafficMaterialCache.DestroyOwned(instance.root);
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
