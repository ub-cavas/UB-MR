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
        [SerializeField] private Transform mapRoot;
        [Tooltip("Apply only the runtime client-local Unity Y rotation delta from the map UI.")]
        [SerializeField] private bool applyClientMapYawCorrection = true;
        [Tooltip("Map UI Y rotation that corresponds to the uncorrected CARLA/RoadRunner traffic frame.")]
        [SerializeField] private float uncorrectedMapYawDegrees = 90f;
        [SerializeField] private Vector3 originOffset = new Vector3(1.347f, 0f, 5.916f);

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
            ResolveModule();
            ResolveMapRoot();
        }

        void OnEnable()
        {
            ResolveModule();
            ResolveMapRoot();
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
            // Fallback appearance remains authored, even if it has paint bindings.
            if (mapped) instance.appearance = instance.root.GetComponent<TrafficVehicleAppearance>();
            instance.appearance?.ApplyColor(data.color, materials);
            ApplyPose(instance.root.transform, data);
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
            ApplyPose(instance.root.transform, data);
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

        private void ApplyPose(Transform vehicleTransform, TrafficReceiver.VehicleData data)
        {
            Vector3 mapLocalPosition = data.Position() + originOffset;
            Quaternion mapLocalRotation = data.Orientation();
            float clientMapYawDelta = GetClientMapYawDeltaDegrees();

            if (!Mathf.Approximately(clientMapYawDelta, 0f))
            {
                Quaternion trafficYawCorrection = Quaternion.Euler(0f, clientMapYawDelta, 0f);

                vehicleTransform.SetPositionAndRotation(
                    trafficYawCorrection * mapLocalPosition,
                    trafficYawCorrection * mapLocalRotation
                );
                return;
            }

            vehicleTransform.SetPositionAndRotation(mapLocalPosition, mapLocalRotation);
        }

        private float GetClientMapYawDeltaDegrees()
        {
            if (!applyClientMapYawCorrection)
                return 0f;

            Module mapModule = ResolveModule();
            if (mapModule != null && mapModule.HasMapRotationState)
            {
                return Mathf.DeltaAngle(uncorrectedMapYawDegrees, mapModule.CurrentMapRotationEuler.y);
            }

            if (!TryGetMapRoot(out Transform root))
                return 0f;

            return Mathf.DeltaAngle(uncorrectedMapYawDegrees, root.eulerAngles.y);
        }

        private bool TryGetMapRoot(out Transform root)
        {
            root = ResolveMapRoot();
            return root != null;
        }

        private Transform ResolveMapRoot()
        {
            if (mapRoot != null)
                return mapRoot;

            Module mapModule = ResolveModule();
            if (mapModule != null)
                mapRoot = mapModule.MapRoot;

            return mapRoot;
        }

        private Module ResolveModule()
        {
            if (module != null)
                return module;

            module = FindFirstObjectByType<Module>();
            return module;
        }
    }
}
