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
        [SerializeField] private Transform mapRoot;
        [Tooltip("Apply only the runtime client-local Unity Y rotation delta from the map UI.")]
        [SerializeField] private bool applyClientMapYawCorrection = true;
        [Tooltip("Map UI Y rotation that corresponds to the uncorrected CARLA/RoadRunner traffic frame.")]
        [SerializeField] private float uncorrectedMapYawDegrees = 90f;
        [SerializeField] private Vector3 originOffset = new Vector3(1.347f, 0f, 5.916f);

        private Dictionary<string, GameObject> spawnedVehicles = new();

        void Awake()
        {
            ResolveModule();
            ResolveMapRoot();
        }

        void OnEnable()
        {
            ResolveModule();
            ResolveMapRoot();
            receiver.OnSpawnVehicle += HandleSpawn;
            receiver.OnVehicleUpdate += HandleUpdate;
            receiver.OnDespawnVehicle += HandleDespawn;
        }

        void OnDisable()
        {
            receiver.OnSpawnVehicle -= HandleSpawn;
            receiver.OnVehicleUpdate -= HandleUpdate;
            receiver.OnDespawnVehicle -= HandleDespawn;
        }

        private void HandleSpawn(TrafficReceiver.VehicleData data)
        {
            if (spawnedVehicles.ContainsKey(data.id)) return;

            GameObject go = Instantiate(vehiclePrefab);
            go.name = $"Vehicle_{data.id}";

            ApplyPose(go.transform, data);

            spawnedVehicles[data.id] = go;
        }

        private void HandleUpdate(TrafficReceiver.VehicleData data)
        {
            if (!spawnedVehicles.TryGetValue(data.id, out GameObject go)) return;

            ApplyPose(go.transform, data);
        }

        private void HandleDespawn(TrafficReceiver.VehicleData data)
        {
            if (!spawnedVehicles.TryGetValue(data.id, out GameObject go)) return;

            Destroy(go);
            spawnedVehicles.Remove(data.id);
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
