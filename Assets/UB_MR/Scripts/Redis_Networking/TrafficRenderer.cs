using System.Collections.Generic;
using CAVAS.UB_MR;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    public class TrafficRenderer : MonoBehaviour
    {
        [SerializeField] private TrafficReceiver receiver;
        [SerializeField] private GameObject vehiclePrefab;
        [SerializeField] private Transform mapRoot;
        [SerializeField] private bool applyMapTransform = true;
        [SerializeField] private Vector3 originOffset = new Vector3(1.347f, 0f, 5.916f);

        private Dictionary<string, GameObject> spawnedVehicles = new();

        void Awake()
        {
            ResolveMapRoot();
        }

        void OnEnable()
        {
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

            if (applyMapTransform && TryGetMapRoot(out Transform root))
            {
                if (vehicleTransform.parent != root)
                    vehicleTransform.SetParent(root, false);

                vehicleTransform.localPosition = mapLocalPosition;
                vehicleTransform.localRotation = mapLocalRotation;
                return;
            }

            vehicleTransform.SetPositionAndRotation(mapLocalPosition, mapLocalRotation);
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

            Module module = FindFirstObjectByType<Module>();
            if (module != null)
                mapRoot = module.MapRoot;

            return mapRoot;
        }
    }
}
