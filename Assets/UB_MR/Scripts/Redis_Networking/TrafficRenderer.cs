using System.Collections.Generic;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    public class TrafficRenderer : MonoBehaviour
    {
        [SerializeField] private TrafficReceiver receiver;
        [SerializeField] private GameObject vehiclePrefab;
        [SerializeField] private Vector3 originOffset = new Vector3(1.347f, 0f, 5.916f);

        private Dictionary<string, GameObject> spawnedVehicles = new();

        void OnEnable()
        {
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

            go.transform.position = data.Position() + originOffset;
            go.transform.rotation = data.Orientation();

            spawnedVehicles[data.id] = go;
        }

        private void HandleUpdate(TrafficReceiver.VehicleData data)
        {
            Debug.Log($"Vehicle {data.id} → {data.Position()}");
            Debug.Log($"SPAWN POINT: {data.location.x}, {data.location.y}, {data.location.z}");
        
            if (!spawnedVehicles.TryGetValue(data.id, out GameObject go)) return;

            go.transform.position = data.Position() + originOffset;
            go.transform.rotation = data.Orientation();
        }

        private void HandleDespawn(TrafficReceiver.VehicleData data)
        {
            if (!spawnedVehicles.TryGetValue(data.id, out GameObject go)) return;

            Destroy(go);
            spawnedVehicles.Remove(data.id);
        }
    }
}