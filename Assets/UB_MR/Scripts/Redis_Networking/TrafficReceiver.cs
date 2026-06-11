using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text;
using UnityEngine;
using Newtonsoft.Json;

namespace UB_MR.Redis_Networking
{
    public class TrafficReceiver : MonoBehaviour
    {
        public event Action<VehicleData> OnSpawnVehicle;
        public event Action<VehicleData> OnDespawnVehicle;
        public event Action<VehicleData> OnVehicleUpdate;

        [Serializable]
        public class LocationData
        {
            public float x;
            public float y;
            public float z;
        }

        [Serializable]
        public class VehicleData
        {
            public string id;
            public LocationData location;
            public float yaw;
            public string blueprint;
            public string color;

            public Vector3 Position()
            {
                return new Vector3(
                     location.y,
                     location.z,
                     location.x
                );
            }

            public Quaternion Orientation()
            {
                // CARLA +X forward maps to Unity +Z, and CARLA +Y maps to Unity +X.
                // With that basis, CARLA yaw has the same sign as Unity yaw.
                return Quaternion.Euler(0f, yaw, 0f);
            }
        }

        [Serializable]
        private class TrafficPayload
        {
            public List<VehicleData> vehicles;
            public double timestamp;
        }

        [Header("Network")]
        [SerializeField] private int listenPort = 12345;

        private readonly ConcurrentQueue<TrafficPayload> _incomingPayloads = new();

        private readonly Dictionary<string, VehicleData> _knownVehicles = new();

        private UdpClient _udpClient;
        private Thread _receiveThread;
        private volatile bool _isReceiving;

        void Start()
        {
            _udpClient = new UdpClient(listenPort);

            Debug.Log($"Listening on UDP port {listenPort}");
            Debug.Log($"Socket bound to: {_udpClient.Client.LocalEndPoint}");

            _isReceiving = true;
            _receiveThread = new Thread(ReceiveLoop) { IsBackground = true, Name = "TrafficReceiver" };
            _receiveThread.Start();
            Debug.Log($"Listening on UDP port {listenPort}");
        }

        void Update()
        {
            while (_incomingPayloads.TryDequeue(out TrafficPayload payload))
            {
                ProcessPayload(payload);
            }
        }

        void OnDestroy()
        {
            _isReceiving = false;
            _udpClient?.Close();
            _receiveThread?.Join(500);
        }

        private void ReceiveLoop()
        {
            IPEndPoint endpoint = new IPEndPoint(IPAddress.Any, 0);
            while (_isReceiving)
            {
                try
                {
                    byte[] data = _udpClient.Receive(ref endpoint);
                    string json = Encoding.UTF8.GetString(data);

                    Debug.Log($"[TrafficReceiver] Packet received from {endpoint.Address}:{endpoint.Port} | Size: {data.Length} bytes");

                    TrafficPayload payload = JsonConvert.DeserializeObject<TrafficPayload>(json);
                    if (payload?.vehicles != null)
                        _incomingPayloads.Enqueue(payload);
                }
                catch (SocketException)
                {
                    break;
                }
                catch (Exception e)
                {
                    Debug.LogError($"[TrafficReceiver] Receive error: {e.Message}");
                }
            }
        }

        private void ProcessPayload(TrafficPayload payload)
        {


            HashSet<string> seen = new HashSet<string>();

            foreach (VehicleData vehicle in payload.vehicles)
            {
                if (string.IsNullOrEmpty(vehicle.id)) continue;

                seen.Add(vehicle.id);

                if (_knownVehicles.TryGetValue(vehicle.id, out _))
                {
                    _knownVehicles[vehicle.id] = vehicle;
                    OnVehicleUpdate?.Invoke(vehicle);
                }
                else
                {
                    _knownVehicles[vehicle.id] = vehicle;
                    OnSpawnVehicle?.Invoke(vehicle);
                }
            }

            List<string> toRemove = new List<string>();
            foreach (string id in _knownVehicles.Keys)
            {
                if (!seen.Contains(id))
                    toRemove.Add(id);
            }

            foreach (string id in toRemove)
            {
                OnDespawnVehicle?.Invoke(_knownVehicles[id]);
                _knownVehicles.Remove(id);
            }
        }

        public IReadOnlyDictionary<string, VehicleData> KnownVehicles => _knownVehicles;
    }
}
