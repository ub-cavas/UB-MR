using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

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

        [Header("UDP fallback (scene-only launches)")]
        [SerializeField] private int listenPort = 12345;

        private readonly ConcurrentQueue<string> _incomingPayloads = new();

        private readonly Dictionary<string, VehicleData> _knownVehicles = new();

        private readonly Dictionary<string, float> _egoArrivals = new();
        private float _lastTraffic = float.NegativeInfinity;
        private ServerConnection serverConnection;
        private UdpClient _udpClient;
        private Thread _receiveThread;
        private volatile bool _isReceiving;

        void Start()
        {
            serverConnection = ServerConnection.Instance;
            if (serverConnection != null)
            {
                serverConnection.TrafficReceived += ReceiveServerTraffic;
                serverConnection.EgoReceived += ReceiveServerTraffic;
                serverConnection.ParticipantsReset += ResetParticipants;
                return;
            }
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
            while (_incomingPayloads.TryDequeue(out string json)) ReceiveServerTraffic(json);
            float now = Time.realtimeSinceStartup;
            foreach (string id in new List<string>(_egoArrivals.Keys))
                if (now - _egoArrivals[id] > 2f || id == "ego:" + EgoPublisher.LocalEgoId) RemoveVehicle(id);
            if (now - _lastTraffic > 2f) ProcessPayload(new TrafficPayload { vehicles = new List<VehicleData>() });
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool Valid(VehicleData vehicle) => vehicle != null && !string.IsNullOrWhiteSpace(vehicle.id) &&
            vehicle.location != null && Finite(vehicle.location.x) && Finite(vehicle.location.y) &&
            Finite(vehicle.location.z) && Finite(vehicle.yaw);

        private void ReceiveServerTraffic(string json)
        {
            try
            {
                var packet = JObject.Parse(json);
                int? type = (int?)packet["type"];
                if (type == 3)
                {
                    if (packet["ego"] is not JObject ego || ego["id"] == null ||
                        (ego["id"].Type != JTokenType.String && ego["id"].Type != JTokenType.Integer) ||
                        ego["location"] is not JObject location || location["x"] == null ||
                        location["y"] == null || location["z"] == null || ego["yaw"] == null) return;
                    var vehicle = ego.ToObject<VehicleData>();
                    if (!Valid(vehicle)) return;
                    string sourceId = vehicle.id;
                    vehicle.id = "ego:" + sourceId;
                    if (sourceId == EgoPublisher.LocalEgoId) { RemoveVehicle(vehicle.id); return; }
                    if (string.IsNullOrEmpty(vehicle.blueprint)) vehicle.blueprint = "vehicle.lincoln.mkz_2017";
                    _egoArrivals[vehicle.id] = Time.realtimeSinceStartup;
                    Upsert(vehicle);
                }
                else if ((type == null || type == 2) && packet["vehicles"] is JArray)
                {
                    var payload = new TrafficPayload { vehicles = new List<VehicleData>() };
                    foreach (JToken entry in (JArray)packet["vehicles"])
                    {
                        if (entry is not JObject vehicle || vehicle["id"] == null ||
                            (vehicle["id"].Type != JTokenType.String && vehicle["id"].Type != JTokenType.Integer) ||
                            vehicle["location"] is not JObject position || position["x"] == null ||
                            position["y"] == null || position["z"] == null || vehicle["yaw"] == null) continue;
                        var parsed = vehicle.ToObject<VehicleData>();
                        if (Valid(parsed)) payload.vehicles.Add(parsed);
                    }
                    ProcessPayload(payload);
                    _lastTraffic = Time.realtimeSinceStartup;
                }
            }
            catch (Exception error) when (error is JsonException || error is FormatException || error is InvalidCastException || error is OverflowException) { }
        }

        private void Upsert(VehicleData vehicle)
        {
            bool exists = _knownVehicles.ContainsKey(vehicle.id);
            _knownVehicles[vehicle.id] = vehicle;
            if (exists) OnVehicleUpdate?.Invoke(vehicle); else OnSpawnVehicle?.Invoke(vehicle);
        }

        private void RemoveVehicle(string id)
        {
            if (_knownVehicles.TryGetValue(id, out var vehicle)) OnDespawnVehicle?.Invoke(vehicle);
            _knownVehicles.Remove(id);
            _egoArrivals.Remove(id);
        }

        private void ResetParticipants()
        {
            while (_incomingPayloads.TryDequeue(out _)) { }
            foreach (string id in new List<string>(_knownVehicles.Keys)) RemoveVehicle(id);
            _lastTraffic = float.NegativeInfinity;
        }

        void OnDestroy()
        {
            if (serverConnection != null)
            {
                serverConnection.TrafficReceived -= ReceiveServerTraffic;
                serverConnection.EgoReceived -= ReceiveServerTraffic;
                serverConnection.ParticipantsReset -= ResetParticipants;
            }
            ResetParticipants();
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

                    _incomingPayloads.Enqueue(json);
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
                if (!Valid(vehicle) || vehicle.id.StartsWith("ego:", StringComparison.Ordinal)) continue;
                seen.Add(vehicle.id);
                Upsert(vehicle);
            }
            foreach (string id in new List<string>(_knownVehicles.Keys))
                if (!_egoArrivals.ContainsKey(id) && !seen.Contains(id)) RemoveVehicle(id);
        }

        public IReadOnlyDictionary<string, VehicleData> KnownVehicles => _knownVehicles;
    }
}
