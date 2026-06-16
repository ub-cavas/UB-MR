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
            public double? server_timestamp;
            public int? server_frame;
        }

        public struct TrafficDiagnosticsSnapshot
        {
            public TrafficDiagnosticsSnapshot(
                int listenPort,
                string remoteEndpoint,
                long totalPackets,
                long totalBytes,
                float packetsPerSecond,
                float bytesPerSecond,
                int vehicleCount,
                float lastPacketAgeSeconds,
                int malformedPackets,
                int missedFrames,
                int duplicateFrames,
                float jitterSeconds,
                int? lastServerFrame)
            {
                ListenPort = listenPort;
                RemoteEndpoint = remoteEndpoint;
                TotalPackets = totalPackets;
                TotalBytes = totalBytes;
                PacketsPerSecond = packetsPerSecond;
                BytesPerSecond = bytesPerSecond;
                VehicleCount = vehicleCount;
                LastPacketAgeSeconds = lastPacketAgeSeconds;
                MalformedPackets = malformedPackets;
                MissedFrames = missedFrames;
                DuplicateFrames = duplicateFrames;
                JitterSeconds = jitterSeconds;
                LastServerFrame = lastServerFrame;
            }

            public int ListenPort { get; }
            public string RemoteEndpoint { get; }
            public long TotalPackets { get; }
            public long TotalBytes { get; }
            public float PacketsPerSecond { get; }
            public float BytesPerSecond { get; }
            public int VehicleCount { get; }
            public float LastPacketAgeSeconds { get; }
            public int MalformedPackets { get; }
            public int MissedFrames { get; }
            public int DuplicateFrames { get; }
            public float JitterSeconds { get; }
            public int? LastServerFrame { get; }
            public bool HasTraffic => TotalPackets > 0;
        }

        [Header("Network")]
        [SerializeField] private int listenPort = 12345;
        [SerializeField] private bool logPackets = false;

        private class ReceivedTrafficPacket
        {
            public TrafficPayload Payload;
            public int ByteCount;
            public string RemoteEndpoint;
            public float ReceivedAtRealtime;
        }

        private readonly ConcurrentQueue<ReceivedTrafficPacket> _incomingPayloads = new();

        private readonly Dictionary<string, VehicleData> _knownVehicles = new();

        private UdpClient _udpClient;
        private Thread _receiveThread;
        private volatile bool _isReceiving;
        private int _malformedPacketCount;
        private long _totalPackets;
        private long _totalBytes;
        private int _windowPackets;
        private long _windowBytes;
        private float _windowStartRealtime;
        private float _packetsPerSecond;
        private float _bytesPerSecond;
        private float _lastPacketRealtime = -1f;
        private float _previousPacketRealtime = -1f;
        private string _lastRemoteEndpoint = "none";
        private int? _lastServerFrame;
        private double? _lastServerTimestamp;
        private int _missedFrames;
        private int _duplicateFrames;
        private float _jitterSeconds;
        private readonly System.Diagnostics.Stopwatch _diagnosticsClock = System.Diagnostics.Stopwatch.StartNew();

        void Start()
        {
            _udpClient = new UdpClient(listenPort);
            _windowStartRealtime = DiagnosticsNowSeconds;

            Debug.Log($"Listening on UDP port {listenPort}");
            Debug.Log($"Socket bound to: {_udpClient.Client.LocalEndPoint}");

            _isReceiving = true;
            _receiveThread = new Thread(ReceiveLoop) { IsBackground = true, Name = "TrafficReceiver" };
            _receiveThread.Start();
            Debug.Log($"Listening on UDP port {listenPort}");
        }

        void Update()
        {
            while (_incomingPayloads.TryDequeue(out ReceivedTrafficPacket packet))
            {
                RecordPacket(packet);
                ProcessPayload(packet.Payload);
            }

            UpdateRates(DiagnosticsNowSeconds);
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

                    if (logPackets)
                        Debug.Log($"[TrafficReceiver] Packet received from {endpoint.Address}:{endpoint.Port} | Size: {data.Length} bytes");

                    TrafficPayload payload = JsonConvert.DeserializeObject<TrafficPayload>(json);
                    if (payload?.vehicles != null)
                    {
                        _incomingPayloads.Enqueue(new ReceivedTrafficPacket
                        {
                            Payload = payload,
                            ByteCount = data.Length,
                            RemoteEndpoint = $"{endpoint.Address}:{endpoint.Port}",
                            ReceivedAtRealtime = DiagnosticsNowSeconds
                        });
                    }
                    else
                    {
                        Interlocked.Increment(ref _malformedPacketCount);
                    }
                }
                catch (SocketException)
                {
                    break;
                }
                catch (Exception e)
                {
                    Interlocked.Increment(ref _malformedPacketCount);
                    Debug.LogError($"[TrafficReceiver] Receive error: {e.Message}");
                }
            }
        }

        private void RecordPacket(ReceivedTrafficPacket packet)
        {
            _totalPackets++;
            _totalBytes += packet.ByteCount;
            _windowPackets++;
            _windowBytes += packet.ByteCount;
            _lastRemoteEndpoint = packet.RemoteEndpoint;
            _previousPacketRealtime = _lastPacketRealtime;
            _lastPacketRealtime = packet.ReceivedAtRealtime;

            RecordFrameDiagnostics(packet.Payload, packet.ReceivedAtRealtime);
            UpdateRates(DiagnosticsNowSeconds);
        }

        private void RecordFrameDiagnostics(TrafficPayload payload, float receivedAtRealtime)
        {
            if (payload.server_frame.HasValue)
            {
                int serverFrame = payload.server_frame.Value;
                if (_lastServerFrame.HasValue)
                {
                    int frameDelta = serverFrame - _lastServerFrame.Value;
                    if (frameDelta == 0)
                        _duplicateFrames++;
                    else if (frameDelta > 1)
                        _missedFrames += frameDelta - 1;
                }
                _lastServerFrame = serverFrame;
            }

            if (payload.server_timestamp.HasValue && _lastServerTimestamp.HasValue && _previousPacketRealtime >= 0f)
            {
                double serverDelta = payload.server_timestamp.Value - _lastServerTimestamp.Value;
                float receiveDelta = receivedAtRealtime - _previousPacketRealtime;
                float sampleJitter = Mathf.Abs(receiveDelta - (float)serverDelta);
                _jitterSeconds += (sampleJitter - _jitterSeconds) / 16f;
            }

            if (payload.server_timestamp.HasValue)
                _lastServerTimestamp = payload.server_timestamp.Value;
        }

        private void UpdateRates(float now)
        {
            float elapsed = now - _windowStartRealtime;
            if (elapsed < 1f)
                return;

            _packetsPerSecond = _windowPackets / elapsed;
            _bytesPerSecond = _windowBytes / elapsed;
            _windowPackets = 0;
            _windowBytes = 0;
            _windowStartRealtime = now;
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

        public TrafficDiagnosticsSnapshot GetDiagnosticsSnapshot()
        {
            float lastPacketAgeSeconds = _lastPacketRealtime >= 0f
                ? DiagnosticsNowSeconds - _lastPacketRealtime
                : -1f;

            return new TrafficDiagnosticsSnapshot(
                listenPort,
                _lastRemoteEndpoint,
                _totalPackets,
                _totalBytes,
                _packetsPerSecond,
                _bytesPerSecond,
                _knownVehicles.Count,
                lastPacketAgeSeconds,
                _malformedPacketCount,
                _missedFrames,
                _duplicateFrames,
                _jitterSeconds,
                _lastServerFrame
            );
        }

        private float DiagnosticsNowSeconds => (float)_diagnosticsClock.Elapsed.TotalSeconds;
    }
}
