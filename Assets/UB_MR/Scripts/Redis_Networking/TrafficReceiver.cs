using System;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using Newtonsoft.Json;
using System.Text;

namespace UB_MR.Redis_Networking
{
    public class TrafficReceiver : MonoBehaviour
    {
        //Agent events
        public Action<VehicleData> OnSpawnNewVehicle;
        public Action<VehicleData> OnDespawnVehicle;
        public Action<VehicleData> OnVehicleUpdate;

        //Traffic actor events
        public Action<TrafficActorData> OnSpawnTrafficActor;
        public Action<TrafficActorData> OnDespawnTrafficActor;
        public Action<TrafficActorData> OnTrafficActorUpdate;

        [Serializable]
        public class Location
        {
            public float x;
            public float y;
            public float z;
        }

        [Serializable]
        public class Rotation
        {
            public float roll;
            public float pitch;
            public float yaw;
        }

        [Serializable]
        public class VehicleData
        {
            public string id;
            public Location location;
            public float yaw;
            public string blueprint;
            public string color;

            public Vector3 Position()
            {
                return ConvertPositionUEToUnity(
                    new Vector3(location.x, location.y, location.z),
                    100
                );
            }

            public Quaternion Orientation()
            {
                return Quaternion.Euler(
                    ConvertRotationUEToUnity(new Vector3(0, yaw, 0))
                );
            }
        }

        [Serializable]
        public class TrafficActorData
        {
            public int actor_id;
            public string actor_type;
            public string type_id;
            public Location location;
            public Rotation rotation;
            public Location extent;
            public Location velocity;

            public string Id => actor_id.ToString();

            public Vector3 Position()
            {
                return ConvertPositionUEToUnity(
                    new Vector3(location.x, location.y, location.z),
                    100
                );
            }

            public Quaternion Orientation()
            {
                return Quaternion.Euler(
                    ConvertRotationUEToUnity(
                        new Vector3(rotation.pitch, rotation.yaw, rotation.roll)
                    )
                );
            }
        }

        [Serializable]
        public class TrafficPayload
        {
            public Dictionary<string, VehicleData> vehicles;
            public List<TrafficActorData> traffic;
        }

        public static Vector3 ConvertPositionUEToUnity(Vector3 ue, float scale)
        {
            return new Vector3(
                ue.y * scale / 100f,
                ue.z * scale / 100f,
                ue.x * scale / 100f
            );
        }

        public static Vector3 ConvertRotationUEToUnity(Vector3 ue)
        {
            return new Vector3(-ue.x, -ue.y, ue.z);
        }

        private UdpClient udpClient;
        private Thread receiveThread;
        private bool isReceiving;

        public int port = 12345;

        private readonly object _lock = new object();

        private Dictionary<string, VehicleData> agents = new();
        private Dictionary<string, TrafficActorData> trafficActors = new();

        void Start()
        {
            StartReceiving();
        }

        void OnDestroy()
        {
            isReceiving = false;
            udpClient?.Close();
            receiveThread?.Join(500);
        }

        void StartReceiving()
        {
            udpClient = new UdpClient(port);
            isReceiving = true;

            receiveThread = new Thread(ReceiveLoop);
            receiveThread.Start();

            Debug.Log($"[TrafficReceiver] Listening on UDP {port}");
        }

        void ReceiveLoop()
        {
            IPEndPoint ep = new IPEndPoint(IPAddress.Any, 0);

            while (isReceiving)
            {
                try
                {
                    byte[] data = udpClient.Receive(ref ep);
                    string json = Encoding.UTF8.GetString(data);

                    TrafficPayload payload =
                        JsonConvert.DeserializeObject<TrafficPayload>(json);

                    lock (_lock)
                    {
                        ProcessAgents(payload.vehicles);
                        ProcessTraffic(payload.traffic);
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"[TrafficReceiver] {e.Message}");
                }
            }
        }

        void ProcessAgents(Dictionary<string, VehicleData> incoming)
        {
            foreach (var kv in incoming)
            {
                kv.Value.id = kv.Key;

                if (agents.ContainsKey(kv.Key))
                {
                    agents[kv.Key] = kv.Value;
                    OnVehicleUpdate?.Invoke(kv.Value);
                }
                else
                {
                    agents.Add(kv.Key, kv.Value);
                    OnSpawnNewVehicle?.Invoke(kv.Value);
                }
            }

            List<string> toRemove = new();
            foreach (var id in agents.Keys)
                if (!incoming.ContainsKey(id))
                    toRemove.Add(id);

            foreach (var id in toRemove)
            {
                OnDespawnVehicle?.Invoke(agents[id]);
                agents.Remove(id);
            }
        }

        void ProcessTraffic(List<TrafficActorData> incoming)
        {
            HashSet<string> seen = new();

            foreach (var actor in incoming)
            {
                string id = actor.Id;
                seen.Add(id);

                if (trafficActors.ContainsKey(id))
                {
                    trafficActors[id] = actor;
                    OnTrafficActorUpdate?.Invoke(actor);
                }
                else
                {
                    trafficActors.Add(id, actor);
                    OnSpawnTrafficActor?.Invoke(actor);
                }
            }

            List<string> toRemove = new();
            foreach (var id in trafficActors.Keys)
                if (!seen.Contains(id))
                    toRemove.Add(id);

            foreach (var id in toRemove)
            {
                OnDespawnTrafficActor?.Invoke(trafficActors[id]);
                trafficActors.Remove(id);
            }
        }
    }
}