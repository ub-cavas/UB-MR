using System;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEngine;

namespace UB_MR.Redis_Networking
{
    // Created by the Main Menu, persists across map changes. Scene-only launches retain UDP support.
    public sealed class ServerConnection : MonoBehaviour
    {
        public static ServerConnection Instance { get; private set; }
        public ServerSettings Settings { get; private set; }
        public string Status => session?.Status ?? "Disconnected";
        public bool IsConnected => session != null && session.Connected;
        public bool HasTraffic => IsConnected && Time.realtimeSinceStartup - lastTrafficTime < 2f;
        public int VehicleCount { get; private set; }
        public event Action<string> TrafficReceived;
        RedisSession session;
        float lastTrafficTime = float.NegativeInfinity;
        bool hadTraffic;

        public static ServerConnection GetOrCreate()
        {
            if (Instance == null) new GameObject("Server Connection").AddComponent<ServerConnection>();
            return Instance;
        }

        void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(gameObject); return; }
            Instance = this;
            Settings = ServerSettings.Load();
            DontDestroyOnLoad(gameObject);
        }

        public bool Connect(ServerSettings settings, out string error)
        {
            if (!settings.TryValidate(out error)) return false;
            Disconnect();
            Settings = settings.Copy();
            Settings.Save();
            session = new RedisSession(Settings);
            return true;
        }

        public void Disconnect()
        {
            session?.Dispose();
            session = null;
            lastTrafficTime = float.NegativeInfinity;
            VehicleCount = 0;
            hadTraffic = false;
            TrafficReceived?.Invoke("{\"vehicles\":[]}");
        }

        void Update()
        {
            string json = session?.TakeTraffic();
            if (json != null && IsConnected)
            {
                try
                {
                    var packet = JObject.Parse(json);
                    VehicleCount = ((JArray)packet["vehicles"]).Count;
                    lastTrafficTime = Time.realtimeSinceStartup;
                    hadTraffic = true;
                    TrafficReceived?.Invoke(json);
                }
                catch (JsonException) { }
            }
            if (hadTraffic && !HasTraffic)
            {
                VehicleCount = 0;
                hadTraffic = false;
                TrafficReceived?.Invoke("{\"vehicles\":[]}");
            }
        }

        public void PublishEgo(object ego)
        {
            if (!IsConnected) return;
            session.Publish(JsonConvert.SerializeObject(new
            {
                id = "ub-mr", type = 3,
                timestamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0,
                ego
            }));
        }

        void OnApplicationQuit() => Disconnect();
        void OnDestroy()
        {
            if (Instance != this) return;
            Disconnect();
            Instance = null;
        }
    }
}
