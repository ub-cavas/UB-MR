using System.Collections.Generic;
using UnityEngine;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

namespace CAVAS.UB_MR.Config
{
    [System.Serializable]
    [JsonConverter(typeof(StringEnumConverter))] // Makes JSON readable ("LiDAR" instead of 0)
    public enum SensorType
    {
        LiDAR,
        Camera,
        GNSS,
        IMU
    }

    [System.Serializable]
    public class Vehicle
    {
        public string vehicle_name;
        public Dictionary<string, Sensor> sensors = new Dictionary<string, Sensor>();
    }

    [System.Serializable]
    public class Sensor
    {
        public SensorType type;
        public Vector3 position;
        public Vector3 rotation;
    }

    public static class ConfigManager
    {
        private static readonly string path = Path.Combine(Application.persistentDataPath, "vehicles.json");

        public static void Save(Vehicle data)
        {
            // Pretty-print JSON for readability
            string json = JsonConvert.SerializeObject(data, Formatting.Indented);
            File.WriteAllText(path, json);
            Debug.Log($"Saved vehicle config to {path}");
        }

        public static Vehicle Load()
        {
            if (File.Exists(path))
            {
                string json = File.ReadAllText(path);
                return JsonConvert.DeserializeObject<Vehicle>(json);
            }
            else
            {
                Debug.LogWarning("No save.json found, returning empty Vehicle object.");
                return new Vehicle();
            }            
        }
    }
}