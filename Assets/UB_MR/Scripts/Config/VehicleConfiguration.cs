using System.Collections.Generic;
using UnityEngine;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Converters;
using System;

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
    public class Agent
    {
        public string name;
        public Dictionary<string, Sensor> sensors = new Dictionary<string, Sensor>();
    }

    [System.Serializable]
    public class Sensor
    {
        public string name;
        public string topic;
        public SensorType type;
        public Vector3 position;
        public Vector3 rotation;
    }

    public static class ConfigurationManager
    {
        static Agent activeAgent;

        public static void SaveToJSON(Agent data)
        {
            string filename = "agent-" + data.name + ".json";
            string path = Path.Combine(Application.persistentDataPath, filename);

            string json = JsonConvert.SerializeObject(data, Formatting.Indented,
                new JsonSerializerSettings
                {
                    ReferenceLoopHandling = ReferenceLoopHandling.Ignore,
                    ContractResolver = new Newtonsoft.Json.Serialization.DefaultContractResolver
                    {
                        IgnoreSerializableAttribute = true
                    }
                });

            File.WriteAllText(path, json);
            Debug.Log($"Saved agent config to {path}");
        }

        // Accepts either JSON file name or agent name
        public static Agent LoadFromJSON(string inAgentName)
        {
            string filename;
            if (inAgentName.Contains(".json"))
                filename = inAgentName;
            else
                filename = "agent-" + inAgentName + ".json";

            string path = Path.Combine(Application.persistentDataPath, filename);
            if (File.Exists(path))
            {
                string json = File.ReadAllText(path);
                return JsonConvert.DeserializeObject<Agent>(json);
            }
            else
            {
                Debug.LogWarning(filename + " not found");
                return null;
            }
        }
        
        public static List<Agent> GetAllAgents()
        {
            List<Agent> agents = new List<Agent>();
            string[] files = Directory.GetFiles(Application.persistentDataPath);
            foreach (string file in files)
            {
                Agent agent = LoadFromJSON(file);
                if (agent is not null)
                    agents.Add(agent);
            }
                
            return agents;
        }

        public static void SetActiveAgent(Agent agent)
        {
            activeAgent = agent;
        }
    }
}