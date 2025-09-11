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
        public System.Action<VehicleData> OnSpawnNewVehicle;
        public System.Action<VehicleData> OnDespawnVehicle;
        public System.Action<VehicleData> OnVehicleUpdate;
        
        [System.Serializable]
        public class Location
        {
            public float x;
            public float y;
            public float z;
        }

        [System.Serializable]
        public class VehicleData
        {
            public string id;
            public Location location; // Don't use this... use Position()
            public float yaw; // Don;t use this... Use Orientation()
            public string blueprint;
            public string color;

            public Vector3 Position()
            {
                return ConvertPositionUEToUnity(new Vector3(location.x, location.y, location.z), 100);
            }

            public Quaternion Orientation()
            {
                return Quaternion.Euler(ConvertRotationUEToUnity(new Vector3(0, 0, yaw)));
            }
        }
        
        public static Vector3 ConvertPositionUEToUnity(Vector3 uePosition, float scale = 1)
        {
            return new Vector3(
                uePosition.y * scale / 100f,   // UE Y → Unity X
                uePosition.z * scale / 100f,   // UE Z → Unity Y
                uePosition.x * scale / 100f    // UE X → Unity Z
            );
        }

        public static Vector3 ConvertRotationUEToUnity(Vector3 ueRotation)
        {
            return new Vector3(
                -ueRotation.x,  // -Pitch
                -ueRotation.y,  // -Yaw
                ueRotation.z    // Roll
            );
        }
        
        
        private UdpClient udpClient;
        private Thread receiveThread;
        private bool isReceiving = false;
        public int port = 12345;
        // Latest received data
        public static Dictionary<string, VehicleData> trafficData;
        public static bool hasNewData = false;
        
        void Start()
        {
            trafficData = new Dictionary<string, VehicleData>();
            StartReceiving();
        }
        
        void Update()
        {
            // Process new data on main thread
            if (hasNewData)
            {
                lock (this)
                {
                    if (trafficData != null)
                    {
                        ProcessTrafficData();
                        hasNewData = false;
                    }
                }
            }
        }
        
        void OnDestroy()
        {
            isReceiving = false;
            if (receiveThread != null && receiveThread.IsAlive)
                receiveThread.Join(1000);
            if (udpClient != null)
                udpClient.Close();
        }
        
        void StartReceiving()
        {
            try
            {
                udpClient = new UdpClient(port);
                isReceiving = true;
            
                receiveThread = new Thread(ReceiveData);
                receiveThread.Start();
            
                Debug.Log($"Started listening on port {port}");
            }
            catch (Exception e)
            {
                Debug.LogError($"Error starting UDP receiver: {e.Message}");
            }
        }
        
        void ReceiveData()
        {
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        
            while (isReceiving)
            {
                try
                {
                    byte[] data = udpClient.Receive(ref remoteEndPoint);
                    string jsonString = Encoding.UTF8.GetString(data);
                
                    // Parse JSON
                    Dictionary<string, VehicleData> receivedDataDict = 
                        JsonConvert.DeserializeObject<Dictionary<string, VehicleData>>(jsonString);
                    // Thread-safe update
                    lock (this)
                    {
                        foreach (string vehicleID in receivedDataDict.Keys)
                        {
                            receivedDataDict[vehicleID].id = vehicleID;
                            //Update Existing Vehicles
                            if (trafficData.ContainsKey(vehicleID))
                            {
                                trafficData[vehicleID].location = receivedDataDict[vehicleID].location; 
                                trafficData[vehicleID].yaw = receivedDataDict[vehicleID].yaw;
                                OnVehicleUpdate?.Invoke(receivedDataDict[vehicleID]); // Let the listeners know
                            }
                            // Add New Vehicles
                            else
                            {
                                trafficData.Add(vehicleID, receivedDataDict[vehicleID]);
                                OnSpawnNewVehicle?.Invoke(receivedDataDict[vehicleID]);// Let the listeners know
                            }
                                
                        }
                        // Remove deleted vehicles
                        if (receivedDataDict.Count != trafficData.Keys.Count)
                            foreach (string id in trafficData.Keys)
                                if (!receivedDataDict.ContainsKey(id))
                                {
                                    trafficData.Remove(id);
                                    OnDespawnVehicle?.Invoke(trafficData[id]); // Let the listeners know
                                }
                        hasNewData = true;
                    }
                }
                catch (Exception e)
                {
                    Debug.LogError($"Error receiving data: {e.Message}");
                }
            }
        }
        
        void ProcessTrafficData()
        {
            //Debug.Log("Traffic Agents: " + trafficData.Count);
            foreach (KeyValuePair<string, VehicleData> vehicle in trafficData)
            {
                // Use your traffic data here
                Vector3 position = new Vector3(vehicle.Value.location.x, vehicle.Value.location.y, vehicle.Value.location.z);
                float yaw = vehicle.Value.yaw;
                Debug.Log($"Received traffic at position: {position}, yaw: {yaw}, blueprint: {vehicle.Value.blueprint}");
            }
        }

    }

}