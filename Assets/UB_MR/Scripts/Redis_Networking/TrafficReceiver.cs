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
            public Location location;
            public float yaw;
            public string blueprint;
            public string color;
        }
        
        
        
        private UdpClient udpClient;
        private Thread receiveThread;
        private bool isReceiving = false;
        public int port = 12345;
        // Latest received data
        private Dictionary<string, VehicleData> trafficData;
        private bool hasNewData = false;
        
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
                    List<VehicleData> receivedDataList = JsonConvert.DeserializeObject<List<VehicleData>>(jsonString);
                
                    // Thread-safe update
                    lock (this)
                    {
                        // Add New Vehicles + Update Existing Vehicles
                        foreach (VehicleData vehicleData in receivedDataList)
                        {
                            if (trafficData.ContainsKey(vehicleData.id))
                            {
                                trafficData[vehicleData.id].location = vehicleData.location; 
                                trafficData[vehicleData.id].yaw = vehicleData.yaw;
                            }
                            else
                                trafficData.Add(vehicleData.id, vehicleData);
                        }
                        // Remove deleted vehicles
                        if (receivedDataList.Count != trafficData.Keys.Count)
                            foreach (string id in trafficData.Keys)
                                if (trafficData.ContainsKey(id))
                                    trafficData.Remove(id);
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
            Debug.Log("Traffic Agents: " + trafficData.Count);
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