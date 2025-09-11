using System;
using System.Collections;
using UnityEngine;
using System.Collections.Generic;

namespace UB_MR.Redis_Networking
{
    public class TrafficRenderer : MonoBehaviour
    {
        [SerializeField] TrafficReceiver receiver;
        [SerializeField] GameObject defaultTrafficAgent;
        public Dictionary<string, GameObject> trafficAgents;
        Queue<TrafficReceiver.VehicleData> UPDATE_Q;
        Queue<TrafficReceiver.VehicleData> CREATE_Q;
        Queue<TrafficReceiver.VehicleData> REMOVAL_Q;

        void Start()
        {
            if (receiver != null)
            {
                receiver.OnSpawnNewVehicle += TrafficReceiver_OnSpawnNewVehicle;
                receiver.OnVehicleUpdate  += TrafficReceiver_OnVehicleUpdate;
                receiver.OnDespawnVehicle += TrafficReceiver_OnDespawnVehicle;
                
                trafficAgents = new Dictionary<string, GameObject>();
                UPDATE_Q = new Queue<TrafficReceiver.VehicleData>();
                CREATE_Q = new Queue<TrafficReceiver.VehicleData>();
                REMOVAL_Q = new Queue<TrafficReceiver.VehicleData>();
            }
        }

        void OnDestroy()
        {
            if (receiver != null)
            {
                receiver.OnSpawnNewVehicle -= TrafficReceiver_OnSpawnNewVehicle;
                receiver.OnVehicleUpdate -= TrafficReceiver_OnVehicleUpdate;
                receiver.OnDespawnVehicle -= TrafficReceiver_OnDespawnVehicle;
            }
        }
        
        private void Update()
        {
            if (trafficAgents == null)
                return;
            
            // Creation Queue
            while (CREATE_Q.TryDequeue(out TrafficReceiver.VehicleData creationBP))
            { 
                Vector3 position = new Vector3(creationBP.location.x, creationBP.location.z, creationBP.location.y);
                Quaternion orientation = Quaternion.identity;
                GameObject trafficAgent = GameObject.Instantiate(defaultTrafficAgent, position, orientation);
                trafficAgents.Add(creationBP.id, trafficAgent);
                trafficAgent.transform.SetPositionAndRotation(position, orientation);
            }
            
            // Removal Queue
            while (REMOVAL_Q.TryDequeue(out TrafficReceiver.VehicleData removalBP))
            {
                trafficAgents.Remove(removalBP.id);
                Destroy(trafficAgents[removalBP.id].gameObject);
            }
            
            // Update Queue
            while (UPDATE_Q.TryDequeue(out TrafficReceiver.VehicleData updateBP))
            {
                if (!trafficAgents.ContainsKey(updateBP.id)) return; // leftover (already destroyed)
                
                GameObject agent = trafficAgents[updateBP.id];
                // TODO: Convert from Unreal to Unity Coordinate system and set yaw
                Vector3 position = new Vector3(updateBP.location.x, updateBP.location.z, updateBP.location.y);
                Quaternion orientation = Quaternion.identity;
                agent.transform.SetPositionAndRotation(position, orientation);
            }
        }

        private void TrafficReceiver_OnSpawnNewVehicle(TrafficReceiver.VehicleData inData)
        {
            CREATE_Q.Enqueue(inData);
        }

        private void TrafficReceiver_OnVehicleUpdate(TrafficReceiver.VehicleData inData)
        {
            UPDATE_Q.Enqueue(inData);
        }

        private void TrafficReceiver_OnDespawnVehicle(TrafficReceiver.VehicleData inData)
        {
            REMOVAL_Q.Enqueue(inData);
        }
        
        
    }
}

