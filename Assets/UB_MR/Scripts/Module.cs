using UnityEngine;
using CAVAS.UB_MR.Config;
using CAVAS.UB_MR.DT.Vehicle;
using ROS2;
using robot_localization.srv;
using CAVAS.UB_MR.ROS2;
using System.Threading.Tasks;

namespace CAVAS.UB_MR
{
    public class Module : MonoBehaviour
    {
        [SerializeField] double origin_latitude = 42.9899575863; 
        [SerializeField] double origin_longitude = -78.7980738989; 
        [SerializeField] double origin_altitude = 0;
        string agentPath = "Prefabs/Agent";
        ROS2Node mNode;

        void Start()
        {
            SpawnActiveAgent();
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Unity_Map");
                SetDatum();
            }
        }

        void SetDatum()
        {
            IClient<SetDatum_Request, SetDatum_Response> setDatumClient = this.mNode.CreateClient<SetDatum_Request, SetDatum_Response>("/datum");
            SetDatum_Request request = new SetDatum_Request();
            request.Geo_pose.Position.Latitude = origin_latitude;
            request.Geo_pose.Position.Longitude = origin_longitude;
            request.Geo_pose.Position.Altitude = origin_altitude;
            request.Geo_pose.Orientation.X = 0;
            request.Geo_pose.Orientation.Y = 0;
            request.Geo_pose.Orientation.Z = 0;
            request.Geo_pose.Orientation.W = 1;

            Task<SetDatum_Response> asyncTask = setDatumClient.CallAsync(request);
            asyncTask.ContinueWith(task =>
            {
                if (task.IsCompletedSuccessfully)
                {
                    SetDatum_Response response = task.Result;
                    Debug.Log("Datum set successfully!");
                }
                else
                {
                    Debug.LogWarning("Failed to set datum!");
                }
            });
        }

        DT.Agent SpawnActiveAgent()
        {
            return SpawnAgent(ConfigurationManager.GetConfiguration().Item1);
        }

        DT.Agent SpawnAgent(Config.Agent inAgent)
        {
            DT.Agent agent;
            GameObject newAgent = Instantiate(Resources.Load<GameObject>(agentPath), new Vector3(0,0,0), Quaternion.identity);
            if (inAgent.type == AgentType.AutonomousVehicle)
                agent = newAgent.AddComponent<AutonomousVehicle>();
            else
                agent = newAgent.AddComponent<DT.Agent>();

            agent.Setup(inAgent);
            return agent;
        }
    }
}
