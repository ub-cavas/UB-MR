using UnityEngine;
using CAVAS.UB_MR.Config;
using CAVAS.UB_MR.DT.Vehicle;
using ROS2;
using robot_localization.srv;
using CAVAS.UB_MR.ROS2;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.Collections;

namespace CAVAS.UB_MR
{
    public class Module : MonoBehaviour
    {
        [Header("Map Data")]
        [SerializeField] Transform map_root;
        [SerializeField] double origin_latitude = 42.9926175773; 
        [SerializeField] double origin_longitude = -78.7925575781; 
        [SerializeField] double origin_altitude = 152.5;

        [Space]

        [Header("Scene Data")]
        [SerializeField] List<SDFTexture> mSdfs;
        [SerializeField, Min(0f)] float sdfRefreshIntervalSeconds = 0.25f;

        [Space]

        [Header("UI")]
        [SerializeField] MapPanel mapPanel;


        string agentPath = "Prefabs/Agent";
        ROS2Node mNode;

        protected virtual void Start()
        {
            RefreshSDFList();
            mapPanel.SetMapPosition(map_root.position);
            mapPanel.SetMapRotation(map_root.rotation.eulerAngles);
            StartCoroutine(MapUpdate());
            StartCoroutine(SDFListUpdate());
            
            SpawnActiveAgent();
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode("Unity_Map");
                SetDatum();
            }
        }

        IEnumerator MapUpdate()
        {
            while (true)
            {
                yield return null;
                UpdateMap(mapPanel.GetMapPosition(), mapPanel.GetMapRotation());
            }
            
        }

        IEnumerator SDFListUpdate()
        {
            if (this.sdfRefreshIntervalSeconds <= 0f)
            {
                while (true)
                {
                    RefreshSDFList();
                    yield return null;
                }
            }

            WaitForSeconds refreshDelay = new WaitForSeconds(this.sdfRefreshIntervalSeconds);
            while (true)
            {
                RefreshSDFList();
                yield return refreshDelay;
            }
        }

        void RefreshSDFList()
        {
            this.mSdfs ??= new List<SDFTexture>();
            this.mSdfs.Clear();
            this.mSdfs.AddRange(FindObjectsByType<SDFTexture>(FindObjectsSortMode.None));
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

        public IReadOnlyList<SDFTexture> GetSDFs()
        {
            return this.mSdfs;
        }

        DT.Agent SpawnActiveAgent()
        {
            return SpawnAgent(ConfigurationManager.GetConfiguration().Item1);
        }

        DT.Agent SpawnAgent(Config.Agent inAgent)
        {
            if (inAgent is null)
            {
                Debug.LogWarning("NO AGENT SPAWNED");
                return null;
            }
                

            DT.Agent agent;
            GameObject newAgent = Instantiate(Resources.Load<GameObject>(agentPath), new Vector3(0,0,0), Quaternion.identity);
            if (inAgent.isDynamic)
                agent = newAgent.AddComponent<DynamicAgent>();
            else
                agent = newAgent.AddComponent<DT.Agent>();

            agent.Setup(inAgent, this);
            return agent;
        }
    
        void UpdateMap(Vector3 inPosition, Quaternion inRotation)
        {
            // Set position
            map_root.position = inPosition;
            // Set rotation
            map_root.rotation = inRotation;
        }

    }
}
