using System.Collections.Generic;
using CAVAS.UB_MR.Config;
using Unity.Cinemachine;
using UnityEngine;
using ROS2;
using CAVAS.UB_MR.ROS2;
using System.Collections;
using CAVAS.UB_MR.DT.Sensors;
using CAVAS.UB_MR.DT.Sensors.Lidar;
using CAVAS.UB_MR.DT.Sensors.Camera;
using System;



/*TODO:
    1.) Update database to support CAMERA and LiDAR spec configuration
    2.) Test CameraModification.cs with multiple cameras
    3.) Add Traffic Light Agent Model
    4.) Add empty Agent Model
    5.) Test full stack with KITTI dataset
*/


namespace CAVAS.UB_MR.DT
{
    public class Agent : MonoBehaviour
    {
        Transform baseLink;
        Transform visRoot;
        Transform spectatorCameras;
        List<Tuple<Config.Sensor, SensorModifier, Transform>> sensors = new List<Tuple<Config.Sensor, SensorModifier, Transform>>();
        HUD hud;
        ROS2Node mNode;
        Vector3 mWorldPosition;
        Quaternion mWorldRotation;

        // TODO: Store QOS settings in the sensor
        #region LiDAR
        ComputeShader lidarModifierComputeShader;

        // General Parameters
        int raysPerScan = 60_000;

        // SDF Parameters
        float maxRaytraceDistance = 100.0f;
        float hitThreshold = 0.0001f;
        int maxIterations = 64;

        // QOS
        ReliabilityPolicy reliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT;
        HistoryPolicy historyPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST;
        int historyDepth = 2;
        DurabilityPolicy durabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE;
        #endregion

        #region Camera
        float camera_pub_rate = 24.0f;
        Dictionary<string, CameraModifier> cameraModifiers;
        int imageWidth = 640;
        int imageHeight = 480;
        #endregion

        #region Spectator
        CinemachineCamera[] cinemachineCameras;
        int camIdx = 0;
        #endregion
        
        #region Ground Truth
        float gt_pub_rate = 30.0f;
        string boundingBoxTopicName = "/virtual_obstacles"; // Topic name for publishing virtual object bounding boxes
        float detectionRadius = 1000.0f;
        VirtualBoundingBoxDetector mVirtualBoundingBoxDetector;
        #endregion

        float timer = 0f;

        protected virtual IEnumerator Start()
        {
            yield return new WaitUntil(() => ROS2_Bridge.ROS_CORE.Ok() && this.mNode is not null);
            
            foreach (Tuple<Config.Sensor, SensorModifier, Transform> sensor in sensors)
            {
                switch(sensor.Item1.type)
                {
                    case SensorType.Camera:
                        StartCoroutine(PublishCameraImages()); // TODO: cache a ref to the running coroutine 
                        break;
                    default:
                        break;
                }
            }
            // Publish GT bounding boxes
            StartCoroutine(PublishGroundTruth());
        }

        protected virtual void Update()
        {
            timer += Time.deltaTime;
            
            // LiDAR Modification... try to modify the LiDAR as quickly as possible / no set Hz
            foreach (Tuple<Config.Sensor, SensorModifier, Transform> sensor in sensors)
            {
                switch(sensor.Item1.type)
                {
                    case SensorType.LiDAR:
                        LidarModifier lidarModifier = (LidarModifier)sensor.Item2;
                        if (lidarModifier.TryModify(sensor.Item3))
                            lidarModifier.Publish();
                        break;
                    default:
                        break;
                }
            }
        }

        public virtual void Setup(Config.Agent inAgent, Module inModule)
        {
            // Name
            this.gameObject.name = inAgent.name;
            // Computer Shaders
            this.lidarModifierComputeShader = Resources.Load<ComputeShader>("Scripts/SDFRaymarch");

            baseLink = transform.Find("base_link");

            // ROS2
            ConnectToROS();

            // Sensors
            foreach (Config.Sensor sensor_config in inAgent.sensors.Values)
            {
                GameObject sensorGO = new GameObject(sensor_config.name);
                sensorGO.transform.SetParent(baseLink);
                sensorGO.transform.SetLocalPositionAndRotation(sensor_config.position, Quaternion.Euler(sensor_config.rotation));
                SensorModifier sensor;
                
                switch (sensor_config.type)
                {
                    case SensorType.LiDAR:
                        // TODO: Store QOS profile in Config.Sensor
                        QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
                        qosProfile.SetReliability(reliabilityPolicy);
                        qosProfile.SetHistory(historyPolicy, historyDepth);
                        qosProfile.SetDurability(durabilityPolicy);

                        sensor = new LidarModifier(this, sensor_config.topic, raysPerScan, lidarModifierComputeShader, ROSNode(), qosProfile, inModule.GetSDFs());
                        LidarModifier lidarModifier = (LidarModifier)sensor;
                        lidarModifier.UpdateSDFRaytraceParameters(maxRaytraceDistance, hitThreshold, maxIterations); // TODO: Does this need to be called here?
                        break;
                    
                    case SensorType.Camera:
                        Camera cam = sensorGO.AddComponent<Camera>();
                        sensor = new CameraModifier(this, sensor_config.topic, "/zed/zed_node/depth/depth_registered", cam);
                        break;
                        
                    default:
                        sensor = null;
                        break;
                }
                sensors.Add(new Tuple<Config.Sensor, SensorModifier, Transform>(sensor_config, sensor, sensorGO.transform));
            }
            // Visuals
            visRoot = new GameObject("visuals").transform;
            visRoot.SetParent(baseLink);
            SpawnVisuals(visRoot, inAgent);

                
            // Spectator Cameras
            spectatorCameras = transform.Find("spectator_cameras");
            cinemachineCameras = spectatorCameras.GetComponentsInChildren<CinemachineCamera>(true);
            DisableAllSpectatorCameras();
            EnableSpectatorCamera(cinemachineCameras[camIdx], true);

            // HUD
            if (hud is null)
                hud = new HUD();
            hud.OnNextSpectatorCamera += NextSpectatorCamera;
            hud.OnPrevSpectatorCamera += PreviousSpectatorCamera;
        }

        public virtual void Teardown()
        {
            // Sensors
            foreach (Tuple<Config.Sensor, SensorModifier, Transform> sensor in sensors)
            {
                sensor.Item2.CleanUp();
                Destroy(sensor.Item3.gameObject);
            }
            sensors.Clear();

            // Visuals
            if (visRoot is not null)
                Destroy(visRoot);

            // Spectator Cameras
            if (spectatorCameras is not null)
                DisableAllSpectatorCameras();

            // HUD
            if (hud is not null)
            {
                hud.OnNextSpectatorCamera -= NextSpectatorCamera;
                hud.OnPrevSpectatorCamera -= PreviousSpectatorCamera;
            }

            // ROS2 Topics
            if (this.mVirtualBoundingBoxDetector != null)
                this.mVirtualBoundingBoxDetector.CleanUp();
            if (ROSNode() is not null)
            {
                ROS2_Bridge.ROS_CORE.RemoveNode(ROSNode());
                this.mNode = null;
            }
        }

        void SpawnVisuals(Transform inParent, Config.Agent inAgent)
        {
            string prefabPath;
            switch (inAgent.model)
            {
                default:
                    prefabPath = "Prefabs/Vehicles/Lincoln_MKZ_2020";
                    break;
            }
            GameObject prefab = Resources.Load<GameObject>(prefabPath);
            GameObject.Instantiate(prefab, inParent);
        }

        void ConnectToROS()
        {
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                string name = gameObject.name.Replace("(Clone)", "");
                name = name.Replace(" Variant", "");
                name = name.Replace(" ", "_");
                // This is sort of cheating but ROS2_Bridge is not immediately deleting nodes so this avoids a collision (~99% of the time)
                int randomSuffix = UnityEngine.Random.Range(0, 1000);
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(name + "_Agent_" + randomSuffix.ToString());
                // Bounding Box (Ground Truth)
                this.mVirtualBoundingBoxDetector = new VirtualBoundingBoxDetector(boundingBoxTopicName, ROSNode(), this.transform);
            }
        }

        protected void OdometryUpdate(nav_msgs.msg.Odometry msg)
        {
            this.mWorldPosition = Ros2Utility.Ros2ToUnityPosition(msg.Pose.Pose.Position);
            this.mWorldRotation = Ros2Utility.Ros2ToUnityRotation(msg.Pose.Pose.Orientation);
        }

        //TODO: Call this somewhere
        IEnumerator PublishGroundTruth()
        {
            while (true)
            {
                yield return new WaitForSeconds(1.0f / gt_pub_rate);
                yield return new WaitForEndOfFrame();
                this.mVirtualBoundingBoxDetector.PublishNearbyVirtualObjects(baseLink, detectionRadius);
            }
        }

        IEnumerator PublishCameraImages()
        {
            while (true)
            {
                yield return new WaitForSeconds(1.0f / camera_pub_rate);
                foreach (var sensor in sensors)
                {
                    switch (sensor.Item1.type)
                    {
                        case SensorType.Camera:
                            CameraModifier cameraModifier = (CameraModifier)sensor.Item2;
                            cameraModifier.Publish(); 
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        public ROS2Node ROSNode()
        {
            return this.mNode;
        }

        public Vector3 WorldPosition()
        {
            return this.mWorldPosition;
        }

        public Quaternion WorldRotation()
        {
            return this.mWorldRotation;
        }

        protected void DisableAllSpectatorCameras()
        {
            foreach (CinemachineCamera cam in cinemachineCameras)
                EnableSpectatorCamera(cam, false);
        }

        protected void EnableSpectatorCamera(CinemachineCamera inCamera, bool inEnable)
        {
            inCamera.gameObject.SetActive(inEnable);
        }

        public void SetLayerCulling(Camera camera, string layerName, bool shouldRender)
        {
            int layerIndex = LayerMask.NameToLayer(layerName);
            if (layerIndex == -1)
            {
                Debug.LogWarning($"Layer '{layerName}' does not exist.");
                return;
            }

            int layerMask = 1 << layerIndex;
            if (shouldRender)
            {
                camera.cullingMask |= layerMask;
                camera.clearFlags = CameraClearFlags.Skybox;
            }
            else
            {
                camera.cullingMask &= ~layerMask;
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = Color.black;
            }
        }

        public void NextSpectatorCamera()
        {
            camIdx = (camIdx + 1) % cinemachineCameras.Length;
        }

        public void PreviousSpectatorCamera()
        {
            camIdx = (camIdx - 1) % cinemachineCameras.Length;
        }
    }
}
