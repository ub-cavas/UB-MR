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
        Quaternion mWorldRotation = Quaternion.identity;
        protected bool HasOdometry { get; private set; }
        protected virtual bool ReadyForDetection => true;

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
        
        VirtualObjectRecognitionSettings recognition;
        VirtualBoundingBoxDetector mVirtualBoundingBoxDetector;
        DetectionClock detectionClock;
        bool tornDown;

        protected virtual IEnumerator Start()
        {
            yield return new WaitUntil(() => ROS2_Bridge.ROS_CORE.Ok() && this.mNode is not null);
            
            if (sensors.Exists(sensor => sensor.Item1.type == SensorType.Camera))
                StartCoroutine(PublishCameraImages());
        }

        protected virtual void Update()
        {
            foreach (var sensor in sensors)
            {
                if (sensor.Item2 is LidarModifier lidar)
                {
                    if (lidar.TryModify(sensor.Item3)) lidar.Publish();
                }
                else if (sensor.Item2 is LidarPassthrough passthrough)
                    passthrough.Publish();
            }
        }

        protected virtual void LateUpdate()
        {
            // All Update calls (including DynamicAgent.SnapUpdate) have completed.
            if (!tornDown && ReadyForDetection && detectionClock != null && detectionClock.TrySample(out var stamp))
                mVirtualBoundingBoxDetector.PublishNearbyVirtualObjects(baseLink, recognition.detectionRadiusMeters, stamp);
        }

        public virtual void Setup(Config.Agent inAgent, Module inModule)
        {
            recognition = inAgent.recognition ?? new VirtualObjectRecognitionSettings();
            if (!recognition.TryValidate(out string error))
                throw new ArgumentException($"Invalid virtual-object recognition settings: {error}");
            tornDown = false;
            // Name
            this.gameObject.name = inAgent.name;
            // Computer Shaders
            if (recognition.mode == VirtualObjectRecognitionMode.LidarModification)
                this.lidarModifierComputeShader = Resources.Load<ComputeShader>("Scripts/SDFRaymarch");

            baseLink = transform.Find("base_link");

            // ROS2
            ConnectToROS();
            if (recognition.mode == VirtualObjectRecognitionMode.BoundingBoxInjection)
            {
                mVirtualBoundingBoxDetector = new VirtualBoundingBoxDetector(recognition.boundingBoxTopic, ROSNode(), transform);
                detectionClock = new DetectionClock(ROSNode(), recognition.useSimTime, recognition.publishRateHz);
                Debug.Log($"Direct bounding boxes: {recognition.boundingBoxTopic}, {recognition.publishRateHz} Hz, " +
                    $"{recognition.detectionRadiusMeters} m. Clock: {(recognition.useSimTime ? "/clock (waiting for time)" : "ROS system time")}.");
            }

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

                        if (recognition.mode == VirtualObjectRecognitionMode.BoundingBoxInjection)
                            sensor = new LidarPassthrough(sensor_config.topic, ROSNode(), qosProfile);
                        else
                        {
                            var lidar = new LidarModifier(this, sensor_config.topic, raysPerScan,
                                lidarModifierComputeShader, ROSNode(), qosProfile, inModule.GetSDFs());
                            lidar.UpdateSDFRaytraceParameters(maxRaytraceDistance, hitThreshold, maxIterations);
                            sensor = lidar;
                        }
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

        protected virtual void OnDestroy() => Teardown();

        public virtual void Teardown()
        {
            if (tornDown) return;
            tornDown = true;
            StopAllCoroutines();
            detectionClock?.CleanUp();
            detectionClock = null;
            // Sensors
            foreach (Tuple<Config.Sensor, SensorModifier, Transform> sensor in sensors)
            {
                sensor.Item2?.CleanUp();
                if (sensor.Item3 != null) Destroy(sensor.Item3.gameObject);
            }
            sensors.Clear();

            // Visuals
            if (visRoot is not null)
                Destroy(visRoot.gameObject);

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
                if (Ros2cs.Ok()) ROS2_Bridge.ROS_CORE.RemoveNode(ROSNode());
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
                name = name.Replace(" ", "_").Replace("-", "_");
                // This is sort of cheating but ROS2_Bridge is not immediately deleting nodes so this avoids a collision (~99% of the time)
                int randomSuffix = UnityEngine.Random.Range(0, 1000);
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(name + "_Agent_" + randomSuffix.ToString());

            }
        }

        protected void OdometryUpdate(nav_msgs.msg.Odometry msg)
        {
            this.mWorldPosition = Ros2Utility.Ros2ToUnityPosition(msg.Pose.Pose.Position);
            this.mWorldRotation = Ros2Utility.Ros2ToUnityRotation(msg.Pose.Pose.Orientation);
            HasOdometry = true;
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

        protected Vector3 WorldPosition()
        {
            return this.mWorldPosition;
        }

        protected Quaternion WorldRotation()
        {
            return this.mWorldRotation;
        }

        protected void DisableAllSpectatorCameras()
        {
            if (cinemachineCameras == null) return;
            foreach (CinemachineCamera cam in cinemachineCameras)
                EnableSpectatorCamera(cam, false);
        }

        protected void EnableSpectatorCamera(CinemachineCamera inCamera, bool inEnable)
        {
            if (inCamera != null) inCamera.gameObject.SetActive(inEnable);
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
