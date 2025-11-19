using System.Collections.Generic;
using CAVAS.UB_MR.Config;
using Unity.Cinemachine;
using UnityEngine;
using ROS2;
using CAVAS.UB_MR.ROS2;
using System.Collections;
using CAVAS.UB_MR.DT.VirtualObjectDetection;
using CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar;
using CAVAS.UB_MR.DT.VirtualObjectDetection.Camera;

namespace CAVAS.UB_MR.DT
{
    public class Agent : MonoBehaviour
    {
        Transform baseLink;
        Transform visRoot;
        Transform spectatorCameras;
        Dictionary<string, GameObject> sensors;
        HUD hud;
        ROS2Node mNode;
        Vector3 mWorldPosition;
        Quaternion mWorldRotation;

        // TODO: Store QOS settings in the sensor
        #region LiDAR
        ComputeShader lidarModifierComputeShader;
        Dictionary<string, LidarModifier> lidarModifiers;

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
        Dictionary<string, VirtualCameraOverlay> cameraModifiers;
        int imageWidth = 640;
        int imageHeight = 480;
        #endregion

        #region Spectator
        CinemachineCamera[] cinemachineCameras;
        int camIdx = 0;
        #endregion

        


        [Header("Ground Truth Bounding Boxes")]
        float gt_pub_rate = 1.0f; // 1 FPS
        string boundingBoxTopicName = "/virtual_obstacles"; // Topic name for publishing virtual object bounding boxes
        float detectionRadius = 30.0f;
        VirtualBoundingBoxDetector mVirtualBoundingBoxDetector;

        protected virtual void Update()
        {
            // LiDAR Modification
            foreach (KeyValuePair<string, LidarModifier> lidar in lidarModifiers)
            {
                Transform lidarTransform = sensors[lidar.Key].transform;
                if (lidar.Value.TryModify(lidarTransform))
                    lidar.Value.PublishPCD();
            }
        }

        public virtual void Setup(Config.Agent inAgent)
        {
            // Name
            this.gameObject.name = inAgent.name;
            // Computer Shaders
            this.lidarModifierComputeShader = Resources.Load<ComputeShader>("Scripts/SDFRaymarch");

            baseLink = transform.Find("base_link");

            // Sensors
            sensors = new Dictionary<string, GameObject>();
            lidarModifiers = new Dictionary<string, LidarModifier>();
            cameraModifiers = new Dictionary<string, VirtualCameraOverlay>();

            foreach (Sensor sensor in inAgent.sensors.Values)
            {
                GameObject sensorGO = GameObject.Instantiate(new GameObject(), baseLink);
                sensorGO.name = sensor.name;
                sensorGO.transform.SetLocalPositionAndRotation(sensor.position, Quaternion.Euler(sensor.rotation));
                sensors[sensor.name] = sensorGO;
                switch (sensor.type)
                {
                    case SensorType.LiDAR:
                        // TODO: Store QOS profile in Sensor
                        QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
                        qosProfile.SetReliability(reliabilityPolicy);
                        qosProfile.SetHistory(historyPolicy, historyDepth);
                        qosProfile.SetDurability(durabilityPolicy);

                        //LidarModifier lidarModifier = new LidarModifier(this, sensor.topic, raysPerScan, lidarModifierComputeShader, ROSNode(), qosProfile, this.mSdfs[0]);
                        LidarModifier lidarModifier = new LidarModifier(this, sensor.topic, raysPerScan, lidarModifierComputeShader, ROSNode(), qosProfile, null);
                        lidarModifier.UpdateSDFRaytraceParameters(maxRaytraceDistance, hitThreshold, maxIterations); // Does this need to be called?
                        lidarModifiers[sensor.name] = lidarModifier;
                        break;
                    
                    case SensorType.Camera:
                        //TODO: Implement Camera Modification
                        //TODO: Store Camera Resolution in Sensor Data
                        VirtualCameraOverlay cameraModifier = new VirtualCameraOverlay(sensor.topic, sensor.topic + "_depth", ROSNode(), FindFirstObjectByType<Camera>(), imageWidth, imageHeight);
                        cameraModifiers[sensor.name] = cameraModifier;
                        break;
                        
                    default:
                        break;
                }
            }
            // Visuals
            visRoot = GameObject.Instantiate(new GameObject(), baseLink).transform;
            visRoot.name = "visuals";
                
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

            // ROS2
            ConnectToROS();
        }

        public virtual void Teardown()
        {
            // Sensors
            foreach (string name in sensors.Keys)
            {
                if (lidarModifiers.ContainsKey(name))
                    lidarModifiers[name].CleanUp();
                if (cameraModifiers.ContainsKey(name))
                    cameraModifiers[name].CleanUp();
                Destroy(sensors[name]);
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

        void ConnectToROS()
        {
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                string name = gameObject.name.Replace("(Clone)", "");
                name = name.Replace(" Variant", "");
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

        protected GameObject GetSensor(string inName)
        {
            return sensors[inName];
        }

        //TODO: Call this somewhere
        IEnumerator PublishGroundTruth()
        {
            while (true)
            {
                yield return new WaitForEndOfFrame();
                this.mVirtualBoundingBoxDetector.PublishNearbyVirtualObjects(detectionRadius);
                yield return new WaitForSeconds(1.0f / gt_pub_rate);
            }
        }

        //TODO: Call this somewhere
        IEnumerator PublishImage()
        {
            //this.mVirtualCameraOverlay.UpdateCameraResolution(imageWidth, imageHeight);
            //this.mVirtualCameraOverlay.CaptureAndPublishImage();
            yield return new WaitForSeconds(1.0f / gt_pub_rate);
        }

        protected ROS2Node ROSNode()
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
            foreach (CinemachineCamera cam in cinemachineCameras)
                EnableSpectatorCamera(cam, false);
        }

        protected void EnableSpectatorCamera(CinemachineCamera inCamera, bool inEnable)
        {
            DisableAllSpectatorCameras();
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
