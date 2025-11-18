using System.Collections.Generic;
using CAVAS.UB_MR.Config;
using Unity.Cinemachine;
using UnityEngine;
using ROS2;
using CAVAS.UB_MR.ROS2;
using System.Collections;
using CAVAS.UB_MR.DT.VirtualObjectDetection;

namespace CAVAS.UB_MR.DT
{
    public class Agent : MonoBehaviour
    {
        Transform baseLink;
        Transform visRoot;
        Transform spectatorCameras;
        List<GameObject> sensors;
        HUD hud;
        CinemachineCamera[] cinemachineCameras;
        int camIdx = 0;
        ROS2Node mNode;
        Vector3 mWorldPosition;
        Quaternion mWorldRotation;


        [Header("Ground Truth Bounding Boxes")]
        float gt_pub_rate = 1.0f; // 1 FPS
        string boundingBoxTopicName = "/virtual_obstacles"; // Topic name for publishing virtual object bounding boxes
        float detectionRadius = 30.0f;
        VirtualBoundingBoxDetector mVirtualBoundingBoxDetector;

        public virtual void Setup(Config.Agent inAgent)
        {
            // Name
            this.gameObject.name = inAgent.name;

            baseLink = transform.Find("base_link");
            // Sensors
            sensors = new List<GameObject>();
            foreach (Sensor sensor in inAgent.sensors.Values)
            {
                GameObject sensorGO = GameObject.Instantiate(new GameObject(), baseLink);
                sensorGO.name = sensor.name;
                sensorGO.transform.SetLocalPositionAndRotation(sensor.position, Quaternion.Euler(sensor.rotation));
                sensors.Add(sensorGO);
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
        }

        public virtual void Teardown()
        {
            // Sensors
            for (int i = 0; i < sensors.Count; i++)
                Destroy(sensors[i]);
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

        protected virtual void ConnectToROS()
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
