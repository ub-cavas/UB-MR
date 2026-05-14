using ROS2;
using System.Collections.Generic;
//using vision_msgs.msg;
using autoware_perception_msgs.msg;
using UnityEngine;
using CAVAS.UB_MR.ROS2;
using System.Linq;

namespace CAVAS.UB_MR.DT.Sensors
{
    public class VirtualBoundingBoxDetector
    {
        static List<VirtualObject> sVirtualObjects;

        float mDetectionRadius;
        Transform mTransform;
        IPublisher<DetectedObjects> mObstacleBoundingBoxPublisher;

        public VirtualBoundingBoxDetector(string inTopicName, ROS2Node inNode, Transform inTransform)
        {
            if (sVirtualObjects == null)
            {
                sVirtualObjects = new List<VirtualObject>();
            }
            this.mTransform = inTransform;
            this.mObstacleBoundingBoxPublisher = inNode.CreatePublisher<DetectedObjects>(inTopicName);
        }

        public static void AddVirtualObjectToDatabase(VirtualObject vObj)
        {
            if (sVirtualObjects == null)
                sVirtualObjects = new List<VirtualObject>();
            if (sVirtualObjects.Contains(vObj) == false)
                sVirtualObjects.Add(vObj);
        }

        public static void UpdateVirtualObjectDatabase()
        {
            ClearVirtualObjectDatabase();
            VirtualObject[] objects = GameObject.FindObjectsByType<VirtualObject>(FindObjectsSortMode.None);
            // Add new virtual objects
            foreach (VirtualObject vObj in objects)
            {
                if (sVirtualObjects.Contains(vObj) == false)
                    sVirtualObjects.Add(vObj);
            }
        }

        public static void ClearVirtualObjectDatabase()
        {
            sVirtualObjects.Clear();
        }

        public List<VirtualObject> GetNearbyObstacles(Transform baseLink, float radius)
        {
            List<VirtualObject> nearbyObjects = new List<VirtualObject>();
            foreach (VirtualObject vObj in sVirtualObjects)
            {
                if (Vector3.Distance(vObj.transform.position, baseLink.position) <= radius)
                    nearbyObjects.Add(vObj);
            }
            return nearbyObjects;
        }

        public void PublishNearbyVirtualObjects(Transform baseLink, float detectionRadius)
        {
            // Message Structure + Header
            DetectedObjects detectedObjectMsg = new DetectedObjects();
            detectedObjectMsg.Header = new std_msgs.msg.Header();
            detectedObjectMsg.Header.Frame_id = "base_link";
            builtin_interfaces.msg.Time time = new builtin_interfaces.msg.Time();
            time.Sec = (int)UnityEngine.Time.timeSinceLevelLoad;
            detectedObjectMsg.Header.Stamp = time; //TODO: get correct timestamp

            List<VirtualObject> virtualObjects = GetNearbyObstacles(baseLink, detectionRadius);
            DetectedObject[] detectedObjects = new DetectedObject[virtualObjects.Count];
            for (int i = 0; i < virtualObjects.Count; i++)
            {
                // Extract Unity Object Properties
                VirtualObject virtualObject = virtualObjects[i];
                Bounds bounds = virtualObject.GetBoundingBox();
                Vector3 worldCenter = bounds.center;
                Vector3 localCenter = baseLink.InverseTransformPoint(worldCenter);
                Quaternion worldRot = virtualObject.transform.rotation;
                Quaternion localRot = Quaternion.Inverse(baseLink.rotation) * worldRot;
                Vector3 ros2Center = Ros2Utility.UnityToRos2Position(localCenter);
                Quaternion ros2Rotation = Ros2Utility.UnityToRosRotation(localRot);

                // ROS Object
                DetectedObject obj = new DetectedObject();

                // Classification
                ObjectClassification classification = new ObjectClassification();
                classification.Label = ObjectClassification.CAR; //TODO: support other types of classifications
                classification.Probability = 1.0f;
                obj.Classification = new ObjectClassification[] { classification };

                // Pose
                obj.Kinematics.Pose_with_covariance.Pose.Position.X = ros2Center.x;
                obj.Kinematics.Pose_with_covariance.Pose.Position.Y = ros2Center.y;
                obj.Kinematics.Pose_with_covariance.Pose.Position.Z = ros2Center.z;
                // Rotation
                obj.Kinematics.Pose_with_covariance.Pose.Orientation.X = ros2Rotation.x;
                obj.Kinematics.Pose_with_covariance.Pose.Orientation.Y = ros2Rotation.y;
                obj.Kinematics.Pose_with_covariance.Pose.Orientation.Z = ros2Rotation.z;
                obj.Kinematics.Pose_with_covariance.Pose.Orientation.W = ros2Rotation.w;
                // Pose Covariance (6x6 matrix, row-major, diagonal elements)
                obj.Kinematics.Pose_with_covariance.Covariance[0]  = 0.01; // x
                obj.Kinematics.Pose_with_covariance.Covariance[7]  = 0.01; // y
                obj.Kinematics.Pose_with_covariance.Covariance[14] = 0.01; // z
                obj.Kinematics.Pose_with_covariance.Covariance[21] = 0.01; // roll
                obj.Kinematics.Pose_with_covariance.Covariance[28] = 0.01; // pitch
                obj.Kinematics.Pose_with_covariance.Covariance[35] = 0.01; // yaw
                // Twist //TODO: should not be defaulting to 0.0
                obj.Kinematics.Twist_with_covariance.Twist.Linear.X = 0.0f;
                obj.Kinematics.Twist_with_covariance.Twist.Linear.Y = 0.0f;

                // Bounding Box
                Shape bbox = new Shape();
                bbox.Type = Shape.BOUNDING_BOX;
                Vector3 ros2Size = Ros2Utility.UnityToRos2Scale(bounds.size);
                bbox.Dimensions.X = ros2Size.x;
                bbox.Dimensions.Y = ros2Size.y;
                bbox.Dimensions.Z = ros2Size.z;
                obj.Shape = bbox;

                detectedObjects[i] = obj;
            }
            detectedObjectMsg.Objects = detectedObjects;
            detectedObjectMsg.WriteNativeMessage();
            this.mObstacleBoundingBoxPublisher.Publish(detectedObjectMsg);
        }

        public void CleanUp()
        {
            // TODO: Implement cleanup logic if necessary
        }

    }
}
