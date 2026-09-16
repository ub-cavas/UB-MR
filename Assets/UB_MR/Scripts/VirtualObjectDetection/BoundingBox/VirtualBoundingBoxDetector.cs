using ROS2;
using System.Collections.Generic;
using autoware_perception_msgs.msg;
using UnityEngine;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.DT.Sensors
{
    public sealed class VirtualBoundingBoxDetector
    {
        static readonly HashSet<VirtualObject> virtualObjects = new HashSet<VirtualObject>();
        readonly ROS2Node node;
        readonly Transform egoRoot;
        IPublisher<DetectedObjects> publisher;

        public VirtualBoundingBoxDetector(string topic, ROS2Node node, Transform egoRoot)
        {
            this.node = node;
            this.egoRoot = egoRoot;
            var qos = new QualityOfServiceProfile();
            qos.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE);
            qos.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE);
            qos.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 1);
            publisher = node.CreatePublisher<DetectedObjects>(topic, qos);
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        public static void ClearVirtualObjectDatabase() => virtualObjects.Clear();
        public static void AddVirtualObjectToDatabase(VirtualObject obj) => virtualObjects.Add(obj);
        public static void RemoveVirtualObjectFromDatabase(VirtualObject obj) => virtualObjects.Remove(obj);

        public static void UpdateVirtualObjectDatabase()
        {
            virtualObjects.Clear();
            foreach (var obj in Object.FindObjectsByType<VirtualObject>(FindObjectsSortMode.None))
                if (obj.isActiveAndEnabled) virtualObjects.Add(obj);
        }

        public static DetectedObjects BuildMessage(Transform baseLink, Transform egoRoot, float radius,
            builtin_interfaces.msg.Time stamp)
        {
            var objects = new List<DetectedObject>();
            foreach (var obj in virtualObjects)
            {
                if (obj == null || !obj.isActiveAndEnabled ||
                    (egoRoot != null && obj.transform.IsChildOf(egoRoot))) continue;
                if (!obj.TryGetBoundingBox(out var center, out var rotation, out var size)) continue;
                if ((center - baseLink.position).sqrMagnitude > radius * radius) continue;
                objects.Add(CreateDetection(baseLink, center, rotation, size, obj.Classification));
            }
            return new DetectedObjects
            {
                Header = new std_msgs.msg.Header { Frame_id = "base_link", Stamp = stamp },
                Objects = objects.ToArray()
            };
        }

        public static DetectedObject CreateDetection(Transform baseLink, Vector3 center,
            Quaternion rotation, Vector3 size, VirtualObjectClassification classification)
        {
            Quaternion inverse = Quaternion.Inverse(baseLink.rotation);
            Vector3 position = Ros2Utility.UnityToRos2Position(inverse * (center - baseLink.position));
            Quaternion orientation = Ros2Utility.UnityToRosRotation(inverse * rotation);
            Vector3 dimensions = Ros2Utility.UnityToRos2Scale(size);
            var obj = new DetectedObject
            {
                Existence_probability = 1.0f,
                Classification = new[] { new ObjectClassification { Label = (byte)classification, Probability = 1.0f } }
            };
            var pose = obj.Kinematics.Pose_with_covariance;
            pose.Pose.Position.X = position.x;
            pose.Pose.Position.Y = position.y;
            pose.Pose.Position.Z = position.z;
            pose.Pose.Orientation.X = orientation.x;
            pose.Pose.Orientation.Y = orientation.y;
            pose.Pose.Orientation.Z = orientation.z;
            pose.Pose.Orientation.W = orientation.w;
            for (int i = 0; i < 6; i++) pose.Covariance[i * 7] = 0.01;
            obj.Kinematics.Has_position_covariance = true;
            obj.Kinematics.Orientation_availability = DetectedObjectKinematics.AVAILABLE;
            obj.Kinematics.Has_twist = false;
            obj.Kinematics.Has_twist_covariance = false;
            obj.Shape.Type = Shape.BOUNDING_BOX;
            obj.Shape.Dimensions.X = dimensions.x;
            obj.Shape.Dimensions.Y = dimensions.y;
            obj.Shape.Dimensions.Z = dimensions.z;
            return obj;
        }

        public void PublishNearbyVirtualObjects(Transform baseLink, float radius, builtin_interfaces.msg.Time stamp)
        {
            if (publisher != null) publisher.Publish(BuildMessage(baseLink, egoRoot, radius, stamp));
        }

        public void CleanUp()
        {
            if (publisher == null) return;
            if (Ros2cs.Ok()) node.RemovePublisher<DetectedObjects>(publisher);
            publisher = null;
        }
    }
}
