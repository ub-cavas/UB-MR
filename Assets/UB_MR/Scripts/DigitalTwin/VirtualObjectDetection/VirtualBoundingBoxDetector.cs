using ROS2;
using System.Collections.Generic;
using vision_msgs.msg;
using UnityEngine;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection
{
    public class VirtualBoundingBoxDetector
    {
        static List<VirtualObject> sVirtualObjects;

        float mDetectionRadius;
        Transform mTransform;
        IPublisher<BoundingBox3DArray> mObstacleBoundingBoxPublisher;

        public VirtualBoundingBoxDetector(string inTopicName, ROS2Node inNode, Transform inTransform)
        {
            if (sVirtualObjects == null)
            {
                sVirtualObjects = new List<VirtualObject>();
            }
            this.mTransform = inTransform;
            this.mObstacleBoundingBoxPublisher = inNode.CreatePublisher<BoundingBox3DArray>(inTopicName);
        }

        public static void AddVirtualObjectToDatabase(VirtualObject vObj)
        {
            if (sVirtualObjects == null)
            {
                sVirtualObjects = new List<VirtualObject>();
            }
            if (sVirtualObjects.Contains(vObj) == false)
            {
                sVirtualObjects.Add(vObj);
            }
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

        public List<VirtualObject> GetNearbyObstacles(float radius)
        {
            List<VirtualObject> nearbyObjects = new List<VirtualObject>();
            foreach (VirtualObject vObj in sVirtualObjects)
            {
                if (Vector3.Distance(vObj.transform.position, this.mTransform.position) <= radius)
                    nearbyObjects.Add(vObj);
            }
            return nearbyObjects;
        }

        public void PublishNearbyVirtualObjects(float detectionRadius)
        {
            // TODO: Bounding Boxes position and orientation should be relative to the Ego-Vehicle, not the world
            vision_msgs.msg.BoundingBox3DArray msg = new vision_msgs.msg.BoundingBox3DArray();
            // Header
            msg.Header = new std_msgs.msg.Header();
            msg.Header.Frame_id = "world";
            builtin_interfaces.msg.Time time = new builtin_interfaces.msg.Time();
            time.Sec = (int)UnityEngine.Time.timeSinceLevelLoad;
            msg.Header.Stamp = time;

            List<VirtualObject> virtualObjects = GetNearbyObstacles(detectionRadius);
            var boxes = new BoundingBox3D[virtualObjects.Count];
            for (int i = 0; i < boxes.Length; i++)
            {
                VirtualObject virtualObject = virtualObjects[i];
                Bounds bounds = virtualObject.GetBoundingBox();
                BoundingBox3D bbox = new BoundingBox3D();
                bbox.Center = new geometry_msgs.msg.Pose();
                // Position
                bbox.Center.Position = new geometry_msgs.msg.Point();
                bbox.Center.Position.X = bounds.center.x;
                bbox.Center.Position.Y = bounds.center.y;
                bbox.Center.Position.Z = bounds.center.z;
                // Orientation
                bbox.Center.Orientation = new geometry_msgs.msg.Quaternion();
                bbox.Center.Orientation.X = virtualObject.transform.rotation.x;
                bbox.Center.Orientation.Y = virtualObject.transform.rotation.y;
                bbox.Center.Orientation.Z = virtualObject.transform.rotation.z;
                bbox.Center.Orientation.W = virtualObject.transform.rotation.w;
                // Size
                bbox.Size = new geometry_msgs.msg.Vector3();
                bbox.Size.X = bounds.size.x;
                bbox.Size.Y = bounds.size.y;
                bbox.Size.Z = bounds.size.z;

                boxes[i] = bbox;
            }
            msg.Boxes = boxes;
            msg.WriteNativeMessage();
            this.mObstacleBoundingBoxPublisher.Publish(msg);

            //Debug.Log("Published " + virtualObjects.Count + " virtual objects to topic: " + virtualObjectsTopicName);
        }

        public void CleanUp()
        {
            // TODO: Implement cleanup logic if necessary
        }

    }
}
