using UnityEngine;
using sensor_msgs.msg;
using ROS2;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class LidarVisualizer : MonoBehaviour
    {
        [SerializeField] string lidarTopicName = "/scan"; // Topic name for Lidar scans
        ROS2Node mNode;
        ISubscription<LaserScan> mLidarSubscriber;

        void Awake()
        {
            if (ROS2_Bridge.ROS_CORE.Ok() && this.mNode == null)
            {
                string name = gameObject.name.Replace("(Clone)", "");
                name = name.Replace(" Variant", "");
                // This is sort of cheating but ROS2_Bridge is not immediately deleting nodes so this avoids a collision
                int randomSuffix = UnityEngine.Random.Range(0, 1000);
                this.mNode = ROS2_Bridge.ROS_CORE.CreateNode(name + "_Digital_Twin_Lidar_" + randomSuffix.ToString());
                // Lidar Scan Subscription
                this.mLidarSubscriber = this.mNode.CreateSubscription<LaserScan>(lidarTopicName, VisualizeLidarScan);
            }
        }
        
        void VisualizeLidarScan(LaserScan scan)
        {
            // TODO: Draw points from the Lidar scan
        }
    }
}
