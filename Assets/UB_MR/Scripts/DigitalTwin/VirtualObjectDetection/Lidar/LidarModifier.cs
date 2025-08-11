using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace CAVAS.UB_MR.DT.VirtualObjectDetection.Lidar
{
    public class LidarModifier
    {
        ROS2Node mNode;
        ISubscription<LaserScan> mLidarSubscriber;
        Transform mLidarTransform;

        public LidarModifier(Transform inLidarSensorTransform, string inTopicName, ROS2Node inNode, QualityOfServiceProfile inQoSProfile)
        {
            QualityOfServiceProfile qosProfile = new QualityOfServiceProfile();
            qosProfile.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT);
            qosProfile.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 2);
            qosProfile.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE);
            this.mLidarTransform =  inLidarSensorTransform;
            this.mNode = inNode;
            this.mLidarSubscriber = this.mNode.CreateSubscription<LaserScan>(inTopicName, GPU_LidarModification, qosProfile);
        }

        void GPU_LidarModification(LaserScan inLaserScan)
        {
            
        }

        public void CleanUp()
        {
            
        }

    }
}
