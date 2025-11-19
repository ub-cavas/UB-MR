using ROS2;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.DT.Vehicle
{
    public class DynamicAgent : Agent
    {
        string worldTransformationTopicName = "/world_transform";
        string virtualCameraImageTopicName = "/virtual_camera/image_raw";
        string virtualCameraDepthTopicName = "/virtual_camera/depth";
        string lidarTopicName = "/sensing/lidar/top/aw_points";
        ISubscription<nav_msgs.msg.Odometry> odometrySubscriber;
        
        public override void Setup(Config.Agent inAgent)
        {
            base.Setup(inAgent);

            // Odometry
            if (ROS2_Bridge.ROS_CORE.Ok())
                this.odometrySubscriber = ROSNode().CreateSubscription<nav_msgs.msg.Odometry>(worldTransformationTopicName, OdometryUpdate);
        }

        public override void Teardown()
        {   
            // Odometry
            if (ROSNode() is not null && odometrySubscriber is not null)
            {
                ROSNode().RemoveSubscription<nav_msgs.msg.Odometry>(this.odometrySubscriber);
                this.odometrySubscriber = null;
            }

            base.Teardown();
        }
    }

}
