using ROS2;
using CAVAS.UB_MR.ROS2;

namespace CAVAS.UB_MR.DT.Vehicle
{
    public class DynamicAgent : Agent
    {
        string worldTransformationTopicName = "/localization/kinematic_state";
        
        ISubscription<nav_msgs.msg.Odometry> odometrySubscriber;
        
        protected override void Update()
        {
            base.Update();
            SnapUpdate();
        }

        public override void Setup(Config.Agent inAgent, Module inModule)
        {
            base.Setup(inAgent, inModule);
            // Odometry
            if (ROS2_Bridge.ROS_CORE.Ok())
            {
                this.odometrySubscriber = ROSNode().CreateSubscription<nav_msgs.msg.Odometry>(worldTransformationTopicName, OdometryUpdate);
                print("Subscribed to odometry");
            }
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

        void SnapUpdate()
        {
            this.transform.position = WorldPosition();
            this.transform.rotation = WorldRotation();
        }
    }

}
