using ROS2;
using sensor_msgs.msg;

namespace CAVAS.UB_MR.DT.Sensors.Lidar
{
    /// <summary>One pending scan; forwards the complete message without decoding or modifying it.</summary>
    public sealed class LidarPassthrough : SensorModifier
    {
        readonly ROS2Node node;
        readonly object gate = new object();
        ISubscription<PointCloud2> subscription;
        IPublisher<PointCloud2> publisher;
        PointCloud2 pending;
        bool disposed;

        public LidarPassthrough(string topic, ROS2Node node, QualityOfServiceProfile inputQos)
        {
            this.node = node;
            publisher = node.CreatePublisher<PointCloud2>(topic + "_modified");
            subscription = node.CreateSubscription<PointCloud2>(topic, message =>
            {
                lock (gate)
                {
                    if (!disposed) pending = message;
                }
            }, inputQos);
        }

        public override void Publish()
        {
            PointCloud2 message;
            lock (gate)
            {
                if (disposed) return;
                message = pending;
                pending = null;
            }
            if (message != null) publisher.Publish(message);
        }

        public override void CleanUp()
        {
            lock (gate)
            {
                if (disposed) return;
                disposed = true;
                pending = null;
            }
            if (Ros2cs.Ok())
            {
                if (subscription != null) node.RemoveSubscription<PointCloud2>(subscription);
                if (publisher != null) node.RemovePublisher<PointCloud2>(publisher);
            }
            subscription = null;
            publisher = null;
        }
    }
}
