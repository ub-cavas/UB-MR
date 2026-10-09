using ROS2;
using sensor_msgs.msg;
using CAVAS.UB_MR.Telemetry;

namespace CAVAS.UB_MR.DT.Sensors.Lidar
{
    /// <summary>One pending scan; forwards the complete message without decoding or modifying it.</summary>
    public sealed class LidarPassthrough : SensorModifier
    {
        readonly ROS2Node node;
        readonly object gate = new object();
        readonly ResourceTelemetry telemetry;
        readonly SensorTelemetry timing;
        readonly IMonotonicClock clock;
        ISubscription<PointCloud2> subscription;
        IPublisher<PointCloud2> publisher;
        TimedMessage<PointCloud2> pending;
        bool disposed;

        public LidarPassthrough(string topic, ROS2Node node, QualityOfServiceProfile inputQos,
            ResourceTelemetry telemetry = null, string sensorName = null)
        {
            this.node = node;
            this.telemetry = telemetry;
            clock = telemetry?.Clock ?? MonotonicClock.Instance;
            timing = telemetry?.RegisterLidar(sensorName ?? topic, true);
            publisher = node.CreatePublisher<PointCloud2>(topic + "_modified");
            subscription = node.CreateSubscription<PointCloud2>(topic, message =>
            {
                double receivedAt = clock.Seconds;
                lock (gate)
                {
                    if (disposed) return;
                    telemetry?.SensorPayload.AddReceived(message?.Data?.LongLength ?? 0);
                    if (message != null) pending = new TimedMessage<PointCloud2>(message, receivedAt, true);
                }
            }, inputQos);
        }

        public override void Publish()
        {
            TimedMessage<PointCloud2> scan;
            lock (gate)
            {
                if (disposed) return;
                scan = pending;
                pending = null;
            }
            if (scan == null) return;
            publisher.Publish(scan.Message);
            telemetry?.SensorPayload.AddSent(scan.Message.Data?.LongLength ?? 0);
            // Forward the complete original message; invalid payloads do not produce timing samples.
            if (PointCloudValidation.HasValidPayload(scan.Message)) timing?.RecordPublished(scan.ReceivedAt, true);
        }

        public override void CleanUp()
        {
            lock (gate)
            {
                if (disposed) return;
                disposed = true;
                timing?.Dispose();
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
