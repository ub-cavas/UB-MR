using ROS2;

namespace CAVAS.UB_MR.DT.Sensors
{
    /// <summary>Schedules samples in the same time domain as Autoware, independently of Unity time.</summary>
    public sealed class DetectionClock
    {
        readonly ROS2Node node;
        readonly bool useSimTime;
        readonly object gate = new object();
        readonly DetectionSchedule schedule;
        ISubscription<rosgraph_msgs.msg.Clock> subscription;
        long simulationNanoseconds;
        bool receivedClock;
        bool disposed;

        public DetectionClock(ROS2Node node, bool useSimTime, float rateHz)
        {
            this.node = node;
            this.useSimTime = useSimTime;
            schedule = new DetectionSchedule(rateHz);
            if (useSimTime)
            {
                // Compatible with both best-effort and reliable /clock publishers.
                var qos = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);
                subscription = node.CreateSubscription<rosgraph_msgs.msg.Clock>("/clock", message =>
                {
                    lock (gate)
                    {
                        if (disposed) return;
                        simulationNanoseconds = (long)message.Clock_.Sec * 1_000_000_000L + message.Clock_.Nanosec;
                        receivedClock = true;
                    }
                }, qos);
            }
        }

        public bool TrySample(out builtin_interfaces.msg.Time stamp)
        {
            stamp = null;
            if (disposed) return false;
            long now;
            if (useSimTime)
            {
                lock (gate)
                {
                    // Zero would request the latest TF rather than the sample's actual transform.
                    if (!receivedClock || simulationNanoseconds <= 0) return false;
                    now = simulationNanoseconds;
                }
            }
            else
            {
                var time = new builtin_interfaces.msg.Time();
                node.clock.UpdateROSClockTime(time);
                now = (long)time.Sec * 1_000_000_000L + time.Nanosec;
            }
            if (!schedule.ShouldPublish(now)) return false;
            stamp = new builtin_interfaces.msg.Time
            {
                Sec = (int)(now / 1_000_000_000L), Nanosec = (uint)(now % 1_000_000_000L)
            };
            return true;
        }

        public void CleanUp()
        {
            lock (gate)
            {
                if (disposed) return;
                disposed = true;
            }
            if (Ros2cs.Ok() && subscription != null)
                node.RemoveSubscription<rosgraph_msgs.msg.Clock>(subscription);
            subscription = null;
        }
    }

    public sealed class DetectionSchedule
    {
        readonly long interval;
        long lastObserved = -1;
        long nextSample;

        public DetectionSchedule(float rateHz)
        {
            if (float.IsNaN(rateHz) || float.IsInfinity(rateHz) || rateHz <= 0)
                throw new System.ArgumentOutOfRangeException(nameof(rateHz));
            interval = System.Math.Max(1L, (long)(1_000_000_000.0 / rateHz));
        }

        public bool ShouldPublish(long now)
        {
            if (now <= 0 || now == lastObserved) return false;
            if (now < lastObserved) nextSample = now;
            lastObserved = now;
            if (now < nextSample) return false;
            nextSample = now + interval;
            return true;
        }
    }
}
