using System.Threading;
using ROS2;
using UnityEngine;

namespace CAVAS.UB_MR.ROS2
{
    /// <summary>
    /// Follows simulation time published on /clock.
    ///
    /// The CARLA bridge stamps every sensor message with CARLA's elapsed simulation seconds and
    /// publishes the same value on /clock, so Autoware runs on simulation time. Messages we
    /// originate must use that same clock. Wall time would sit decades ahead of it and every
    /// consumer downstream would discard our messages as far-future data.
    ///
    /// Until a /clock message arrives this falls back to ROS system time, so a stack without a
    /// simulator still gets sensible stamps.
    /// </summary>
    public class SimulationClock
    {
        const string CLOCK_TOPIC = "/clock";
        const long NANOSECONDS_PER_SECOND = 1000000000L;

        readonly ROS2Node mNode;
        ISubscription<rosgraph_msgs.msg.Clock> mClockSubscriber;

        // Written from the ROS executor thread, read from the main thread.
        long mLatestNanoseconds = -1;

        public bool HasSimulationTime
        {
            get { return Interlocked.Read(ref this.mLatestNanoseconds) >= 0; }
        }

        public SimulationClock(ROS2Node inNode)
        {
            this.mNode = inNode;
            if (inNode == null)
            {
                Debug.LogWarning("SimulationClock created without a ROS2 node; stamps will be zero.");
                return;
            }
            this.mClockSubscriber = inNode.CreateSubscription<rosgraph_msgs.msg.Clock>(CLOCK_TOPIC, OnClock);
        }

        void OnClock(rosgraph_msgs.msg.Clock inClock)
        {
            if (inClock == null || inClock.Clock_ == null)
                return;
            long nanoseconds = (long)inClock.Clock_.Sec * NANOSECONDS_PER_SECOND + inClock.Clock_.Nanosec;
            Interlocked.Exchange(ref this.mLatestNanoseconds, nanoseconds);
        }

        /// <summary>
        /// Writes the current simulation time into an existing message timestamp.
        /// </summary>
        public void Stamp(builtin_interfaces.msg.Time outTime)
        {
            if (outTime == null)
                return;

            long nanoseconds = Interlocked.Read(ref this.mLatestNanoseconds);
            if (nanoseconds < 0)
            {
                // No simulator on this graph: fall back to ROS system time.
                if (this.mNode != null)
                    this.mNode.clock.UpdateROSClockTime(outTime);
                return;
            }

            outTime.Sec = (int)(nanoseconds / NANOSECONDS_PER_SECOND);
            outTime.Nanosec = (uint)(nanoseconds % NANOSECONDS_PER_SECOND);
        }

        public void CleanUp()
        {
            if (Ros2cs.Ok() && this.mNode != null && this.mClockSubscriber != null)
                this.mNode.RemoveSubscription<rosgraph_msgs.msg.Clock>(this.mClockSubscriber);
            this.mClockSubscriber = null;
        }
    }
}
