using System;
using System.Collections.Generic;
using ROS2;
using sensor_msgs.msg;
using UnityEngine;

namespace CAVAS.UB_MR.DT.Sensors.Camera
{
    public class CameraModifier : SensorModifier
    {
        Agent owner;
        UnityEngine.Camera rgbCamera;
        DepthCamera depthCamera;

        ISubscription<Image> rgbSubscriber;
        ISubscription<Image> depthSubscriber;

        IPublisher<Image> mrFramePublisher;

        // Queues for incoming ROS images (timestamp + message)
        readonly Queue<(double stamp, Image msg)> rgbQueue = new();
        readonly Queue<(double stamp, Image msg)> depthQueue = new();

        // Single lock protecting BOTH queues
        readonly object syncLock = new();

        // Max allowed timestamp difference between matched frames (seconds)
        const double MaxSyncDelta = 0.02; // 20 ms, tune as needed

        public CameraModifier(Agent inOwner, string rgbTopicName, string depthTopicName, UnityEngine.Camera inCamera)
        {
            owner = inOwner;

            // Virtual camera
            rgbCamera = inCamera;
            depthCamera = new DepthCamera(inCamera);

            // Physical camera subscriptions
            rgbSubscriber = owner.ROSNode()
                .CreateSubscription<Image>(rgbTopicName, OnRGB_Receive);

            depthSubscriber = owner.ROSNode()
                .CreateSubscription<Image>(depthTopicName, OnDepth_Receive);

            // Mixed reality output publisher
            mrFramePublisher = owner.ROSNode()
                .CreatePublisher<Image>("mixed_reality_frame");
        }

        // Convert ROS2 builtin_interfaces/Time to double seconds
        static double StampToSeconds(builtin_interfaces.msg.Time t)
        {
            return t.Sec + t.Nanosec * 1e-9;
        }

        // ROS callback: runs on a background thread
        void OnRGB_Receive(Image inImage)
        {
            double t = StampToSeconds(inImage.Header.Stamp);

            lock (syncLock)
                rgbQueue.Enqueue((t, inImage));
        }

        // ROS callback: runs on a background thread
        void OnDepth_Receive(Image inImage)
        {
            double t = StampToSeconds(inImage.Header.Stamp);

            lock (syncLock)
                depthQueue.Enqueue((t, inImage));
        }

        /// <summary>
        /// Try to get a time-synchronized RGB + depth pair.
        /// Must be called on main thread; holds syncLock while working.
        /// </summary>
        bool TryGetSyncedPair(out Image rgb, out Image depth)
        {
            rgb = null;
            depth = null;

            lock (syncLock)
            {
                while (rgbQueue.Count > 0 && depthQueue.Count > 0)
                {
                    var (tRgb, msgRgb) = rgbQueue.Peek();
                    var (tDepth, msgDepth) = depthQueue.Peek();

                    double dt = tRgb - tDepth;
                    double absDt = Math.Abs(dt);

                    if (absDt <= MaxSyncDelta)
                    {
                        // Found a matched pair
                        rgbQueue.Dequeue();
                        depthQueue.Dequeue();

                        rgb = msgRgb;
                        depth = msgDepth;
                        return true;
                    }

                    // Drop the older one and keep searching
                    if (tRgb < tDepth)
                        rgbQueue.Dequeue();
                    else
                        depthQueue.Dequeue();
                }
            }

            return false;
        }

        /// <summary>
        /// Publishes a Image Message to a designated topic. Must be called from the main thread.
        /// </summary>
        public override void Publish()
        {
            if (!Ros2cs.Ok())   return;

            if (TryGetSyncedPair(out var physicalRgb, out var physicalDepth))
            {
                var mrFrame = ComputeMixedRealityFrame(physicalRgb, physicalDepth);
                if (mrFrame != null)
                    mrFramePublisher.Publish(mrFrame);
            }
        }

        /// <summary>
        /// Combine virtual RGB + physical RGB/Depth into a mixed reality frame.
        /// This is where you’ll do your occlusion logic.
        /// </summary>
        Image ComputeMixedRealityFrame(Image physicalRgb, Image physicalDepth)
        {
            // TODO:
            // 1. Use rgbCamera + depthCamera to get virtual RGB + depth.
            // 2. For each pixel:
            //    - Decode physical depth from physicalDepth.data
            //    - Get virtual depth from depthCamera
            //    - Compare depths, choose which color should be visible.
            // 3. Return a new Image (or reuse a buffer) as the mixed reality frame.

            // For now, just pass through physicalRgb as a placeholder.
            return physicalRgb;
        }

        public override void CleanUp()
        {
            // Virtual camera
            depthCamera?.CleanUp();

            if (!Ros2cs.Ok())
                return;

            // Unsubscribe from ROS topics
            if (rgbSubscriber != null)
            {
                owner.ROSNode().RemoveSubscription<Image>(rgbSubscriber);
                rgbSubscriber = null;
            }

            if (depthSubscriber != null)
            {
                owner.ROSNode().RemoveSubscription<Image>(depthSubscriber);
                depthSubscriber = null;
            }

            // Remove publisher
            if (mrFramePublisher != null)
            {
                owner.ROSNode().RemovePublisher<Image>(mrFramePublisher);
                mrFramePublisher = null;
            }
        }
    }
}
