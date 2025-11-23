using System;
using System.Collections.Generic;
using System.Buffers.Binary;
using ROS2;
using sensor_msgs.msg;
using UnityEngine;
using Unity.Collections;

namespace CAVAS.UB_MR.DT.Sensors.Camera
{
    public class CameraModifier : SensorModifier
    {
        Agent owner;
        UnityEngine.Camera rgbCamera;
        DepthCamera depthCamera;
        RenderTexture virtualColorRT;
        Texture2D virtualColorTex;

        ISubscription<Image> rgbSubscriber;
        ISubscription<Image> depthSubscriber;

        IPublisher<Image> mrFramePublisher;
        Image reusableMrImage;
        byte[] physicalRgbaBuffer;
        float[] physicalDepthBuffer;
        byte[] mixedRgbaBuffer;

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

        void EnsureVirtualResources(int width, int height)
        {
            if (width <= 0 || height <= 0)
                return;

            bool needsResize = virtualColorRT == null || virtualColorRT.width != width || virtualColorRT.height != height;
            if (needsResize)
            {
                if (virtualColorRT != null)
                    virtualColorRT.Release();

                if (virtualColorTex != null)
                    GameObject.Destroy(virtualColorTex);

                virtualColorRT = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
                virtualColorRT.name = $"{rgbCamera.name}_VirtualColorRT";
                virtualColorRT.Create();

                virtualColorTex = new Texture2D(width, height, TextureFormat.RGBA32, false, false);
                rgbCamera.targetTexture = virtualColorRT;
                depthCamera.EnsureResolution(width, height);
            }

            int pixelCount = width * height;
            int rgbaBytes = pixelCount * 4;
            if (physicalRgbaBuffer == null || physicalRgbaBuffer.Length != rgbaBytes)
                physicalRgbaBuffer = new byte[rgbaBytes];

            if (physicalDepthBuffer == null || physicalDepthBuffer.Length != pixelCount)
                physicalDepthBuffer = new float[pixelCount];

            if (mixedRgbaBuffer == null || mixedRgbaBuffer.Length != rgbaBytes)
                mixedRgbaBuffer = new byte[rgbaBytes];

            reusableMrImage ??= new Image();
        }

        void CaptureVirtualFrame(int width, int height, out NativeArray<byte> colorRaw, out float[] depthRaw)
        {
            EnsureVirtualResources(width, height);

            var prevActive = RenderTexture.active;
            if (rgbCamera.targetTexture != virtualColorRT)
                rgbCamera.targetTexture = virtualColorRT;

            rgbCamera.Render();
            RenderTexture.active = virtualColorRT;
            virtualColorTex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            virtualColorTex.Apply(false, false);
            RenderTexture.active = prevActive;

            colorRaw = virtualColorTex.GetRawTextureData<byte>();
            depthRaw = depthCamera.CaptureDepth(width, height);
        }

        bool TryDecodeRosColor(Image src, byte[] dst)
        {
            if (src == null || src.Data == null || dst == null)
                return false;

            int width = (int)src.Width;
            int height = (int)src.Height;
            int pixelCount = width * height;
            if (dst.Length < pixelCount * 4)
                return false;

            string encoding = src.Encoding?.ToLowerInvariant() ?? string.Empty;
            int step = (int)src.Step;
            byte[] data = src.Data;
            int expectedLength = step * height;
            if (data.Length < expectedLength)
                return false;

            int dstStride = width * 4;

            if (encoding == "rgba8")
            {
                if (step == dstStride)
                {
                    Buffer.BlockCopy(data, 0, dst, 0, dstStride * height);
                    return true;
                }

                for (int y = 0; y < height; y++)
                    Buffer.BlockCopy(data, y * step, dst, y * dstStride, dstStride);
                return true;
            }

            if (encoding == "bgra8")
            {
                for (int y = 0, dstRow = 0; y < height; y++, dstRow += dstStride)
                {
                    int srcRow = y * step;
                    for (int x = 0, dstIdx = dstRow, srcIdx = srcRow; x < width; x++, dstIdx += 4, srcIdx += 4)
                    {
                        dst[dstIdx] = data[srcIdx + 2];
                        dst[dstIdx + 1] = data[srcIdx + 1];
                        dst[dstIdx + 2] = data[srcIdx];
                        dst[dstIdx + 3] = data[srcIdx + 3];
                    }
                }
                return true;
            }

            if (encoding == "rgb8" || encoding == "bgr8" || encoding == "mono8")
            {
                bool isBgr = encoding == "bgr8";
                int channels = encoding == "mono8" ? 1 : 3;
                int srcStride = width * channels;

                for (int y = 0, dstRow = 0; y < height; y++, dstRow += dstStride)
                {
                    int srcRow = y * step;
                    for (int x = 0, dstIdx = dstRow, srcIdx = srcRow; x < width; x++, dstIdx += 4, srcIdx += channels)
                    {
                        byte r, g, b;
                        if (channels == 1)
                        {
                            r = g = b = data[srcIdx];
                        }
                        else if (isBgr)
                        {
                            b = data[srcIdx];
                            g = data[srcIdx + 1];
                            r = data[srcIdx + 2];
                        }
                        else
                        {
                            r = data[srcIdx];
                            g = data[srcIdx + 1];
                            b = data[srcIdx + 2];
                        }

                        dst[dstIdx] = r;
                        dst[dstIdx + 1] = g;
                        dst[dstIdx + 2] = b;
                        dst[dstIdx + 3] = 255;
                    }
                }
                return true;
            }

            return false;
        }

        bool TryDecodeRosDepth(Image src, float[] dst)
        {
            if (src == null || src.Data == null || dst == null)
                return false;

            int width = (int)src.Width;
            int height = (int)src.Height;
            int pixelCount = width * height;
            if (dst.Length < pixelCount)
                return false;

            string encoding = src.Encoding?.ToLowerInvariant() ?? string.Empty;
            int step = (int)src.Step;
            byte[] data = src.Data;
            int expected = step * height;
            if (data.Length < expected)
                return false;

            bool bigEndian = src.Is_bigendian != 0;

            if (encoding == "32fc1")
            {
                int bytesPerRow = width * 4;
                if (!bigEndian && step == bytesPerRow)
                {
                    Buffer.BlockCopy(data, 0, dst, 0, bytesPerRow * height);
                    return true;
                }

                int idx = 0;
                for (int y = 0; y < height; y++)
                {
                    int row = y * step;
                    for (int x = 0; x < width; x++, idx++)
                    {
                        int offset = row + x * 4;
                        if (bigEndian)
                        {
                            uint tmp = BinaryPrimitives.ReadUInt32BigEndian(new ReadOnlySpan<byte>(data, offset, 4));
                            dst[idx] = BitConverter.Int32BitsToSingle((int)tmp);
                        }
                        else
                        {
                            dst[idx] = BitConverter.ToSingle(data, offset);
                        }
                    }
                }
                return true;
            }

            if (encoding == "16uc1")
            {
                int idx = 0;
                for (int y = 0; y < height; y++)
                {
                    int row = y * step;
                    for (int x = 0; x < width; x++, idx++)
                    {
                        int offset = row + x * 2;
                        ushort depth = bigEndian
                            ? BinaryPrimitives.ReadUInt16BigEndian(new ReadOnlySpan<byte>(data, offset, 2))
                            : BitConverter.ToUInt16(data, offset);
                        dst[idx] = depth * 0.001f; // convert mm to meters
                    }
                }
                return true;
            }

            return false;
        }

        /// <summary>
        /// Combine virtual RGB + physical RGB/Depth into a mixed reality frame.
        /// </summary>
        Image ComputeMixedRealityFrame(Image physicalRgb, Image physicalDepth)
        {
            if (physicalRgb == null || physicalDepth == null ||
                physicalRgb.Data == null || physicalDepth.Data == null)
                return physicalRgb;

            int width = (int)physicalRgb.Width;
            int height = (int)physicalRgb.Height;
            if (width <= 0 || height <= 0 ||
                physicalDepth.Width != physicalRgb.Width ||
                physicalDepth.Height != physicalRgb.Height)
                return physicalRgb;

            CaptureVirtualFrame(width, height, out var virtualColorRaw, out var virtualDepthRaw);

            if (!TryDecodeRosColor(physicalRgb, physicalRgbaBuffer) ||
                !TryDecodeRosDepth(physicalDepth, physicalDepthBuffer))
                return physicalRgb;

            int pixelCount = width * height;
            const float MaxDepthMeters = 200f; 
            const float DepthEpsilon = 0.02f; // bias toward physical 

            for (int i = 0, dst = 0; i < pixelCount; i++, dst += 4)
            {
                int x = i % width;
                int y = i / width;

                // Physical buffers are interpreted as top-left origin (ROS convention),
                // while Unity render textures are bottom-left. Flip the virtual index
                // so virtual color/depth line up with the physical frame.
                int vIndex = (height - 1 - y) * width + x;

                float pDepth = physicalDepthBuffer[i];
                float vDepth = virtualDepthRaw[vIndex];

                bool pValid = pDepth > 0f && !float.IsNaN(pDepth) && pDepth < MaxDepthMeters;
                bool vValid = vDepth > 0f && !float.IsNaN(vDepth) && vDepth < MaxDepthMeters;

                bool useVirtual;
                if (!vValid && !pValid) // No reliable depth info; keep physical pixel.
                    useVirtual = false;
                else if (!vValid) // Only physical depth is valid.
                    useVirtual = false;
                else if (!pValid) // Only virtual depth is valid.
                    useVirtual = true;
                else // Both valid: prefer virtual unless it's clearly behind physical.
                    useVirtual = vDepth <= pDepth + DepthEpsilon;

                if (useVirtual)
                {
                    int src = vIndex * 4;
                    mixedRgbaBuffer[dst] = virtualColorRaw[src];
                    mixedRgbaBuffer[dst + 1] = virtualColorRaw[src + 1];
                    mixedRgbaBuffer[dst + 2] = virtualColorRaw[src + 2];
                    mixedRgbaBuffer[dst + 3] = virtualColorRaw[src + 3];
                }
                else
                {
                    mixedRgbaBuffer[dst] = physicalRgbaBuffer[dst];
                    mixedRgbaBuffer[dst + 1] = physicalRgbaBuffer[dst + 1];
                    mixedRgbaBuffer[dst + 2] = physicalRgbaBuffer[dst + 2];
                    mixedRgbaBuffer[dst + 3] = physicalRgbaBuffer[dst + 3];
                }
            }

            reusableMrImage.Header = physicalRgb.Header;
            reusableMrImage.Height = (uint)height;
            reusableMrImage.Width = (uint)width;
            reusableMrImage.Encoding = "rgba8";
            reusableMrImage.Is_bigendian = 0;
            reusableMrImage.Step = (uint)(width * 4);
            reusableMrImage.Data = mixedRgbaBuffer;

            return reusableMrImage;
        }

        public override void CleanUp()
        {
            // Virtual camera
            if (virtualColorRT != null)
            {
                virtualColorRT.Release();
                virtualColorRT = null;
            }

            if (virtualColorTex != null)
            {
                GameObject.Destroy(virtualColorTex);
                virtualColorTex = null;
            }

            if (rgbCamera != null)
                rgbCamera.targetTexture = null;

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
