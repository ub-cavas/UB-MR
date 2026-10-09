using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Reflection;
using System.Runtime.Serialization;
using CAVAS.UB_MR.DT.Sensors.Lidar;
using CAVAS.UB_MR.Telemetry;
using ROS2;
using sensor_msgs.msg;

namespace CAVAS.UB_MR.Tests
{
    public static class ResourceLidarChecks
    {
        sealed class Publisher : IPublisher<PointCloud2>
        {
            public string Topic => "/test/modified";
            public bool IsDisposed { get; private set; }
            public PointCloud2 Message;
            public bool Fail;
            public void Publish(PointCloud2 message)
            { if (Fail) throw new InvalidOperationException("Injected publication failure"); Message = message; }
            public void Dispose() => IsDisposed = true;
        }
        static void Set(object instance, string name, object value) => instance.GetType()
            .GetField(name, BindingFlags.NonPublic | BindingFlags.Instance).SetValue(instance, value);
        static void Receive(LidarModifier modifier, PointCloud2 message) => typeof(LidarModifier)
            .GetMethod("Receive", BindingFlags.NonPublic | BindingFlags.Instance).Invoke(modifier, new object[] { message });
        static void Check(bool condition, string message) => ResourceTelemetryChecks.Check(condition, message);
        static PointCloud2 Scan(byte value) => new()
        {
            Width = 1, Height = 1, Point_step = 16, Row_step = 16, Is_bigendian = true,
            Fields = new[]
            {
                new PointField { Name = "x", Offset = 0, Datatype = PointField.FLOAT32, Count = 1 },
                new PointField { Name = "y", Offset = 4, Datatype = PointField.FLOAT32, Count = 1 },
                new PointField { Name = "z", Offset = 8, Datatype = PointField.FLOAT32, Count = 1 }
            },
            Data = new byte[] { value, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 8, 9, 10 }
        };

        public static void Run()
        {
            var clock = new ResourceTelemetryChecks.Clock();
            using var telemetry = new ResourceTelemetry(clock);
            using var timing = telemetry.RegisterLidar("test");
            var publisher = new Publisher();
            // Supply the managed pipeline state without creating a ROS node or GPU resources.
            var modifier = (LidarModifier)FormatterServices.GetUninitializedObject(typeof(LidarModifier));
            Set(modifier, "telemetry", telemetry); Set(modifier, "timing", timing); Set(modifier, "clock", clock);
            Set(modifier, "mInput_PCD_Queue", new ConcurrentQueue<TimedMessage<PointCloud2>>());
            Set(modifier, "mOutput_PCD_Queue", new ConcurrentQueue<TimedMessage<PointCloud2>>());
            Set(modifier, "mActiveSDFs", new List<SDFTexture>()); Set(modifier, "mSDFs", Array.Empty<SDFTexture>());
            Set(modifier, "mPointCloudPublisher", publisher);
            Check(!modifier.TryModify(null), "Empty queue processed a scan");
            clock.Now = 1; Receive(modifier, Scan(1));
            clock.Now = 2; Receive(modifier, Scan(2));
            clock.Now = 3; var newest = Scan(3); byte[] originalBytes = (byte[])newest.Data.Clone(); Receive(modifier, newest);
            clock.Now = 4; Check(modifier.TryModify(null), "No-SDF bypass failed");
            clock.Now = 4.025; modifier.Publish();
            var sample = timing.Snapshot();
            Check(ReferenceEquals(newest, publisher.Message), "Bypass replaced the original ROS message");
            for (int i = 0; i < originalBytes.Length; i++) Check(originalBytes[i] == newest.Data[i], "Bypass altered payload bytes");
            Check(Math.Abs(sample.ReceiveToPublish.LatestMs.Value - 1025) < .0001, "Newest scan retained an older receipt timestamp");
            Check(sample.Bypass && !sample.Processing.Available, "Bypass added a processing sample");
            Check(telemetry.SensorPayload.Snapshot().ReceivedBytes == 48 && telemetry.SensorPayload.Snapshot().SentBytes == 16,
                "Discarded input bytes or successful publication bytes counted incorrectly");
            clock.Now = 5; Receive(modifier, Scan(5)); Check(modifier.TryModify(null), "Second bypass failed");
            publisher.Fail = true;
            try { modifier.Publish(); throw new Exception("Publication failure was swallowed"); }
            catch (InvalidOperationException) { }
            Check(timing.Snapshot().ReceiveToPublish.SampleTime == sample.ReceiveToPublish.SampleTime,
                "Failed publication created a latency sample");
            Check(telemetry.SensorPayload.Snapshot().SentBytes == 16, "Failed publication counted outgoing bytes");
            var invalid = Scan(6); invalid.Fields[2].Offset = 16;
            Receive(modifier, invalid); Check(!modifier.TryModify(null), "Out-of-bounds XYZ offset processed");
            invalid = Scan(7); invalid.Row_step = 8;
            Check(!PointCloudValidation.HasValidPayload(invalid), "Short row accepted");
            invalid = Scan(8); invalid.Data = new byte[4];
            Receive(modifier, invalid); Check(!modifier.TryModify(null), "Truncated scan processed");
            invalid = Scan(9); invalid.Width = uint.MaxValue;
            Check(!PointCloudValidation.HasCoordinates(invalid), "Overflowing scan dimensions accepted");
            Check(timing.Snapshot().ReceiveToPublish.SampleTime == sample.ReceiveToPublish.SampleTime,
                "Invalid scan altered timing history");
        }
    }
}
