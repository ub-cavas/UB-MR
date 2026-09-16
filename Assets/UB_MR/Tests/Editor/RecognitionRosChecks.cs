using System;
using System.Linq;
using System.Threading;
using CAVAS.UB_MR.DT.Sensors;
using CAVAS.UB_MR.DT.Sensors.Lidar;
using ROS2;
using sensor_msgs.msg;
using UnityEditor;
using UnityEngine;

namespace CAVAS.UB_MR.Tests
{
    public static class RecognitionRosChecks
    {
        public static void Run()
        {
            try
            {
                var core = new ROS2UnityCore();
                if (!core.Ok()) throw new Exception("ROS initialization failed.");
                var node = core.CreateNode("ub_mr_recognition_test");
                var qos = new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);
                var passthrough = new LidarPassthrough("/ub_mr_test/scan", node, qos);
                var publisher = node.CreatePublisher<PointCloud2>("/ub_mr_test/scan");
                PointCloud2 received = null;
                using var delivered = new AutoResetEvent(false);
                var subscription = node.CreateSubscription<PointCloud2>("/ub_mr_test/scan_modified", message =>
                {
                    received = message;
                    delivered.Set();
                });
                var original = new PointCloud2
                {
                    Header = new std_msgs.msg.Header { Frame_id = "test_lidar", Stamp = new builtin_interfaces.msg.Time
                        { Sec = 456, Nanosec = 987654321 } },
                    Height = 2, Width = 2, Point_step = 16, Row_step = 36,
                    Is_bigendian = true, Is_dense = false,
                    Fields = new[] { new PointField { Name = "x", Offset = 0, Datatype = PointField.FLOAT32, Count = 1 },
                        new PointField { Name = "ring", Offset = 12, Datatype = PointField.UINT16, Count = 1 } },
                    Data = Enumerable.Range(0, 72).Select(i => (byte)(i * 3)).ToArray()
                };
                var deadline = DateTime.UtcNow.AddSeconds(10);
                while (DateTime.UtcNow < deadline && received == null)
                {
                    publisher.Publish(original);
                    Thread.Sleep(30);
                    passthrough.Publish();
                    delivered.WaitOne(30);
                }
                if (received == null) throw new Exception("Passthrough DDS delivery timed out.");
                if (!received.Data.SequenceEqual(original.Data) || received.Header.Frame_id != original.Header.Frame_id ||
                    received.Header.Stamp.Sec != original.Header.Stamp.Sec || received.Header.Stamp.Nanosec != original.Header.Stamp.Nanosec ||
                    received.Height != original.Height || received.Width != original.Width || received.Point_step != original.Point_step ||
                    received.Row_step != original.Row_step || received.Is_bigendian != original.Is_bigendian || received.Is_dense != original.Is_dense ||
                    received.Fields.Length != original.Fields.Length || received.Fields[1].Name != "ring" ||
                    received.Fields[1].Offset != 12 || received.Fields[1].Datatype != PointField.UINT16)
                    throw new Exception("Passthrough changed the scan or metadata.");
                passthrough.CleanUp();
                passthrough.CleanUp();
                passthrough.Publish();

                var clock = new DetectionClock(node, true, 30);
                if (clock.TrySample(out _)) throw new Exception("Simulation clock published before receiving time.");
                var clockPublisher = node.CreatePublisher<rosgraph_msgs.msg.Clock>("/clock");
                var tick = new rosgraph_msgs.msg.Clock { Clock_ = new builtin_interfaces.msg.Time { Sec = 10, Nanosec = 123456789 } };
                builtin_interfaces.msg.Time stamp = null;
                deadline = DateTime.UtcNow.AddSeconds(10);
                while (DateTime.UtcNow < deadline && stamp == null)
                {
                    clockPublisher.Publish(tick);
                    Thread.Sleep(30);
                    clock.TrySample(out stamp);
                }
                if (stamp == null || stamp.Sec != 10 || stamp.Nanosec != 123456789)
                    throw new Exception("Simulation clock stamp was not preserved.");
                if (clock.TrySample(out _)) throw new Exception("Paused clock published twice.");
                clock.CleanUp();
                clock.CleanUp();
                node.RemoveSubscription<PointCloud2>(subscription);
                node.RemovePublisher<PointCloud2>(publisher);
                node.RemovePublisher<rosgraph_msgs.msg.Clock>(clockPublisher);
                core.RemoveNode(node);
                Debug.Log("UB-MR ROS checks PASSED: byte-exact PointCloud2 passthrough, simulation clock, idempotent cleanup.");
                EditorApplication.Exit(0);
            }
            catch (Exception exception)
            {
                Debug.LogException(exception);
                EditorApplication.Exit(1);
            }
        }
    }
}
