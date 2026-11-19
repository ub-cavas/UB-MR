using System;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using CAVAS.UB_MR.Telemetry;

namespace CAVAS.UB_MR.Tests
{
    public static class ExperimentLogChecks
    {
        public static void Run()
        {
            ConcurrentRecording();
            foreach (bool shortSession in new[] { false, true })
                foreach (int shutdownOrder in new[] { 0, 1, 2 })
                    FinalThroughput(shortSession, shutdownOrder);
            ZeroDurationThroughput();
        }

        static void ConcurrentRecording()
        {
            string directory = Path.Combine(Path.GetTempPath(), "ubmr-recording-check-" + Guid.NewGuid().ToString("N"));
            var clock = new ResourceTelemetryChecks.Clock();
            var previous = ExperimentLog.Active;
            try
            {
                using (var log = new ExperimentLog(directory, clock))
                {
                    ExperimentLog.Active = log;
                    using var resources = new ResourceTelemetry(clock);
                    using var sensor = resources.RegisterLidar("front, \"LiDAR\"");
                    using var redis = new RedisTelemetry(clock);
                    int generation = redis.SetConnected(true);
                    clock.Now = 1;
                    Parallel.For(0, 2000, _ => sensor.RecordPublished(.95, true));
                    redis.RecordRoundTrip(.99, generation);
                    resources.SensorPayload.AddReceived(2048); resources.SensorPayload.AddSent(1024);
                    resources.SensorPayload.Snapshot();
                    redis.Traffic.AddReceived(200); redis.Traffic.AddSent(100);
                    redis.Snapshot();
                    redis.SetConnected(false);
                    redis.SetConnected(true);
                    redis.RecordRoundTrip(0, generation); // Stale response must not reach the CSV.
                    clock.Now = 2;
                    resources.SensorPayload.Snapshot(); // Active but idle = valid zero throughput.
                    sensor.Dispose();
                    sensor.RecordPublished(1, true); // Late callback must not reach the CSV.
                    clock.Now = 3;
                    resources.SensorPayload.Snapshot(); // No registered sensors = unavailable, blank rates.
                    log.Dispose(); log.Dispose();
                    ResourceTelemetryChecks.Check(log.Error == null && log.DroppedRows == 0, "Recording failed or dropped fixture rows");
                }
                string[] rows = File.ReadAllLines(Path.Combine(directory, "metrics.csv"));
                ResourceTelemetryChecks.Check(rows.Count(row => row.Contains(",receive_to_publish,")) == 2000,
                    "Lost/duplicated concurrent samples or recorded a disposed sensor");
                ResourceTelemetryChecks.Check(rows.Count(row => row.Contains(",redis_rtt,")) == 1,
                    "Stale-generation RTT entered log");
                ResourceTelemetryChecks.Check(rows.Any(row => row.Contains("\"front, \"\"LiDAR\"\" [")), "CSV source was not escaped");
                ResourceTelemetryChecks.Check(rows.Any(row => row.EndsWith(",2048,1024,2048,1024,1")), "Payload byte/rate sample missing");
                ResourceTelemetryChecks.Check(rows.Any(row => row.EndsWith(",2048,1024,0,0,1")), "Idle throughput is not zero");
                ResourceTelemetryChecks.Check(rows.Any(row => row.EndsWith(",2048,1024,,,0")), "Unavailable throughput was fabricated");
                ResourceTelemetryChecks.Check(File.ReadAllText(Path.Combine(directory, "recording-status.txt")).Contains("dropped_rows=0"),
                    "Missing shutdown status");
                bool protectedExisting = false;
                try { using var duplicate = new ExperimentLog(directory, clock); }
                catch (IOException) { protectedExisting = true; }
                ResourceTelemetryChecks.Check(protectedExisting && File.ReadAllLines(Path.Combine(directory, "metrics.csv")).Length == rows.Length,
                    "Existing experiment overwritten");
            }
            finally { ExperimentLog.Active = previous; Directory.Delete(directory, true); }
        }

        static void FinalThroughput(bool shortSession, int shutdownOrder)
        {
            string directory = Path.Combine(Path.GetTempPath(), "ubmr-final-throughput-" + Guid.NewGuid().ToString("N"));
            var clock = new ResourceTelemetryChecks.Clock();
            var previous = ExperimentLog.Active;
            try
            {
                using (var log = new ExperimentLog(directory, clock))
                {
                    ExperimentLog.Active = log;
                    using var resources = new ResourceTelemetry(clock);
                    using var firstSensor = resources.RegisterSensor("front");
                    using var lastSensor = resources.RegisterSensor("rear");
                    using var redis = new RedisTelemetry(clock);
                    foreach (var meter in new[] { resources.SensorPayload, redis.Traffic })
                    {
                        meter.AddReceived(100); meter.AddSent(40);
                    }
                    if (!shortSession)
                    {
                        clock.Now = 1;
                        resources.SensorPayload.Snapshot(); redis.Snapshot();
                    }
                    clock.Now += .25;
                    firstSensor.Dispose(); // Remaining sources must keep their original rate interval.
                    foreach (var meter in new[] { resources.SensorPayload, redis.Traffic })
                    {
                        meter.AddReceived(50); meter.AddSent(20);
                    }
                    clock.Now += .25;
                    resources.SensorPayload.Snapshot(); redis.Snapshot(); // Normal polling still waits a full second.
                    if (shutdownOrder == 2)
                        Parallel.Invoke(log.Dispose, redis.Dispose, resources.Dispose);
                    else
                    {
                        if (shutdownOrder == 1) log.Dispose(); // Unity can quit before scene objects are destroyed.
                        redis.Dispose(); resources.Dispose();
                    }
                    clock.Now += .25;
                    log.Dispose(); log.Dispose();
                    ResourceTelemetryChecks.Check(log.Error == null && log.DroppedRows == 0, "Final throughput recording failed");
                }
                string[] rows = File.ReadAllLines(Path.Combine(directory, "metrics.csv"));
                foreach (string scope in new[] { "redis [", "ros_sensor_payload [" })
                {
                    var samples = rows.Where(row => row.Contains(",throughput,\"" + scope)).Select(row => row.Split(',')).ToArray();
                    ResourceTelemetryChecks.Check(samples.Length == (shortSession ? 1 : 2), "Missing or duplicate final throughput sample: " + scope);
                    var last = samples.Last();
                    ResourceTelemetryChecks.Check(last[7] == "150" && last[8] == "60", "Final throughput lost bytes: " + scope);
                    ResourceTelemetryChecks.Check(last[9] == (shortSession ? "300" : "100")
                        && last[10] == (shortSession ? "120" : "40") && last[11] == "1",
                        "Final throughput did not use actual elapsed time: " + scope);
                }
            }
            finally { ExperimentLog.Active = previous; Directory.Delete(directory, true); }
        }

        static void ZeroDurationThroughput()
        {
            string directory = Path.Combine(Path.GetTempPath(), "ubmr-zero-throughput-" + Guid.NewGuid().ToString("N"));
            var previous = ExperimentLog.Active;
            try
            {
                var clock = new ResourceTelemetryChecks.Clock();
                using (var log = new ExperimentLog(directory, clock))
                {
                    ExperimentLog.Active = log;
                    using var redis = new RedisTelemetry(clock);
                    redis.Traffic.AddReceived(25);
                    log.Dispose();
                }
                string[] samples = File.ReadAllLines(Path.Combine(directory, "metrics.csv"))
                    .Where(row => row.Contains(",throughput,")).ToArray();
                ResourceTelemetryChecks.Check(samples.Length == 1 && samples[0].EndsWith(",25,0,,,0"),
                    "Zero-duration shutdown must preserve totals without fabricating rates");
            }
            finally { ExperimentLog.Active = previous; Directory.Delete(directory, true); }
        }
    }
}
