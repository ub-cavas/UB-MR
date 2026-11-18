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
    }
}
