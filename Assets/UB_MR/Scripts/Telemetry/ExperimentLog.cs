using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;

namespace CAVAS.UB_MR.Telemetry
{
    // Callback threads only enqueue. Disk I/O and periodic flushing belong to the writer.
    public sealed class ExperimentLog : IDisposable
    {
        public static ExperimentLog Active { get; internal set; }
        static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;
        readonly BlockingCollection<string> rows = new(65536);
        readonly Thread worker;
        readonly StreamWriter output;
        readonly IMonotonicClock clock;
        readonly double startedAt;
        readonly DateTime startedUtc;
        long dropped;
        int stopped;
        volatile string error;
        public string DirectoryPath { get; }
        public string Error => error;
        public long DroppedRows => Interlocked.Read(ref dropped);

        // Generic adapter entry points: callers choose a metric and source instead
        // of depending on LiDAR, Redis, Unity, or a particular input device.
        public void RecordTiming(string metric, string source, double startedAt, bool bypass = false)
        {
            double now = clock.Seconds;
            Timing(metric, source, now, (now - startedAt) * 1000, bypass);
        }
        public void RecordTraffic(string source, TrafficSnapshot sample)
            => Traffic(source, clock.Seconds, sample);

        public ExperimentLog(string directory, IMonotonicClock clock = null)
        {
            this.clock = clock ?? MonotonicClock.Instance;
            startedAt = this.clock.Seconds; startedUtc = DateTime.UtcNow;
            DirectoryPath = Path.GetFullPath(directory);
            Directory.CreateDirectory(DirectoryPath);
            // Never overwrite a previous experiment.
            output = new StreamWriter(new FileStream(Path.Combine(DirectoryPath, "metrics.csv"),
                FileMode.CreateNew, FileAccess.Write, FileShare.Read), new UTF8Encoding(false));
            output.WriteLine("elapsed_s,utc,metric,source,duration_ms,bypass,connected,rx_bytes,tx_bytes,rx_Bps,tx_Bps,available");
            output.Flush();
            worker = new Thread(WriteRows) { IsBackground = true, Name = "UB-MR experiment log" };
            worker.Start();
        }

        static string Number(double value) => value.ToString("R", Invariant);
        static string Cell(string value) => "\"" + (value ?? "").Replace("\"", "\"\"") + "\"";
        string Prefix(double now, string metric, string source)
        {
            double elapsed = now - startedAt;
            return Number(elapsed) + "," + startedUtc.AddSeconds(elapsed).ToString("O", Invariant)
                + "," + metric + "," + Cell(source) + ",";
        }
        void Enqueue(string row)
        {
            if (Volatile.Read(ref stopped) != 0 || error != null) return;
            try { if (!rows.TryAdd(row)) Interlocked.Increment(ref dropped); }
            catch (InvalidOperationException) { } // Shutdown raced a callback.
        }
        internal void Timing(string metric, string source, double now, double milliseconds, bool bypass = false)
        {
            if (double.IsNaN(milliseconds) || double.IsInfinity(milliseconds) || milliseconds < 0) return;
            Enqueue(Prefix(now, metric, source) + Number(milliseconds) + "," + (bypass ? "1" : "0") + ",,,,,,1");
        }
        internal void Traffic(string source, double now, TrafficSnapshot sample)
        {
            Enqueue(Prefix(now, "throughput", source) + ",,,"
                + sample.ReceivedBytes.ToString(Invariant) + "," + sample.SentBytes.ToString(Invariant) + ","
                + (sample.Available ? Number(sample.ReceivedBytesPerSecond) : "") + ","
                + (sample.Available ? Number(sample.SentBytesPerSecond) : "") + "," + (sample.Available ? "1" : "0"));
        }
        internal void Connection(string source, double now, bool connected)
            => Enqueue(Prefix(now, "redis_connection", source) + ",," + (connected ? "1" : "0") + ",,,,,");

        void WriteRows()
        {
            try
            {
                double nextFlush = MonotonicClock.Instance.Seconds + 1;
                while (!rows.IsCompleted)
                {
                    if (rows.TryTake(out string row, 250)) output.WriteLine(row);
                    double now = MonotonicClock.Instance.Seconds;
                    if (now >= nextFlush) { output.Flush(); nextFlush = now + 1; }
                }
            }
            catch (Exception failure) { error = failure.ToString(); }
            finally
            {
                try { output.Dispose(); }
                catch (Exception failure) { error = failure.ToString(); }
            }
        }
        public void Dispose()
        {
            if (Interlocked.Exchange(ref stopped, 1) != 0) return;
            rows.CompleteAdding(); worker.Join();
            try
            {
                File.WriteAllText(Path.Combine(DirectoryPath, "recording-status.txt"),
                    "started_utc=" + startedUtc.ToString("O", Invariant) + "\n"
                    + "stopped_utc=" + DateTime.UtcNow.ToString("O", Invariant) + "\n"
                    + "dropped_rows=" + DroppedRows.ToString(Invariant) + "\n"
                    + "error=" + (error ?? "none") + "\n");
            }
            catch (Exception failure) { error = failure.ToString(); }
        }
    }
}
