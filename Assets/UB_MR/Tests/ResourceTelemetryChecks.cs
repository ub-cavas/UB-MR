using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using CAVAS.UB_MR.Telemetry;

namespace CAVAS.UB_MR.Tests
{
    // No Unity or GPU dependency: also runnable in a standalone .NET harness.
    public static class ResourceTelemetryChecks
    {
        internal sealed class Clock : IMonotonicClock { public double Now; public double Seconds => Now; }
        internal static void Check(bool condition, string message)
        { if (!condition) throw new Exception(message); }
        static void Near(double expected, double? actual, string message)
            => Check(actual.HasValue && Math.Abs(expected - actual.Value) < .00001, message + $": {actual} != {expected}");

        public static async Task Run()
        {
            RollingTimings(); TrafficRates(); ConnectionGenerations(); GpuReports();
            await GpuWorker().ConfigureAwait(false);
            if (System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Linux))
                await QueryCancellation().ConfigureAwait(false);
        }

        static void RollingTimings()
        {
            var clock = new Clock();
            using var collector = new ResourceTelemetry(clock);
            using var sensor = collector.RegisterLidar("front");
            Check(!sensor.Snapshot().Processing.Available, "New sensor has a fabricated timing.");
            clock.Now = 1; sensor.RecordProcessing(.990); sensor.RecordPublished(.900, false);
            clock.Now = 2; sensor.RecordProcessing(1.970); sensor.RecordPublished(1.950, false);
            var sample = sensor.Snapshot();
            Near(30, sample.Processing.LatestMs, "Latest processing"); Near(20, sample.Processing.AverageMs, "Processing mean");
            Near(75, sample.ReceiveToPublish.AverageMs, "Receive-to-publish mean");
            clock.Now = 3; sensor.RecordPublished(2.999, true);
            Check(sensor.Snapshot().Bypass, "Bypass state missing");
            Near(20, sensor.Snapshot().Processing.AverageMs, "Bypass diluted processing average");
            clock.Now = 5.1; Check(sensor.Snapshot().ReceiveToPublish.Inactive, "Old timing not marked inactive");
            clock.Now = 11; Near(30, sensor.Snapshot().Processing.AverageMs, "Window boundary did not expire oldest sample");
            clock.Now = 12; Check(!sensor.Snapshot().Processing.AverageMs.HasValue, "Empty window retained average");
            Near(30, sensor.Snapshot().Processing.LatestMs, "Latest timing was lost after window expired");
            sensor.Dispose(); sensor.Dispose();
            Check(collector.SensorSnapshots().Count == 0, "Disposed sensor stayed registered");
            sensor.RecordProcessing(11); Check(collector.SensorSnapshots().Count == 0, "Late callback revived sensor");
            using var passthrough = collector.RegisterLidar("rear", true);
            Check(passthrough.Snapshot().Bypass && !passthrough.Snapshot().Processing.Available, "Passthrough created processing sample");
        }

        static void TrafficRates()
        {
            var clock = new Clock(); var meter = new TrafficMeter(clock);
            Check(!meter.Snapshot().Available, "Unregistered traffic should be unavailable");
            using var source = meter.RegisterSource();
            meter.AddReceived(1024); meter.AddSent(512); clock.Now = .5;
            Check(!meter.Snapshot().Available, "Partial first interval was reported");
            clock.Now = 1; var sample = meter.Snapshot();
            Near(1024, sample.ReceivedBytesPerSecond, "RX rate"); Near(512, sample.SentBytesPerSecond, "TX rate");
            Parallel.For(0, 10000, _ => { meter.AddReceived(2); meter.AddSent(3); });
            clock.Now = 3; sample = meter.Snapshot();
            Near(10000, sample.ReceivedBytesPerSecond, "Concurrent RX / elapsed time");
            Near(15000, sample.SentBytesPerSecond, "Concurrent TX / elapsed time");
            Check(sample.ReceivedBytes == 21024 && sample.SentBytes == 30512, "Concurrent counters lost bytes");
            clock.Now = 4; sample = meter.Snapshot(); Near(0, sample.ReceivedBytesPerSecond, "Idle rate did not decay");
            source.Dispose(); Check(!meter.Snapshot().Available, "Removed source stayed available");
            using var replacement = meter.RegisterSource();
            Check(!meter.Snapshot().Available, "New source reused old rate");
        }

        static void ConnectionGenerations()
        {
            var clock = new Clock(); using var redis = new RedisTelemetry(clock);
            int first = redis.SetConnected(true); clock.Now = .005; redis.RecordRoundTrip(0, first);
            Near(5, redis.Snapshot().RoundTrip.AverageMs, "RTT mean");
            redis.SetConnected(false);
            Check(!redis.Snapshot().RoundTrip.Available && !redis.Snapshot().Connected, "Disconnect retained RTT");
            int second = redis.SetConnected(true); clock.Now = .020;
            redis.RecordRoundTrip(0, first);
            Check(!redis.Snapshot().RoundTrip.Available, "Old in-flight PONG contaminated new connection");
            redis.RecordRoundTrip(.010, second); Near(10, redis.Snapshot().RoundTrip.AverageMs, "Reconnect did not reset window");
        }

        static void GpuReports()
        {
            const string xml = "<?xml version='1.0'?><!DOCTYPE nvidia_smi_log SYSTEM 'nvsmi_device_v12.dtd'>" +
                "<nvidia_smi_log><gpu id='bus0'><product_name>First GPU</product_name><uuid>GPU-one</uuid>" +
                "<fb_memory_usage><total>8192 MiB</total><used>2048 MiB</used></fb_memory_usage><processes>" +
                "<process_info><pid>42</pid><type>G</type><used_memory>512 MiB</used_memory></process_info>" +
                "<process_info><pid>42</pid><type>C+G</type><used_memory>512 MiB</used_memory></process_info>" +
                "<process_info><pid>99</pid><type>C</type><used_memory>1024 MiB</used_memory></process_info></processes></gpu>" +
                "<gpu id='bus1'><product_name>Second GPU</product_name><fb_memory_usage><total>4096 MiB</total><used>128 MiB</used>" +
                "</fb_memory_usage><processes/></gpu></nvidia_smi_log>";
            var report = NvidiaSmiXml.Parse(xml, 42, 10);
            Check(report.Available && report.Devices.Count == 2 && report.SampleTime == 10, "GPU identities/sample time");
            Near(512, report.Devices[0].ProcessMiB, "Graphics memory double-counted or missed");
            Near(1536, report.Devices[0].OtherMiB, "Other/system allocation");
            Near(0, report.Devices[1].ProcessMiB, "Unused device process memory");
            var reserved = NvidiaSmiXml.Parse(xml.Replace("<used>2048 MiB</used>", "<used>2048 MiB</used><reserved>256 MiB</reserved>"), 42, 10);
            Near(2304, reserved.Devices[0].UsedMiB, "Driver reservation missing from occupied capacity");
            Near(1792, reserved.Devices[0].OtherMiB, "Driver reservation shown as free memory");
            Check(!NvidiaSmiXml.Parse(xml, null, 10).Devices[0].ProcessMiB.HasValue, "Unknown host PID became zero");
            Check(!NvidiaSmiXml.Parse(xml.Replace("512 MiB", "N/A"), 42, 10).Devices[0].OtherMiB.HasValue,
                "Unavailable process memory became other/system zero");
            Check(!NvidiaSmiXml.Parse(xml.Replace("8192 MiB", "N/A"), 42, 10).Devices[0].TotalMiB.HasValue, "Unknown capacity became zero");
            bool failed = false;
            try { NvidiaSmiXml.Parse("<broken", 42, 0); } catch (System.Xml.XmlException) { failed = true; }
            Check(failed, "Malformed XML accepted");
            Check(!NvidiaSmiXml.Parse("<nvidia_smi_log/>", 42, 0).Available, "Empty report available");
            Check(GpuProcessIdentity.ReadTgid("Name:\tUnity\nTgid:\t12345\nPid:\t12399\nNStgid:\t12345\t7\n") == 12345,
                "Used thread/local PID instead of host process ID");
            Check(GpuProcessIdentity.ReadTgid("Tgid:\tN/A") == null, "Invalid host PID accepted");
            Check(GpuProcessIdentity.ForDriver("550.90.07", 1, 42) == 42, "Legacy driver must use host PID");
            Check(GpuProcessIdentity.ForDriver("580.126.09", 1, 42) == 1, "Namespace-aware driver must use local PID");
            Check(GpuProcessIdentity.ForDriver("unknown", 1, 42) == null, "Unknown driver namespace guessed a PID");
            Check(GpuProcessIdentity.ForDriver("550.90.07", 1, null) == null, "Legacy container without host mapping guessed a PID");
            var localReport = NvidiaSmiXml.Parse(xml.Replace("<nvidia_smi_log>",
                "<nvidia_smi_log><driver_version>580.126.09</driver_version>"), 9000, 10, 42);
            Near(512, localReport.Devices[0].ProcessMiB, "Container-local GPU process was missed");
        }

        sealed class Probe : IGpuProvider
        {
            public int Calls, Concurrent, MaximumConcurrent;
            public async Task<GpuSnapshot> SampleAsync(CancellationToken cancellation)
            {
                Interlocked.Increment(ref Calls); int active = Interlocked.Increment(ref Concurrent); MaximumConcurrent = Math.Max(MaximumConcurrent, active);
                try { await Task.Delay(100, cancellation).ConfigureAwait(false); return GpuSnapshot.Unavailable(MonotonicClock.Instance.Seconds, "test"); }
                finally { Interlocked.Decrement(ref Concurrent); }
            }
        }
        static async Task GpuWorker()
        {
            var probe = new Probe(); using var monitor = new GpuMonitor(probe);
            double deadline = MonotonicClock.Instance.Seconds + 5;
            while (Volatile.Read(ref probe.Calls) < 2 && MonotonicClock.Instance.Seconds < deadline)
                await Task.Delay(20).ConfigureAwait(false);
            monitor.Dispose();
            await monitor.Completion.ConfigureAwait(false);
            Check(probe.Calls == 2 && probe.MaximumConcurrent == 1 && probe.Concurrent == 0, "GPU worker overlapped or failed to stop");
            var unsupported = await new NvidiaSmiProvider(false).SampleAsync(CancellationToken.None).ConfigureAwait(false);
            Check(!unsupported.Available, "Unsupported provider reported success");
        }
        static async Task QueryCancellation()
        {
            var watch = Stopwatch.StartNew(); bool timedOut = false;
            try { await GpuCommand.RunAsync("/bin/sleep", "5", CancellationToken.None, 60).ConfigureAwait(false); }
            catch (TimeoutException) { timedOut = true; }
            Check(timedOut && watch.ElapsedMilliseconds < 1500, "GPU subprocess timeout did not terminate promptly");
            using var cancel = new CancellationTokenSource(60); bool cancelled = false;
            try { await GpuCommand.RunAsync("/bin/sleep", "5", cancel.Token).ConfigureAwait(false); }
            catch (OperationCanceledException) { cancelled = true; }
            Check(cancelled, "GPU subprocess cancellation ignored");
        }
    }
}
