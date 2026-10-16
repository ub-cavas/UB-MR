using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Xml;
using System.Xml.Linq;

namespace CAVAS.UB_MR.Telemetry
{
    public sealed class GpuDeviceSnapshot
    {
        public string Id { get; }
        public string Name { get; }
        public double? TotalMiB { get; }
        // Occupied framebuffer memory, including separately reported driver reservations.
        public double? UsedMiB { get; }
        public double? ProcessMiB { get; }
        public double? OtherMiB => UsedMiB.HasValue && ProcessMiB.HasValue
            ? Math.Max(0, UsedMiB.Value - ProcessMiB.Value) : null;
        public GpuDeviceSnapshot(string id, string name, double? total, double? used, double? process)
        { Id = id; Name = name; TotalMiB = total; UsedMiB = used; ProcessMiB = process; }
    }

    public sealed class GpuSnapshot
    {
        public ReadOnlyCollection<GpuDeviceSnapshot> Devices { get; }
        public double SampleTime { get; }
        public string UnavailableReason { get; }
        public bool Available => UnavailableReason == null && Devices.Count > 0;
        public GpuSnapshot(IEnumerable<GpuDeviceSnapshot> devices, double time, string unavailableReason = null)
        {
            Devices = new List<GpuDeviceSnapshot>(devices).AsReadOnly();
            SampleTime = time; UnavailableReason = unavailableReason;
        }
        public static GpuSnapshot Unavailable(double time, string reason) => new(Array.Empty<GpuDeviceSnapshot>(), time, reason);
    }

    public interface IGpuProvider
    {
        Task<GpuSnapshot> SampleAsync(CancellationToken cancellation);
    }

    internal static class NvidiaSmiXml
    {
        static double? Memory(XElement element)
        {
            string[] parts = ((string)element ?? "").Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != 2 || parts[1] != "MiB" ||
                !double.TryParse(parts[0], NumberStyles.Float, CultureInfo.InvariantCulture, out double value) ||
                value < 0 || double.IsNaN(value) || double.IsInfinity(value)) return null;
            return value;
        }
        internal static GpuSnapshot Parse(string xml, int? processId, double now, int? localProcessId = null)
        {
            // nvidia-smi emits a DOCTYPE; do not resolve it or any external entity.
            using var reader = XmlReader.Create(new StringReader(xml), new XmlReaderSettings
                { DtdProcessing = DtdProcessing.Ignore, XmlResolver = null, MaxCharactersInDocument = 8 * 1024 * 1024 });
            var document = XDocument.Load(reader);
            if (document.Root?.Name != "nvidia_smi_log") throw new FormatException("Unexpected GPU report.");
            if (localProcessId.HasValue)
                processId = GpuProcessIdentity.ForDriver((string)document.Root.Element("driver_version"), localProcessId.Value, processId);
            var devices = new List<GpuDeviceSnapshot>();
            foreach (var gpu in document.Root.Elements("gpu"))
            {
                var memory = gpu.Element("fb_memory_usage");
                double? used = Memory(memory?.Element("used"));
                // Recent drivers report reserved memory separately from used. It is not free capacity.
                if (memory?.Element("reserved") != null) used += Memory(memory.Element("reserved"));
                var processes = gpu.Element("processes");
                double? processMemory = null;
                bool mig = (string)gpu.Element("mig_mode")?.Element("current_mig") == "Enabled";
                if (processId.HasValue && processes != null && !mig && processes.Value.Trim() != "N/A")
                {
                    processMemory = 0;
                    // C, G, and C+G are all relevant. Never add a process twice on one device.
                    foreach (var info in processes.Elements("process_info"))
                    {
                        if (int.TryParse((string)info.Element("pid"), out int pid) && pid == processId.Value)
                        {
                            var value = Memory(info.Element("used_memory"));
                            if (!value.HasValue) { processMemory = null; break; }
                            processMemory = Math.Max(processMemory.Value, value.Value);
                        }
                    }
                }
                devices.Add(new GpuDeviceSnapshot((string)gpu.Element("uuid") ?? (string)gpu.Attribute("id") ?? "GPU",
                    (string)gpu.Element("product_name") ?? "NVIDIA GPU", Memory(memory?.Element("total")),
                    used, processMemory));
            }
            return devices.Count == 0 ? GpuSnapshot.Unavailable(now, "No NVIDIA GPUs reported") : new GpuSnapshot(devices, now);
        }
    }

    internal static class GpuProcessIdentity
    {
        // R555 introduced namespace-aware process reporting. Never guess by matching an unrelated
        // process whose host PID happens to equal Unity's container-local PID.
        internal static int? ForDriver(string driverVersion, int localPid, int? hostPid)
        {
            if (hostPid == localPid) return localPid;
            string major = (driverVersion ?? "").Split('.')[0];
            if (!int.TryParse(major, out int version) || version <= 0) return null;
            return version >= 555 ? localPid : hostPid;
        }
        internal static int? ReadTgid(string status)
        {
            foreach (string line in status.Split('\n'))
                if (line.StartsWith("Tgid:", StringComparison.Ordinal) && int.TryParse(line.Substring(5).Trim(), out int pid) && pid > 0)
                    return pid;
            return null;
        }
        internal static int? Resolve()
        {
            try
            {
                if (File.Exists("/host/proc/self/status")) return ReadTgid(File.ReadAllText("/host/proc/self/status"));
                // Do not accidentally attribute an unrelated host process with the same container PID.
                if (File.Exists("/.dockerenv") || File.Exists("/run/.containerenv") ||
                    !string.IsNullOrEmpty(Environment.GetEnvironmentVariable("UB_MR_PLAYER_DIR"))) return null;
                return ReadTgid(File.ReadAllText("/proc/self/status"));
            }
            catch (IOException) { return null; }
            catch (UnauthorizedAccessException) { return null; }
        }
    }

    // Kept separate from XML parsing so timeout and cancellation can be checked without a GPU.
    internal static class GpuCommand
    {
        internal static async Task<string> RunAsync(string executable, string arguments, CancellationToken cancellation,
            int timeoutMilliseconds = 2000)
        {
            using var process = new Process { StartInfo = new ProcessStartInfo(executable, arguments)
            {
                UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true, RedirectStandardError = true
            } };
            cancellation.ThrowIfCancellationRequested();
            process.Start();
            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellation);
            timeout.CancelAfter(timeoutMilliseconds);
            Task<string> stdout = process.StandardOutput.ReadToEndAsync();
            Task<string> stderr = process.StandardError.ReadToEndAsync();
            try
            {
                while (!process.HasExited) await Task.Delay(25, timeout.Token).ConfigureAwait(false);
                string result = await stdout.ConfigureAwait(false);
                await stderr.ConfigureAwait(false);
                if (process.ExitCode != 0) throw new IOException("NVIDIA driver unavailable");
                return result;
            }
            catch (OperationCanceledException)
            {
                if (cancellation.IsCancellationRequested) throw;
                throw new TimeoutException("GPU query timed out");
            }
            finally
            {
                try { if (!process.HasExited) process.Kill(); }
                catch (InvalidOperationException) { }
                // Drain redirected pipes after termination; no orphaned reader tasks.
                await Task.WhenAll(stdout, stderr).ConfigureAwait(false);
            }
        }
    }

    public sealed class NvidiaSmiProvider : IGpuProvider
    {
        readonly IMonotonicClock clock;
        readonly bool supported;
        readonly int? processId;
        readonly int localProcessId;
        public NvidiaSmiProvider(bool supported, IMonotonicClock clock = null)
        {
            this.supported = supported; this.clock = clock ?? MonotonicClock.Instance;
            if (supported)
            {
                processId = GpuProcessIdentity.Resolve();
                using var current = Process.GetCurrentProcess(); localProcessId = current.Id;
            }
        }
        public async Task<GpuSnapshot> SampleAsync(CancellationToken cancellation)
        {
            if (!supported) return GpuSnapshot.Unavailable(clock.Seconds, "Requires Linux and NVIDIA");
            try
            {
                string xml = await GpuCommand.RunAsync("nvidia-smi", "-q -x", cancellation).ConfigureAwait(false);
                return NvidiaSmiXml.Parse(xml, processId, clock.Seconds, localProcessId);
            }
            catch (OperationCanceledException) { throw; }
            catch (TimeoutException) { return GpuSnapshot.Unavailable(clock.Seconds, "GPU query timed out"); }
            catch (Exception error) when (error is IOException || error is System.ComponentModel.Win32Exception ||
                error is XmlException || error is FormatException || error is UnauthorizedAccessException)
            { return GpuSnapshot.Unavailable(clock.Seconds, "NVIDIA metrics unavailable"); }
        }
    }

    public sealed class GpuMonitor : IDisposable
    {
        readonly CancellationTokenSource cancellation = new();
        readonly IMonotonicClock clock;
        GpuSnapshot snapshot;
        int disposed;
        public Task Completion { get; }
        public GpuSnapshot Snapshot => Volatile.Read(ref snapshot);
        public GpuMonitor(IGpuProvider provider, IMonotonicClock clock = null)
        {
            this.clock = clock ?? MonotonicClock.Instance;
            snapshot = GpuSnapshot.Unavailable(this.clock.Seconds, "Waiting for GPU sample");
            Completion = Task.Run(async () =>
            {
                try
                {
                    while (!cancellation.IsCancellationRequested)
                    {
                        double startedAt = this.clock.Seconds;
                        var sample = await provider.SampleAsync(cancellation.Token).ConfigureAwait(false);
                        Volatile.Write(ref snapshot, sample);
                        int delay = (int)Math.Max(1, (1 - (this.clock.Seconds - startedAt)) * 1000);
                        await Task.Delay(delay, cancellation.Token).ConfigureAwait(false);
                    }
                }
                catch (OperationCanceledException) when (cancellation.IsCancellationRequested) { }
                catch (Exception)
                { Volatile.Write(ref snapshot, GpuSnapshot.Unavailable(this.clock.Seconds, "GPU monitoring stopped")); }
                finally { cancellation.Dispose(); }
            });
        }
        public void Dispose()
        {
            if (Interlocked.Exchange(ref disposed, 1) != 0) return;
            try { cancellation.Cancel(); } catch (ObjectDisposedException) { }
        }
    }
}
