using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Threading;

namespace CAVAS.UB_MR.Telemetry
{
    public interface IMonotonicClock { double Seconds { get; } }

    public sealed class MonotonicClock : IMonotonicClock
    {
        public static readonly MonotonicClock Instance = new();
        public double Seconds => (double)Stopwatch.GetTimestamp() / Stopwatch.Frequency;
    }

    public readonly struct TimingSnapshot
    {
        public readonly double? LatestMs, AverageMs;
        public readonly double SampleTime;
        public readonly bool Available, Inactive;
        internal TimingSnapshot(double? latest, double? average, double sampledAt, double now)
        {
            LatestMs = latest; AverageMs = average; SampleTime = sampledAt;
            Available = latest.HasValue; Inactive = !Available || now - sampledAt > 2;
        }
    }

    // Callers serialize access so a snapshot and a reset are atomic with connection state.
    internal sealed class TimingWindow
    {
        readonly Queue<(double time, double milliseconds)> samples = new();
        double sum, sampledAt;
        double? latest;
        void Prune(double now)
        {
            while (samples.Count > 0 && samples.Peek().time <= now - 10)
                sum -= samples.Dequeue().milliseconds;
            if (samples.Count == 0) sum = 0;
        }
        public void Record(double milliseconds, double now)
        {
            if (double.IsNaN(milliseconds) || double.IsInfinity(milliseconds) || milliseconds < 0) return;
            Prune(now);
            samples.Enqueue((now, milliseconds)); sum += milliseconds;
            latest = milliseconds; sampledAt = now;
        }
        public TimingSnapshot Snapshot(double now)
        {
            Prune(now);
            return new TimingSnapshot(latest, samples.Count == 0 ? null : sum / samples.Count, sampledAt, now);
        }
        public void Clear() { samples.Clear(); sum = 0; latest = null; sampledAt = 0; }
    }

    public readonly struct TrafficSnapshot
    {
        public readonly long ReceivedBytes, SentBytes;
        public readonly double ReceivedBytesPerSecond, SentBytesPerSecond, SampleTime;
        public readonly bool Available;
        internal TrafficSnapshot(long rx, long tx, double rxRate, double txRate, double time, bool available)
        {
            ReceivedBytes = rx; SentBytes = tx; ReceivedBytesPerSecond = rxRate;
            SentBytesPerSecond = txRate; SampleTime = time; Available = available;
        }
    }

    public sealed class TrafficMeter
    {
        readonly object gate = new();
        readonly IMonotonicClock clock;
        long received, sent, previousReceived, previousSent;
        double previousTime, rxRate, txRate;
        bool sampled;
        int sources;
        public TrafficMeter(IMonotonicClock clock = null)
        {
            this.clock = clock ?? MonotonicClock.Instance;
            previousTime = this.clock.Seconds;
        }
        internal void AddReceived(long bytes) { if (bytes > 0) Interlocked.Add(ref received, bytes); }
        internal void AddSent(long bytes) { if (bytes > 0) Interlocked.Add(ref sent, bytes); }
        internal IDisposable RegisterSource()
        {
            lock (gate)
            {
                if (sources++ == 0)
                {
                    previousTime = clock.Seconds;
                    previousReceived = Interlocked.Read(ref received); previousSent = Interlocked.Read(ref sent);
                    sampled = false; rxRate = txRate = 0;
                }
            }
            return new Registration(() => { lock (gate) sources--; });
        }
        public TrafficSnapshot Snapshot()
        {
            lock (gate)
            {
                double now = clock.Seconds;
                long rx = Interlocked.Read(ref received), tx = Interlocked.Read(ref sent);
                double elapsed = now - previousTime;
                if (elapsed >= 1)
                {
                    rxRate = (rx - previousReceived) / elapsed; txRate = (tx - previousSent) / elapsed;
                    previousReceived = rx; previousSent = tx; previousTime = now; sampled = true;
                }
                return new TrafficSnapshot(rx, tx, rxRate, txRate, previousTime, sources > 0 && sampled);
            }
        }
    }

    internal sealed class Registration : IDisposable
    {
        Action release;
        public Registration(Action release) => this.release = release;
        public void Dispose() => Interlocked.Exchange(ref release, null)?.Invoke();
    }

    public readonly struct SensorSnapshot
    {
        public readonly string Name;
        public readonly bool Bypass;
        public readonly TimingSnapshot Processing, ReceiveToPublish;
        internal SensorSnapshot(string name, bool bypass, TimingSnapshot processing, TimingSnapshot endToEnd)
        { Name = name; Bypass = bypass; Processing = processing; ReceiveToPublish = endToEnd; }
    }

    public sealed class SensorTelemetry : IDisposable
    {
        readonly object gate = new();
        readonly IMonotonicClock clock;
        readonly string name;
        readonly TimingWindow processing = new(), endToEnd = new();
        readonly IDisposable registration;
        bool bypass, disposed;
        internal SensorTelemetry(string name, bool bypass, IMonotonicClock clock, IDisposable registration)
        { this.name = name; this.bypass = bypass; this.clock = clock; this.registration = registration; }
        internal void RecordProcessing(double startedAt)
        {
            lock (gate)
            {
                if (disposed) return;
                double now = clock.Seconds;
                bypass = false; processing.Record((now - startedAt) * 1000, now);
            }
        }
        internal void RecordPublished(double receivedAt, bool wasBypassed)
        {
            lock (gate)
            {
                if (disposed) return;
                double now = clock.Seconds;
                bypass = wasBypassed; endToEnd.Record((now - receivedAt) * 1000, now);
            }
        }
        public SensorSnapshot Snapshot()
        {
            lock (gate)
            {
                double now = clock.Seconds;
                return new SensorSnapshot(name, bypass, processing.Snapshot(now), endToEnd.Snapshot(now));
            }
        }
        public void Dispose()
        {
            lock (gate) { if (disposed) return; disposed = true; }
            registration.Dispose();
        }
    }

    public readonly struct RedisSnapshot
    {
        public readonly TrafficSnapshot Traffic;
        public readonly TimingSnapshot RoundTrip;
        public readonly bool Connected;
        internal RedisSnapshot(TrafficSnapshot traffic, TimingSnapshot roundTrip, bool connected)
        { Traffic = traffic; RoundTrip = roundTrip; Connected = connected; }
    }

    public sealed class RedisTelemetry : IDisposable
    {
        readonly object gate = new();
        readonly IMonotonicClock clock;
        readonly TimingWindow roundTrip = new();
        readonly IDisposable source;
        public TrafficMeter Traffic { get; }
        bool connected;
        int generation;
        public RedisTelemetry(IMonotonicClock clock = null)
        {
            this.clock = clock ?? MonotonicClock.Instance;
            Traffic = new TrafficMeter(this.clock); source = Traffic.RegisterSource();
        }
        internal int SetConnected(bool value)
        {
            lock (gate)
            {
                if (connected != value) { generation++; roundTrip.Clear(); connected = value; }
                return generation;
            }
        }
        internal void RecordRoundTrip(double startedAt, int connectionGeneration)
        {
            lock (gate)
            {
                if (!connected || generation != connectionGeneration) return;
                double now = clock.Seconds; roundTrip.Record((now - startedAt) * 1000, now);
            }
        }
        public RedisSnapshot Snapshot()
        {
            lock (gate) return new RedisSnapshot(Traffic.Snapshot(), roundTrip.Snapshot(clock.Seconds), connected);
        }
        public void Dispose() { SetConnected(false); source.Dispose(); }
    }

    public sealed class ResourceTelemetry : IDisposable
    {
        readonly object gate = new();
        readonly List<SensorTelemetry> sensors = new();
        bool disposed;
        public IMonotonicClock Clock { get; }
        public TrafficMeter SensorPayload { get; }
        public ResourceTelemetry(IMonotonicClock clock = null)
        { Clock = clock ?? MonotonicClock.Instance; SensorPayload = new TrafficMeter(Clock); }
        internal SensorTelemetry RegisterLidar(string name, bool bypass = false)
        {
            lock (gate)
            {
                if (disposed) return null;
                IDisposable source = SensorPayload.RegisterSource();
                SensorTelemetry sensor = null;
                sensor = new SensorTelemetry(name, bypass, Clock, new Registration(() =>
                { lock (gate) sensors.Remove(sensor); source.Dispose(); }));
                sensors.Add(sensor); return sensor;
            }
        }
        public ReadOnlyCollection<SensorSnapshot> SensorSnapshots()
        {
            SensorTelemetry[] active;
            lock (gate) active = sensors.ToArray();
            var snapshots = new SensorSnapshot[active.Length];
            for (int i = 0; i < active.Length; i++) snapshots[i] = active[i].Snapshot();
            return Array.AsReadOnly(snapshots);
        }
        public void Dispose()
        {
            SensorTelemetry[] active;
            lock (gate) { if (disposed) return; disposed = true; active = sensors.ToArray(); sensors.Clear(); }
            foreach (var sensor in active) sensor.Dispose();
        }
    }

    // The timestamp belongs to the message, even when a newer message replaces it in a queue.
    internal sealed class TimedMessage<T> where T : class
    {
        public readonly T Message;
        public readonly double ReceivedAt;
        public readonly bool Bypass;
        public TimedMessage(T message, double receivedAt, bool bypass = false)
        { Message = message; ReceivedAt = receivedAt; Bypass = bypass; }
    }
}
