using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;
using CAVAS.UB_MR.Telemetry;

namespace UB_MR.Redis_Networking
{
    internal sealed class RedisSession : IDisposable
    {
        readonly ServerSettings settings;
        readonly CancellationTokenSource cancellation = new();
        readonly object socketLock = new();
        readonly AutoResetEvent publishReady = new(false);
        RedisWire subscriber, publisher;
        volatile bool subscribed, publishing;
        volatile string failure;
        string traffic;
        PendingPose pendingPose;
        int disposed;
        sealed class PendingPose { public string json; public int created; }
        internal Task Completion { get; }
        internal RedisTelemetry Telemetry { get; } = new();

        public bool Connected => subscribed && publishing;
        public string Status => Connected ? "Connected" : failure ?? "Connecting…";

        public RedisSession(ServerSettings settings)
        {
            this.settings = settings.Copy();
            Completion = Task.WhenAll(Task.Run(ReceiveLoop), Task.Run(PublishLoop));
        }

        public string TakeTraffic() => Interlocked.Exchange(ref traffic, null);

        public void Publish(string json)
        {
            if (!Connected || cancellation.IsCancellationRequested) return;
            Interlocked.Exchange(ref pendingPose, new PendingPose { json = json, created = Environment.TickCount });
            publishReady.Set();
        }

        RedisWire Open(bool receive)
        {
            var wire = new RedisWire(Telemetry.Traffic);
            lock (socketLock)
            {
                if (cancellation.IsCancellationRequested) { wire.Dispose(); throw new OperationCanceledException(); }
                if (receive) subscriber = wire; else publisher = wire;
            }
            wire.Open(settings.host, settings.port, settings.password, cancellation.Token);
            return wire;
        }

        void ReceiveLoop()
        {
            while (!cancellation.IsCancellationRequested)
            {
                try
                {
                    using var wire = Open(true);
                    wire.Write("SUBSCRIBE", settings.channel);
                    var reply = wire.Read() as object[];
                    if (reply == null || reply.Length != 3 || !Equals(reply[0], "subscribe"))
                        throw new IOException("Subscription failed.");
                    SetChannelConnected(true, true);
                    while (!cancellation.IsCancellationRequested)
                    {
                        // Keep an idle subscription healthy even before the traffic publisher starts.
                        if (!wire.WaitForData()) wire.Write("PING");
                        if (wire.Read() is not object[] message || message.Length != 3 ||
                            !Equals(message[0], "message") || !Equals(message[1], settings.channel) ||
                            message[2] is not string json) continue;
                        try
                        {
                            var packet = JObject.Parse(json);
                            if ((int?)packet["type"] == 2 && packet["vehicles"] is JArray)
                                Interlocked.Exchange(ref traffic, json);
                        }
                        catch (Newtonsoft.Json.JsonException) { }
                        catch (FormatException) { }
                        catch (InvalidCastException) { }
                    }
                }
                catch (Exception error) { RecordFailure(error); }
                finally
                {
                    SetChannelConnected(true, false);
                    lock (socketLock) { subscriber?.Dispose(); subscriber = null; }
                    Interlocked.Exchange(ref traffic, null);
                }
                if (cancellation.Token.WaitHandle.WaitOne(1000)) return;
            }
        }

        void PublishLoop()
        {
            WaitHandle[] signals = { cancellation.Token.WaitHandle, publishReady };
            while (!cancellation.IsCancellationRequested)
            {
                try
                {
                    using var wire = Open(false);
                    Ping(wire);
                    SetChannelConnected(false, true);
                    double nextPing = 0;
                    while (!cancellation.IsCancellationRequested)
                    {
                        double now = MonotonicClock.Instance.Seconds;
                        if (now >= nextPing)
                        {
                            Ping(wire);
                            nextPing = MonotonicClock.Instance.Seconds + 1;
                        }
                        int wait = Math.Max(1, (int)Math.Ceiling((nextPing - MonotonicClock.Instance.Seconds) * 1000));
                        int signal = WaitHandle.WaitAny(signals, wait);
                        if (signal == 0) return;
                        PendingPose pose = Interlocked.Exchange(ref pendingPose, null);
                        if (pose != null && unchecked((uint)(Environment.TickCount - pose.created)) < 500 && subscribed)
                        {
                            wire.Write("PUBLISH", settings.channel, pose.json);
                            if (wire.Read() is not long) throw new IOException("Publication failed.");
                        }
                    }
                }
                catch (Exception error) { RecordFailure(error); }
                finally
                {
                    SetChannelConnected(false, false);
                    Interlocked.Exchange(ref pendingPose, null);
                    lock (socketLock) { publisher?.Dispose(); publisher = null; }
                }
                if (cancellation.Token.WaitHandle.WaitOne(1000)) return;
            }
        }

        void RecordFailure(Exception error)
        {
            if (cancellation.IsCancellationRequested) return;
            failure = error is UnauthorizedAccessException ? error.Message : "Unable to reach server. Retrying…";
        }

        void SetChannelConnected(bool receive, bool value)
        {
            lock (socketLock)
            {
                value &= !cancellation.IsCancellationRequested;
                if (receive) subscribed = value; else publishing = value;
                Telemetry.SetConnected(Connected);
            }
        }

        void Ping(RedisWire wire)
        {
            int generation;
            lock (socketLock) generation = Telemetry.SetConnected(Connected);
            double startedAt = MonotonicClock.Instance.Seconds;
            wire.Write("PING");
            if (!Equals(wire.Read(), "PONG")) throw new IOException("Server did not respond.");
            Telemetry.RecordRoundTrip(startedAt, generation);
        }

        public void Dispose()
        {
            if (Interlocked.Exchange(ref disposed, 1) != 0) return;
            cancellation.Cancel();
            lock (socketLock)
            {
                subscribed = publishing = false;
                Telemetry.Dispose();
                subscriber?.Dispose(); publisher?.Dispose();
            }
            Interlocked.Exchange(ref pendingPose, null);
            Interlocked.Exchange(ref traffic, null);
            _ = Completion.ContinueWith(_ => { publishReady.Dispose(); cancellation.Dispose(); });
        }
    }
}
