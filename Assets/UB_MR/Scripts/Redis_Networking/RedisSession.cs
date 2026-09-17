using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using Newtonsoft.Json.Linq;

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
            var wire = new RedisWire();
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
                    subscribed = true;
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
                    subscribed = false;
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
                    wire.Write("PING");
                    if (!Equals(wire.Read(), "PONG")) throw new IOException("Server did not respond.");
                    publishing = true;
                    while (!cancellation.IsCancellationRequested)
                    {
                        int signal = WaitHandle.WaitAny(signals, 1000);
                        if (signal == 0) return;
                        PendingPose pose = Interlocked.Exchange(ref pendingPose, null);
                        if (pose != null && unchecked((uint)(Environment.TickCount - pose.created)) < 500 && subscribed)
                        {
                            wire.Write("PUBLISH", settings.channel, pose.json);
                            if (wire.Read() is not long) throw new IOException("Publication failed.");
                        }
                        else
                        {
                            wire.Write("PING");
                            if (!Equals(wire.Read(), "PONG")) throw new IOException("Server did not respond.");
                        }
                    }
                }
                catch (Exception error) { RecordFailure(error); }
                finally
                {
                    publishing = false;
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

        public void Dispose()
        {
            if (Interlocked.Exchange(ref disposed, 1) != 0) return;
            cancellation.Cancel();
            subscribed = publishing = false;
            lock (socketLock) { subscriber?.Dispose(); publisher?.Dispose(); }
            Interlocked.Exchange(ref pendingPose, null);
            Interlocked.Exchange(ref traffic, null);
            _ = Completion.ContinueWith(_ => { publishReady.Dispose(); cancellation.Dispose(); });
        }
    }
}
