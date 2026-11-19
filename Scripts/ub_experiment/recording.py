import csv
import json
import math
import os
import queue
import re
import signal
import socket
import sys
import threading
import time
import uuid
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
from pathlib import Path
from .paths import client_log_directory

FIELDS = "elapsed_s,utc,metric,source,duration_ms,bypass,connected,rx_bytes,tx_bytes,rx_Bps,tx_Bps,available".split(",")
_active = None
_users = 0
_session_lock = threading.RLock()
_signal_handlers = {}


def _install_signals():
    if threading.current_thread() is not threading.main_thread() or _signal_handlers:
        return
    def interrupt(signum, frame):
        # A second stop signal during close must not interrupt the CSV drain.
        if _active is not None:
            raise KeyboardInterrupt
    for signum in (signal.SIGTERM, signal.SIGINT):
        _signal_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, interrupt)


def _restore_signals():
    if threading.current_thread() is threading.main_thread():
        for signum, handler in _signal_handlers.items():
            signal.signal(signum, handler)
        _signal_handlers.clear()


def enabled():
    value = os.environ.get("UB_EXPERIMENT_ENABLED", os.environ.get("UB_MR_METRICS_ENABLED", "0"))
    return value.strip().lower() in ("1", "true", "yes", "on")


def current():
    return _active


class Recorder:
    """Bounded asynchronous CSV writer with monotonic timings and byte counters.

    Timings end at a successful local publish/control-call return. Probe RTTs
    include remote service processing. Byte counters only cover registered scopes.
    """
    def __init__(self, directory=None, *, client_type="client", controller_type="unspecified",
                 client_id=None, plots=True, clock=time.perf_counter, queue_size=65536):
        self.clock = clock
        self.started = clock()
        self.started_utc = datetime.now(timezone.utc)
        self.hostname = os.environ.get("UB_EXPERIMENT_HOST_NAME") or os.environ.get("UB_MR_HOST_NAME") or socket.gethostname()
        self.client_id = client_id or uuid.uuid4().hex
        if directory is None:
            directory = os.environ.get("UB_EXPERIMENT_RUN_DIR")
        if directory is None:
            parent = Path(os.environ.get("UB_EXPERIMENT_DIR") or os.environ.get("UB_MR_METRICS_DIR")
                          or client_log_directory(client_type) / "experiments")
            host = re.sub(r"[^A-Za-z0-9._-]", "_", self.hostname)[:63]
            name = "run-{}-{}-{}".format(self.started_utc.strftime("%Y%m%d-%H%M%SZ"), host, uuid.uuid4().hex[:8])
            directory = parent / name
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self._file = (self.directory / "metrics.csv").open("x", newline="", encoding="utf-8")
        self._csv = csv.writer(self._file)
        self._csv.writerow(FIELDS)
        self._file.flush()
        self.metadata = {"schema_version": 1, "client_type": client_type, "controller_type": controller_type,
                         "client_id": self.client_id, "hostname": self.hostname,
                         "started_utc": self.started_utc.isoformat(), "traffic_scopes": {}}
        self.plots = plots
        self.dropped_rows = 0
        self.error = None
        self.plot_error = None
        self._rows = queue.Queue(maxsize=queue_size)
        self._gate = threading.RLock()
        self._close_gate = threading.Lock()
        self._closed = False
        self._traffic = {}
        self._probes = {}
        self._stop = threading.Event()
        self._writer_stop = threading.Event()
        self._write_metadata()
        self._host_ownership()
        self._writer = threading.Thread(target=self._write, name="experiment-writer", daemon=True)
        self._sampler = threading.Thread(target=self._sample_loop, name="experiment-rates", daemon=True)
        self._writer.start()
        self._sampler.start()
        interface = os.environ.get("UB_EXPERIMENT_INTERFACE")
        self._interface = None
        if interface and interface not in ("auto", "none"):
            try:
                self.monitor_interface(interface)
            except ValueError:
                self.close()
                raise
        print("Experiment metrics: {}".format(self.directory))

    def _write_metadata(self):
        (self.directory / "experiment.json").write_text(json.dumps(self.metadata, indent=2) + "\n", encoding="utf-8")

    def _host_ownership(self):
        uid = os.environ.get("UB_EXPERIMENT_HOST_UID", os.environ.get("UB_MR_HOST_UID"))
        gid = os.environ.get("UB_EXPERIMENT_HOST_GID", os.environ.get("UB_MR_HOST_GID"))
        if not uid or not gid or not hasattr(os, "chown"):
            return
        try:
            for path in [self.directory] + list(self.directory.iterdir()):
                if path.is_symlink():
                    continue
                os.chown(path, int(uid), int(gid))
        except (OSError, ValueError) as exc:
            print("Experiment file ownership: {}".format(exc), file=sys.stderr)

    def _enqueue(self, metric, source, *, now=None, **values):
        now = self.clock() if now is None else now
        elapsed = now - self.started
        row = {"elapsed_s": elapsed, "utc": (self.started_utc + timedelta(seconds=elapsed)).isoformat(),
               "metric": metric, "source": source, **values}
        with self._gate:
            if self._closed or self.error:
                return
            try:
                self._rows.put_nowait([row.get(field, "") for field in FIELDS])
            except queue.Full:
                self.dropped_rows += 1

    def timing(self, metric, source, *, started_at=None, duration_ms=None, bypass=False):
        now = self.clock()
        if duration_ms is None:
            if started_at is None:
                raise ValueError("Supply started_at or duration_ms")
            duration_ms = (now - started_at) * 1000
        if math.isfinite(duration_ms) and duration_ms >= 0:
            self._enqueue(metric, source, now=now, duration_ms=duration_ms, bypass=int(bypass), available=1)

    def connection(self, source, connected):
        self._enqueue("connection", source, connected=int(connected))

    def define_traffic(self, source, description):
        with self._gate:
            self.metadata["traffic_scopes"][source] = description
            self._traffic.setdefault(source, [0, 0, 0, 0, self.clock()])

    def traffic(self, source, *, received=0, sent=0):
        if received < 0 or sent < 0:
            raise ValueError("Byte increments must be nonnegative")
        with self._gate:
            if self._closed:
                return
            sample = self._traffic.setdefault(source, [0, 0, 0, 0, self.clock()])
            sample[0] += received
            sample[1] += sent

    def _read_interface(self):
        try:
            return tuple(int((self._interface / (direction + "_bytes")).read_text()) for direction in ("rx", "tx"))
        except (OSError, ValueError):
            return None

    def monitor_interface(self, interface):
        if not re.fullmatch(r"[A-Za-z0-9_.:-]+", interface):
            raise ValueError("Invalid UB_EXPERIMENT_INTERFACE")
        with self._gate:
            self._interface = Path("/sys/class/net") / interface / "statistics"
            self._interface_source = "interface/" + interface
            self._interface_previous = self._read_interface()
            self.define_traffic(self._interface_source, "Host interface counters; includes other processes and protocol overhead")

    def sample_traffic(self):
        with self._gate:
            now = self.clock()
            interface = getattr(self, "_interface", None)
            unavailable = None
            if interface:
                latest = self._read_interface()
                previous = self._interface_previous
                if latest and previous and all(a >= b for a, b in zip(latest, previous)):
                    self.traffic(self._interface_source, received=latest[0]-previous[0], sent=latest[1]-previous[1])
                else:
                    unavailable = self._interface_source
                self._interface_previous = latest
            for source, sample in self._traffic.items():
                rx, tx, old_rx, old_tx, before = sample
                elapsed = now - before
                if elapsed <= 0:
                    continue
                available = source != unavailable
                self._enqueue("throughput", source, now=now, rx_bytes=rx, tx_bytes=tx,
                              rx_Bps=(rx-old_rx)/elapsed if available else "",
                              tx_Bps=(tx-old_tx)/elapsed if available else "", available=int(available))
                sample[2:] = [rx, tx, now]

    def add_probe(self, source, callback, metric="network_rtt", interval=1):
        """callback must have a bounded timeout. Failed probes emit no RTT sample."""
        with self._gate:
            if self._closed or self._stop.is_set() or (metric, source) in self._probes:
                return
            def run():
                state = None
                generation = 0
                while not self._stop.is_set():
                    started = self.clock()
                    try:
                        if callback() is False:
                            raise ConnectionError("Probe returned False")
                        if state is not True:
                            generation += 1
                            self.connection(source, True)
                        state = True
                        self.timing(metric, source + "/generation-" + str(generation), started_at=started)
                    except Exception:
                        if state is not False:
                            self.connection(source, False)
                        state = False
                    self._stop.wait(interval)
            worker = threading.Thread(target=run, name="experiment-probe", daemon=True)
            self._probes[(metric, source)] = worker
            worker.start()

    def _sample_loop(self):
        while not self._stop.wait(1):
            self.sample_traffic()

    def _write(self):
        next_flush = time.monotonic() + 1
        try:
            while not self._writer_stop.is_set() or not self._rows.empty():
                try:
                    self._csv.writerow(self._rows.get(timeout=.1))
                except queue.Empty:
                    pass
                if time.monotonic() >= next_flush:
                    self._file.flush()
                    next_flush = time.monotonic() + 1
        except Exception as exc:
            self.error = str(exc)
        finally:
            try:
                self._file.close()
            except Exception as exc:
                self.error = str(exc)

    def close(self):
        with self._close_gate:
            if self._closed:
                return
            self._stop.set()
            self._sampler.join()
            with self._gate:
                probes = list(self._probes.values())
            for worker in probes:
                worker.join(timeout=2)
                if worker.is_alive():
                    self.error = "Probe did not stop within two seconds; check its timeout"
            self.sample_traffic()
            with self._gate:
                self._closed = True
            self._writer_stop.set()
            self._writer.join()
            self._write_metadata()
            status = "started_utc={}\nstopped_utc={}\ndropped_rows={}\nerror={}\n".format(
                self.started_utc.isoformat(), datetime.now(timezone.utc).isoformat(), self.dropped_rows, self.error or "none")
            (self.directory / "recording-status.txt").write_text(status, encoding="utf-8")
            if self.plots:
                try:
                    from .plot import render
                    render(self.directory)
                except Exception as exc:
                    self.plot_error = str(exc)
                    print("Experiment graph generation failed; CSV preserved: {}".format(exc), file=sys.stderr)
            self._host_ownership()
            if self.error or self.dropped_rows:
                print("Experiment recording: error={}, dropped_rows={}".format(self.error, self.dropped_rows), file=sys.stderr)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


@contextmanager
def experiment(client_type="client", controller_type="unspecified"):
    """Share one recorder across controllers and transports in the same process."""
    global _active, _users
    if not enabled():
        yield None
        return
    with _session_lock:
        if _active is None:
            _active = Recorder(client_type=client_type,
                               controller_type=os.environ.get("UB_EXPERIMENT_CONTROLLER", controller_type),
                               client_id=os.environ.get("UB_EXPERIMENT_CLIENT_ID"),
                               plots=os.environ.get("UB_EXPERIMENT_PLOTS", "1") != "0")
        elif _active.metadata["controller_type"] in ("telemetry", "unspecified"):
            _active.metadata["controller_type"] = os.environ.get("UB_EXPERIMENT_CONTROLLER", controller_type)
        _install_signals()
        _users += 1
        recorder = _active
    try:
        yield recorder
    finally:
        with _session_lock:
            _users -= 1
            closing = _users == 0
            if closing:
                _active = None
        if closing:
            try:
                recorder.close()
            finally:
                _restore_signals()
