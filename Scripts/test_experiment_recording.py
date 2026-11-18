import csv
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import Mock, patch
from types import SimpleNamespace

from ub_experiment import Recorder, apply_control, current, experiment, input_callback, measured_socket
from ub_experiment.plot import load_metrics, render


class Clock:
    now = 0.
    def __call__(self):
        return self.now


class RecordingChecks(unittest.TestCase):
    def rows(self, directory):
        with (Path(directory) / "metrics.csv").open(newline="") as stream:
            return list(csv.DictReader(stream))

    def test_concurrent_timings_and_final_partial_rate(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = Clock()
            with Recorder(directory, plots=False, clock=clock, client_type="custom", controller_type="joystick") as recorder:
                recorder.define_traffic("serial_payload", "Serial device payload only")
                def work(_):
                    recorder.timing("custom_stage", 'input, "wheel"', duration_ms=12)
                    recorder.traffic("serial_payload", received=2, sent=3)
                with ThreadPoolExecutor(4) as workers:
                    list(workers.map(work, range(2000)))
                clock.now = .5
            recorder.close()
            rows = self.rows(directory)
            self.assertEqual(sum(row["metric"] == "custom_stage" for row in rows), 2000)
            rates = [row for row in rows if row["metric"] == "throughput"]
            self.assertEqual(len(rates), 1)
            self.assertEqual(float(rates[0]["rx_Bps"]), 8000)
            self.assertEqual(float(rates[0]["tx_Bps"]), 12000)
            self.assertEqual(int(rates[0]["rx_bytes"]), 4000)
            self.assertTrue(all(row["source"] == 'input, "wheel"' for row in rows if row["metric"] == "custom_stage"))
            metadata = json.loads((Path(directory) / "experiment.json").read_text())
            self.assertEqual(metadata["controller_type"], "joystick")
            self.assertIn("serial_payload", metadata["traffic_scopes"])
            self.assertIn("dropped_rows=0", (Path(directory) / "recording-status.txt").read_text())
            with self.assertRaises(FileExistsError):
                Recorder(directory, plots=False)

    def test_controls_record_only_successful_submissions(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = Clock()
            with Recorder(directory, plots=False, clock=clock) as recorder, patch("ub_experiment.recording._active", recorder):
                vehicle = Mock()
                def submit(control):
                    clock.now += .02
                    return "submitted"
                vehicle.apply_control.side_effect = submit
                @input_callback("drive-sim")
                def tick(control):
                    clock.now += .03
                    return apply_control(vehicle, control)
                control = object()
                self.assertEqual(tick(control), "submitted")
                vehicle.apply_control.assert_called_with(control)
                vehicle.apply_control.side_effect = RuntimeError("network failure")
                with self.assertRaises(RuntimeError):
                    tick(control)
            rows = [row for row in self.rows(directory) if row["metric"] == "receive_to_publish"]
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["source"], "control/drive-sim")
            self.assertAlmostEqual(float(rows[0]["duration_ms"]), 50)
            vehicle.apply_control.side_effect = None
            apply_control(vehicle, control)  # With recording off, behavior remains a direct call.
            self.assertEqual(len(self.rows(directory)), 1)

    def test_probe_failures_do_not_fabricate_rtt_and_workers_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            completed = threading.Event()
            calls = []
            def probe():
                calls.append(1)
                if len(calls) in (1, 3):
                    raise TimeoutError("offline")
                if len(calls) >= 4:
                    completed.set()
                return "server-version"
            with Recorder(directory, plots=False) as recorder:
                recorder.add_probe("carla_rpc", probe, "carla_rpc_rtt", interval=.01)
                self.assertTrue(completed.wait(2))
            rows = self.rows(directory)
            self.assertEqual([row["connected"] for row in rows if row["metric"] == "connection"], ["0", "1", "0", "1"])
            self.assertTrue(any(row["metric"] == "carla_rpc_rtt" and row["source"].endswith("generation-2") for row in rows))
            self.assertFalse(recorder._writer.is_alive())
            self.assertFalse(recorder._sampler.is_alive())
            self.assertTrue(all(not worker.is_alive() for worker in recorder._probes.values()))
            count = len(calls)
            time.sleep(.03)
            self.assertEqual(len(calls), count)

    def test_shared_session_naming_and_disabled_recording(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
            "UB_EXPERIMENT_ENABLED": "1", "UB_EXPERIMENT_DIR": directory,
            "UB_EXPERIMENT_HOST_NAME": "driving/host name", "UB_EXPERIMENT_PLOTS": "0",
        }):
            with experiment("carla", "mustang") as first:
                with experiment("carla", "wheel") as second:
                    self.assertIs(first, second)
                first.timing("receive_to_publish", "controls", duration_ms=1)
                self.assertIs(current(), first)
            self.assertIsNone(current())
            run = next(Path(directory).iterdir())
            self.assertRegex(run.name, r"^run-\d{8}-\d{6}Z-driving_host_name-[a-f0-9]{8}$")
            self.assertEqual(len(self.rows(run)), 1)
            with patch.dict(os.environ, {"UB_EXPERIMENT_ENABLED": "0"}):
                with experiment() as disabled:
                    self.assertIsNone(disabled)
            self.assertEqual(len(list(Path(directory).iterdir())), 1)

    def test_udp_bytes_are_counted_after_successful_calls(self):
        with tempfile.TemporaryDirectory() as directory:
            with Recorder(directory, plots=False) as recorder, patch("ub_experiment.recording._active", recorder):
                sock = Mock()
                sock.sendto.return_value = 3
                sock.recvfrom.return_value = (b"reply", ("peer", 123))
                wrapped = measured_socket(sock, "udp/mustang")
                self.assertEqual(wrapped.sendto(b"abc", ("peer", 123)), 3)
                self.assertEqual(wrapped.recvfrom(100)[0], b"reply")
                sock.sendto.side_effect = OSError("offline")
                with self.assertRaises(OSError):
                    wrapped.sendto(b"not sent", ("peer", 123))
            rate = next(row for row in self.rows(directory) if row["metric"] == "throughput")
            self.assertEqual((int(rate["rx_bytes"]), int(rate["tx_bytes"])), (5, 3))

    def test_generic_and_legacy_graphs(self):
        with tempfile.TemporaryDirectory() as directory:
            with Recorder(directory, plots=False, client_type="custom-client") as recorder:
                recorder.timing("redis_rtt", "legacy-redis", duration_ms=5)
                recorder.timing("carla_rpc_rtt", "carla", duration_ms=10)
                recorder.timing("custom_stage", "joystick", duration_ms=2)
                recorder.traffic("udp/mustang", received=100, sent=200)
            render(Path(directory))
            for name in ("receive_to_publish.png", "network_rtt.png", "network_throughput.png", "timing_custom_stage.png"):
                self.assertGreater((Path(directory) / name).stat().st_size, 1000)
            summary = json.loads((Path(directory) / "summary.json").read_text())
            self.assertEqual(summary["timings"]["legacy-redis/redis_rtt"]["mean_ms"], 5)
            self.assertEqual(summary["experiment"]["client_type"], "custom-client")

    def test_carla_adapter_uses_independent_probe_and_preserves_arguments(self):
        from ub_experiment.carla import carla_session
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {
            "UB_EXPERIMENT_ENABLED": "1", "UB_EXPERIMENT_DIR": directory,
            "UB_EXPERIMENT_PLOTS": "0", "UB_EXPERIMENT_INTERFACE": "auto",
            "UB_MANUAL_CARLA_PORT": "invalid-ignored-because-args-provided",
        }):
            carla = Mock()
            carla.Client.return_value.get_server_version.return_value = "version"
            @carla_session("wheel")
            def run(args):
                vehicle = Mock()
                vehicle.apply_control.return_value = "ok"
                @input_callback("wheel")
                def poll():
                    return apply_control(vehicle, "unchanged-command")
                self.assertEqual(poll(), "ok")
                vehicle.apply_control.assert_called_once_with("unchanged-command")
                return 42
            with patch.dict(sys.modules, {"carla": carla}), patch("ub_experiment.carla.route_interface", return_value="lo"):
                self.assertEqual(run(SimpleNamespace(host="remote-server", port=2100)), 42)
            carla.Client.assert_called_once_with("remote-server", 2100, worker_threads=1)
            carla.Client.return_value.set_timeout.assert_called_once_with(1.0)
            carla.Client.return_value.get_world.assert_not_called()
            run_dir = next(Path(directory).iterdir())
            self.assertTrue(any(row["metric"] == "carla_rpc_rtt" for row in self.rows(run_dir)))
            self.assertTrue(any(row["source"] == "interface/lo" for row in self.rows(run_dir)))

    def test_sigterm_drains_and_restores_session(self):
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / "ready"
            code = '''
import time,sys
from pathlib import Path
from ub_experiment import experiment
try:
    with experiment("carla", "joystick") as recorder:
        recorder.timing("receive_to_publish", "control/joystick", duration_ms=5)
        Path(sys.argv[1]).write_text("ready")
        while True: time.sleep(1)
except KeyboardInterrupt:
    pass
'''
            env = dict(os.environ, UB_EXPERIMENT_ENABLED="1", UB_EXPERIMENT_DIR=directory, UB_EXPERIMENT_PLOTS="0")
            process = subprocess.Popen([sys.executable, "-c", code, str(ready)], env=env,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and time.monotonic() < deadline and process.poll() is None:
                    time.sleep(.01)
                self.assertTrue(ready.exists(), "Child did not start its recording")
                process.send_signal(signal.SIGTERM)
                out, err = process.communicate(timeout=5)
                self.assertEqual(process.returncode, 0, out + err)
                run_dir = next(Path(directory).glob("run-*"))
                self.assertEqual(len(self.rows(run_dir)), 1)
                self.assertIn("error=none", (run_dir / "recording-status.txt").read_text())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()


if __name__ == "__main__":
    unittest.main()
