"""Thin CARLA lifecycle adapter. Importing this module does not import CARLA."""
import os
import json
import socket
import subprocess
import sys
from functools import wraps

from .recording import experiment
from .paths import client_log_directory


def add_network_logging_arguments(parser):
    """Expose the shared recorder without changing a client's default policy."""
    group = parser.add_argument_group("network experiment logging")
    toggle = group.add_mutually_exclusive_group()
    toggle.add_argument("--log-network", dest="log_network", action="store_true", default=None,
                        help="Record network statistics and control timings using UB-MR's shared recorder.")
    toggle.add_argument("--no-log-network", dest="log_network", action="store_false",
                        help="Disable experiment recording, overriding UB_EXPERIMENT_ENABLED.")
    group.add_argument("--log-dir", metavar="DIRECTORY",
                       help="Parent directory for experiment CSV, metadata, summaries and graphs.")
    group.add_argument("--network-interface", metavar="INTERFACE",
                       help="Host interface for byte counters, or auto/none (host-wide, not process-specific).")


def configure_network_logging(args):
    """Apply CLI overrides before any controllers or Redis renderers start."""
    os.environ.setdefault("UB_CLIENT_LOG_DIR", str(client_log_directory("carla")))
    enabled = getattr(args, "log_network", None)
    if enabled is not None:
        os.environ["UB_EXPERIMENT_ENABLED"] = "1" if enabled else "0"
    for attribute, variable in (("log_dir", "UB_EXPERIMENT_DIR"),
                                ("network_interface", "UB_EXPERIMENT_INTERFACE")):
        value = getattr(args, attribute, None)
        if value is not None:
            os.environ[variable] = value


def route_interface(host):
    """Ask the Linux routing table, including VPN/policy routes, for the probe path."""
    address = socket.getaddrinfo(host, None, type=socket.SOCK_STREAM)[0][4][0]
    reply = subprocess.run(["ip", "-j", "route", "get", address], capture_output=True, text=True,
                           check=True, timeout=1)
    return json.loads(reply.stdout)[0]["dev"]


def carla_session(controller_type):
    def decorate(function):
        @wraps(function)
        def wrapped(*args, **kwargs):
            with experiment("carla", controller_type) as recorder:
                if recorder:
                    import carla
                    config = args[0] if args else None
                    host = config.host if config is not None else os.environ.get("UB_MANUAL_CARLA_HOST", "127.0.0.1")
                    port = config.port if config is not None else int(os.environ.get("UB_MANUAL_CARLA_PORT", "2000"))
                    probe = carla.Client(host, port, worker_threads=1)
                    probe.set_timeout(1.0)
                    recorder.add_probe("carla_rpc/{0}:{1}".format(host, port), probe.get_server_version, "carla_rpc_rtt")
                    if os.environ.get("UB_EXPERIMENT_INTERFACE", "auto") in ("", "auto"):
                        try:
                            network_host = os.environ.get("UB_EXPERIMENT_NETWORK_HOST") or host
                            recorder.monitor_interface(route_interface(network_host))
                        except (OSError, ValueError, KeyError, IndexError, socket.gaierror, subprocess.SubprocessError) as exc:
                            print("Interface throughput unavailable; set UB_EXPERIMENT_INTERFACE explicitly: {}".format(exc), file=sys.stderr)
                return function(*args, **kwargs)
        return wrapped
    return decorate
