"""Thin CARLA lifecycle adapter. Importing this module does not import CARLA."""
import os
import json
import socket
import subprocess
import sys
from functools import wraps

from .recording import experiment


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
                            recorder.monitor_interface(route_interface(host))
                        except (OSError, ValueError, KeyError, IndexError, socket.gaierror, subprocess.SubprocessError) as exc:
                            print("Interface throughput unavailable; set UB_EXPERIMENT_INTERFACE explicitly: {}".format(exc), file=sys.stderr)
                return function(*args, **kwargs)
        return wrapped
    return decorate
