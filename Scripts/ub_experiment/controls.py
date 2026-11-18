import threading
from contextlib import contextmanager
from functools import wraps

from .recording import current

_cycle = threading.local()


@contextmanager
def control_cycle(source):
    recorder = current()
    previous = getattr(_cycle, "sample", None)
    _cycle.sample = (recorder, source, recorder.clock()) if recorder else None
    try:
        yield
    finally:
        _cycle.sample = previous


def input_callback(source):
    """Start timing at input polling/callback entry, excluding the frame limiter."""
    def decorate(function):
        @wraps(function)
        def wrapped(*args, **kwargs):
            with control_cycle(source):
                return function(*args, **kwargs)
        return wrapped
    return decorate


def apply_control(vehicle, control):
    result = vehicle.apply_control(control)
    sample = getattr(_cycle, "sample", None)
    if sample:
        recorder, source, started = sample
        recorder.timing("receive_to_publish", "control/" + source, started_at=started)
    return result


class MeasuredSocket:
    """Delegating socket wrapper for explicit UDP/TCP payload accounting."""
    def __init__(self, sock, source, recorder):
        self._socket, self._source, self._recorder = sock, source, recorder
        recorder.define_traffic(source, "Successful socket data bytes; excludes IP/UDP/TCP headers and retransmissions")

    def __getattr__(self, name):
        return getattr(self._socket, name)

    def sendto(self, *args, **kwargs):
        count = self._socket.sendto(*args, **kwargs)
        self._recorder.traffic(self._source, sent=count)
        return count

    def recvfrom(self, *args, **kwargs):
        data, address = self._socket.recvfrom(*args, **kwargs)
        self._recorder.traffic(self._source, received=len(data))
        return data, address

    def send(self, *args, **kwargs):
        count = self._socket.send(*args, **kwargs)
        self._recorder.traffic(self._source, sent=count)
        return count

    def recv(self, *args, **kwargs):
        data = self._socket.recv(*args, **kwargs)
        self._recorder.traffic(self._source, received=len(data))
        return data


def measured_socket(sock, source):
    recorder = current()
    return MeasuredSocket(sock, source, recorder) if recorder else sock
