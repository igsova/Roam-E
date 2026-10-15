"""Non-blocking Pi status sampler for bounded test helpers; never controls motors.

Queries run only in a daemon worker. No timeout/error is silently retried.
check() drains every sample/error and requires a sample initiated within150ms.
The caller still checks current/new historical power faults and owns STOP.
"""
import queue
import re
import threading
import time

QUERY_TIMEOUT = .10
POLL_INTERVAL = .04
MAX_SAMPLE_AGE = .15

class PowerMonitor:
    def __init__(self, command, clock=time.monotonic):
        self.command, self.clock = command, clock
        self.messages = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run,name='r19-pi-power',daemon=True)
        self.latest = None
        self.failure = None
        self.started = False

    def start(self):
        if self.started: raise RuntimeError('Power monitor already started')
        self.started = True
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            began = self.clock()
            try:
                raw = self.command(['vcgencmd','get_throttled'],timeout=QUERY_TIMEOUT)
                completed = self.clock()
                if completed-began >QUERY_TIMEOUT:
                    raise RuntimeError('Pi power query exceeded100ms deadline')
                if not re.fullmatch(r'throttled=0x[0-9a-fA-F]+',raw):
                    raise RuntimeError('Invalid Pi power flags')
                sample = {'began':began,'completed':completed,'flags':int(raw.split('=')[1],16)}
            except Exception as error:
                self.messages.put({'began':began,'error':str(error)})
                return # No retry after diagnostic failure.
            self.messages.put(sample)
            if self.stop_event.wait(POLL_INTERVAL): return

    def check(self):
        samples = []
        while True:
            try: sample = self.messages.get_nowait()
            except queue.Empty: break
            if 'error' in sample:
                self.failure = sample['error']
            else:
                samples.append(sample)
                self.latest = sample
        if self.failure: raise RuntimeError(self.failure)
        if self.latest is None: raise RuntimeError('Pi power monitor unavailable')
        age = self.clock()-self.latest['began']
        if age <0 or age >MAX_SAMPLE_AGE: raise RuntimeError('Stale Pi power sample')
        return samples

    def close(self):
        self.stop_event.set()
        if self.started: self.thread.join(.25) # Called after STOP, never in motion loop.
        return not self.started or not self.thread.is_alive()
