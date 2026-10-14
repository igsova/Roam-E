"""Offline thread/queue tests: readers are synthetic, never vcgencmd/hardware."""
import queue
import subprocess
import threading
import time
import unittest
from r19_power_monitor import PowerMonitor, QUERY_TIMEOUT, MAX_SAMPLE_AGE

class PowerMonitorTest(unittest.TestCase):
    def ready(self,monitor):
        deadline=time.monotonic()+.3
        while monitor.messages.empty() and time.monotonic()<deadline: time.sleep(.001)
        self.assertFalse(monitor.messages.empty(),'Worker produced no sample')
    def make(self,reader,clock=time.monotonic):
        monitor=PowerMonitor(reader,clock=clock)
        self.addCleanup(monitor.close)
        return monitor

    def test_snapshot_and_exact_query_timeout(self):
        def reader(args,timeout):
            self.assertEqual(args,['vcgencmd','get_throttled'])
            self.assertEqual(timeout,.10)
            return 'throttled=0x0'
        m=self.make(reader); m.start(); self.ready(m)
        samples=m.check()
        self.assertTrue(samples); self.assertEqual(samples[-1]['flags'],0)
        self.assertLessEqual(samples[-1]['completed']-samples[-1]['began'],QUERY_TIMEOUT)

    def test_sixty_ms_query_does_not_block_control_reader(self):
        def reader(args,timeout):
            self.assertEqual(timeout,.10); time.sleep(.06); return 'throttled=0x0'
        m=self.make(reader); m.start(); self.ready(m)
        samples=m.check()
        self.assertGreaterEqual(samples[0]['completed']-samples[0]['began'],.05)
        self.assertLessEqual(samples[0]['completed']-samples[0]['began'],.10)

    def test_check_nonblocking_during_inflight_query(self):
        entered=threading.Event(); release=threading.Event(); calls=[]
        def reader(args,timeout):
            calls.append(1)
            if len(calls)>1:
                entered.set(); release.wait(.2)
            return 'throttled=0x0'
        m=self.make(reader); self.addCleanup(release.set); m.start()
        self.assertTrue(entered.wait(.2))
        began=time.monotonic(); samples=m.check(); elapsed=time.monotonic()-began
        self.assertTrue(samples); self.assertLess(elapsed,.025)
        release.set()

    def test_timeout_latches_and_never_retries(self):
        calls=[]
        def reader(args,timeout):
            calls.append(1); raise subprocess.TimeoutExpired(args,timeout)
        m=self.make(reader); m.start(); m.thread.join(.2)
        with self.assertRaisesRegex(RuntimeError,'timed out'): m.check()
        with self.assertRaisesRegex(RuntimeError,'timed out'): m.check()
        self.assertEqual(len(calls),1)

    def test_invalid_flags_never_retried(self):
        calls=[]
        def reader(args,timeout): calls.append(1); return 'not flags'
        m=self.make(reader); m.start(); m.thread.join(.2)
        with self.assertRaisesRegex(RuntimeError,'Invalid Pi power flags'): m.check()
        self.assertEqual(len(calls),1)

    def test_reader_exceeding_query_deadline_is_fault(self):
        stamps=iter((0.,.101))
        m=self.make(lambda args,timeout:'throttled=0x0',clock=lambda:next(stamps))
        m.start(); m.thread.join(.2)
        with self.assertRaisesRegex(RuntimeError,'exceeded100ms'): m.check()

    def test_stale_snapshot_and_unavailable_fail(self):
        m=self.make(lambda args,timeout:'throttled=0x0',clock=lambda:1.)
        with self.assertRaisesRegex(RuntimeError,'unavailable'): m.check()
        m.messages.put({'began':1-MAX_SAMPLE_AGE-.001,'completed':.9,'flags':0})
        with self.assertRaisesRegex(RuntimeError,'Stale'): m.check()

    def test_fault_sample_not_overwritten_by_later_healthy_sample(self):
        m=self.make(lambda args,timeout:'throttled=0x0',clock=lambda:1.)
        m.messages.put({'began':.98,'completed':.99,'flags':5})
        m.messages.put({'began':.99,'completed':1.,'flags':0})
        self.assertEqual([x['flags'] for x in m.check()],[5,0])

    def test_error_not_overwritten_and_close_before_start(self):
        m=self.make(lambda args,timeout:'throttled=0x0',clock=lambda:1.)
        m.messages.put({'began':.98,'error':'diagnostic failure'})
        m.messages.put({'began':.99,'completed':1.,'flags':0})
        with self.assertRaisesRegex(RuntimeError,'diagnostic failure'): m.check()
        self.assertTrue(m.close())

if __name__=='__main__': unittest.main()
