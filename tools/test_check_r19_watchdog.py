"""Offline watchdog-guard fault tests; no physical motor/watchdog claims."""
import unittest
import check_r19_watchdog as m
class LeaseTests(unittest.TestCase):
    def guard(self):
        g=m.WatchdogGuard([0,0,0,0,0,0,0,0,0],0,1)
        g.ack(.02)
        g.observe([50,10,10,128,128,200,200,180,180],.05)
        g.phase([50,2,2,0],.05)
        g.observe([250,50,50,90,90,200,200,200,200],.25)
        g.phase([250,2,2,0],.25)
        g.observe([350,70,70,90,90,200,200,200,200],.35)
        g.phase([350,2,2,0],.35)
        return g
    def test_expected_stop(self):
        g=self.guard(); g.event(.36)
        g.observe([400,72,72,0,0,0,0,0,0],.4)
        g.phase([400,0,0,0],.4); g.check(.41)
        self.assertTrue(g.watchdog_seen)
        self.assertEqual(g.zero_samples,1)
    def test_deliberate_ack_expiry(self):
        g=self.guard(); g.last_received=g.last_phase=.25
        g.last_progress=[.25,.25]; g.check(.26)
    def test_missing_initial_ack(self):
        g=m.WatchdogGuard([0,0,0,0,0,0,0,0,0],0,1)
        g.last_received=g.last_phase=.16
        with self.assertRaises(RuntimeError): g.check(.16)
    def test_premature_late_missing_event(self):
        for t in (.1,.51):
            with self.subTest(t=t):
                with self.assertRaises(RuntimeError): self.guard().event(t)
        g=self.guard(); g.last_received=g.last_phase=.5
        g.last_progress=[.5,.5]
        with self.assertRaises(RuntimeError): g.check(.51)
    def test_no_run_proof(self):
        g=self.guard(); g.run_seen=[True,False]
        with self.assertRaises(RuntimeError): g.event(.36)
    def test_energized_after_event(self):
        g=self.guard(); g.event(.36)
        with self.assertRaises(RuntimeError):
            g.observe([100,15,15,1,0,0,0,0,0],.4)
    def test_phase_fault_after_event(self):
        g=self.guard(); g.event(.36)
        with self.assertRaises(RuntimeError): g.phase([100,0,0,1],.4)
    def test_stale_after_event(self):
        g=self.guard(); g.event(.36)
        with self.assertRaises(RuntimeError): g.check(.51)
if __name__=='__main__': unittest.main()
