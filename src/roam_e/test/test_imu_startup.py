import pytest

from roam_e import imu_bno055_node
from roam_e.imu_bno055_node import BNO055ImuNode, EXPECTED_CHIP_ID


HEALTHY_STATE = (EXPECTED_CHIP_ID, 0x08, 0x05, 0x00, 0x00)
TRANSIENT_STATE = (EXPECTED_CHIP_ID, 0x00, 0x01, 0x00, 0x00)


class RecordingLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class FakeImuNode:
    def __init__(self, states):
        self.startup_timeout = 8.0
        self.startup_retry_period = 0.25
        self._register_values = iter(
            value for state in states for value in state
        )
        self.logger = RecordingLogger()

    def _read_register(self, _register):
        return next(self._register_values)

    def get_logger(self):
        return self.logger


def test_startup_retries_a_transient_unhealthy_fusion_state(monkeypatch):
    node = FakeImuNode([TRANSIENT_STATE, HEALTHY_STATE])
    times = iter([0.0, 0.0])
    monkeypatch.setattr(
        imu_bno055_node.time,
        'monotonic',
        lambda: next(times),
    )
    monkeypatch.setattr(imu_bno055_node.time, 'sleep', lambda _delay: None)

    BNO055ImuNode._verify_ready(node)

    assert len(node.logger.warnings) == 1
    assert 'waiting up to 8.0 seconds' in node.logger.warnings[0]


def test_startup_reports_the_last_state_after_timeout(monkeypatch):
    node = FakeImuNode([TRANSIENT_STATE])
    node.startup_timeout = 1.0
    times = iter([0.0, 1.0])
    monkeypatch.setattr(
        imu_bno055_node.time,
        'monotonic',
        lambda: next(times),
    )

    with pytest.raises(RuntimeError) as error:
        BNO055ImuNode._verify_ready(node)

    assert 'after restoration and startup retries' in str(error.value)
    assert 'mode=0x00, status=0x01' in str(error.value)
