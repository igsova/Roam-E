import ctypes
import pytest
from roam_e.uvc_capture import StreamControl, UvcCapture


def test_uvc_control_layout():
    # Matches Ubuntu libuvc 0.0.7 header, not a packed USB wire structure.
    assert ctypes.sizeof(StreamControl) == 40
    assert StreamControl.dwMaxPayloadTransferSize.offset == 24
    assert StreamControl.bInterfaceNumber.offset == 36


def test_invalid_mode_rejected_before_opening_hardware():
    with pytest.raises(ValueError):
        UvcCapture(320, 240, 30)
    with pytest.raises(ValueError):
        UvcCapture(640, 480, 30.5)


def test_invalid_payload_rejected_before_opening_hardware():
    with pytest.raises(ValueError):
        UvcCapture(640, 480, 30, 0)


def test_error_status_is_not_silenced():
    with pytest.raises(RuntimeError, match='failed'):
        UvcCapture.check(-3, 'open')
    UvcCapture.check(0, 'open')
