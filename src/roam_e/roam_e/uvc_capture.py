"""Minimal libuvc 0.0.7 binding for the Astra Pro's separate RGB interface.

Structures follow libuvc/libuvc.h. No camera firmware is changed. Payload
reservation is negotiated for this stream only; uvc_close restores ownership.
"""
import ctypes as C
import ctypes.util
import time


class StreamControl(C.Structure):
    _fields_ = [
        ('bmHint', C.c_uint16), ('bFormatIndex', C.c_uint8),
        ('bFrameIndex', C.c_uint8), ('dwFrameInterval', C.c_uint32),
        ('wKeyFrameRate', C.c_uint16), ('wPFrameRate', C.c_uint16),
        ('wCompQuality', C.c_uint16), ('wCompWindowSize', C.c_uint16),
        ('wDelay', C.c_uint16), ('dwMaxVideoFrameSize', C.c_uint32),
        ('dwMaxPayloadTransferSize', C.c_uint32), ('dwClockFrequency', C.c_uint32),
        ('bmFramingInfo', C.c_uint8), ('bPreferredVersion', C.c_uint8),
        ('bMinVersion', C.c_uint8), ('bMaxVersion', C.c_uint8),
        ('bInterfaceNumber', C.c_uint8),
    ]


class FramePrefix(C.Structure):
    # Only read the prefix; libuvc allocates and owns the entire frame.
    _fields_ = [('data', C.c_void_p), ('data_bytes', C.c_size_t),
                ('width', C.c_uint32), ('height', C.c_uint32),
                ('frame_format', C.c_int), ('step', C.c_size_t)]


class UvcCapture:
    def __init__(self, width, height, fps, payload=800):
        if (width, height) != (640, 480) or fps != 30:
            raise ValueError('Direct Astra capture currently supports 640x480 at 30 FPS only')
        if not 128 <= payload <= 3072:
            raise ValueError('USB payload reservation must be 128..3072 bytes')
        path = ctypes.util.find_library('uvc')
        if not path:
            raise RuntimeError('Install libuvc0 for direct Astra RGB capture')
        self.lib = C.CDLL(path)
        P = C.c_void_p
        signatures = {
            'uvc_init': [C.POINTER(P), P],
            'uvc_find_device': [P, C.POINTER(P), C.c_int, C.c_int, C.c_char_p],
            'uvc_open': [P, C.POINTER(P)],
            'uvc_get_stream_ctrl_format_size': [P, C.POINTER(StreamControl), C.c_int, C.c_int, C.c_int, C.c_int],
            'uvc_stream_open_ctrl': [P, C.POINTER(P), C.POINTER(StreamControl)],
            'uvc_stream_start': [P, P, P, C.c_uint8],
            'uvc_stream_get_frame': [P, C.POINTER(C.POINTER(FramePrefix)), C.c_int32],
            'uvc_stream_stop': [P], 'uvc_stream_close': [P],
            'uvc_close': [P], 'uvc_unref_device': [P], 'uvc_exit': [P],
        }
        for name, args in signatures.items():
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, C.c_int
        for name in ['uvc_stream_close', 'uvc_close', 'uvc_unref_device', 'uvc_exit']:
            getattr(self.lib, name).restype = None
        self.ctx, self.dev, self.handle, self.stream = P(), P(), P(), P()
        self.started = False
        self.width, self.height = width, height
        try:
            self.check(self.lib.uvc_init(C.byref(self.ctx), None), 'initialize')
            self.check(self.lib.uvc_find_device(self.ctx, C.byref(self.dev), 0x2bc5, 0x0502, None), 'find Astra RGB')
            self.check(self.lib.uvc_open(self.dev, C.byref(self.handle)), 'open RGB (check video-group USB permissions)')
            ctrl = StreamControl()
            for attempt in range(3):
                result = self.lib.uvc_get_stream_ctrl_format_size(self.handle, C.byref(ctrl), 7, width, height, int(fps))
                if result >= 0:
                    break
                time.sleep(.3)
            self.check(result, 'negotiate VGA MJPEG')
            self.negotiated_payload = ctrl.dwMaxPayloadTransferSize
            ctrl.dwMaxPayloadTransferSize = payload
            self.check(self.lib.uvc_stream_open_ctrl(self.handle, C.byref(self.stream), C.byref(ctrl)), 'open stream')
            self.check(self.lib.uvc_stream_start(self.stream, None, None, 0), 'start stream')
            self.started = True
        except Exception:
            self.release()
            raise

    @staticmethod
    def check(result, operation):
        if result < 0:
            raise RuntimeError(f'libuvc {operation} failed ({result})')

    def read(self):
        frame = C.POINTER(FramePrefix)()
        result = self.lib.uvc_stream_get_frame(self.stream, C.byref(frame), 500000)
        if result == -7 or not frame and result >= 0:
            return None
        self.check(result, 'read')
        f = frame.contents
        if (f.width, f.height, f.frame_format) != (self.width, self.height, 7):
            raise RuntimeError('Unexpected RGB frame dimensions/format')
        if not f.data or not 0 < f.data_bytes <= 4 * 1024 * 1024:
            raise ValueError('Invalid RGB frame length')
        return C.string_at(f.data, f.data_bytes)

    def release(self):
        if self.stream:
            if self.started:
                self.lib.uvc_stream_stop(self.stream)
            self.lib.uvc_stream_close(self.stream)
            self.stream = C.c_void_p()
        if self.handle:
            self.lib.uvc_close(self.handle)
            self.handle = C.c_void_p()
        if self.dev:
            self.lib.uvc_unref_device(self.dev)
            self.dev = C.c_void_p()
        if self.ctx:
            self.lib.uvc_exit(self.ctx)
            self.ctx = C.c_void_p()
