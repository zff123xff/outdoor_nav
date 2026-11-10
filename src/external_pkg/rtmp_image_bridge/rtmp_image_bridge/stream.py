"""RTMP video decoding (FFmpeg or GStreamer) without OpenCV, NumPy, or ROS dependencies.

Two interchangeable backends produce raw interleaved frames on a pipe:

* ``ffmpeg``      -- system FFmpeg, software decode (measured 312 fps @1080p30 on Orin NX).
* ``ffmpeg-hw``   -- FFmpeg with an NVDEC/CUVID decoder, only used after a real decode probe.
* ``gstreamer``   -- rtmpsrc + h264parse + nvv4l2decoder + nvvidconv, requires
                     gstreamer1.0-plugins-bad (h264parse, rtmpsrc).

``decoder: auto`` probes what actually works and reports the choice; it never assumes that
hardware decoding is active just because FFmpeg lists a hwaccel.
"""
from dataclasses import dataclass
import math
import os
import re
import selectors
import shutil
import subprocess
import threading
import time
from urllib.parse import urlsplit

PIX_FMT = {'rgb8': 'rgb24', 'bgr8': 'bgr24'}
DECODERS = ('auto', 'ffmpeg', 'ffmpeg-hw', 'gstreamer')
# rtmpsrc/h264parse live in gstreamer1.0-plugins-bad; the rest in base/good + NVIDIA.
GST_REQUIRED = ('rtmpsrc', 'flvdemux', 'h264parse', 'nvv4l2decoder', 'nvvidconv',
                'videoconvert', 'fdsink')


@dataclass(frozen=True)
class Settings:
    rtmp_url: str
    mode: str = 'pull'
    width: int = 640
    height: int = 480
    encoding: str = 'rgb8'
    publish_fps: float = 30.
    read_timeout_s: float = 5.
    reconnect_delay_s: float = 2.
    max_frame_age_s: float = .5
    ffmpeg_path: str = 'ffmpeg'
    gst_launch_path: str = 'gst-launch-1.0'
    decoder: str = 'auto'
    stats_period_s: float = 5.
    read_chunk_bytes: int = 262144
    # Optional second rawvideo output (same decode, extra scale) for lightweight
    # consumers such as the Python YOLO node. 0 disables it.
    secondary_width: int = 0
    secondary_height: int = 0

    def __post_init__(self):
        try:
            parsed = urlsplit(self.rtmp_url)
            valid = parsed.scheme in ('rtmp', 'rtmps') and parsed.hostname and parsed.port != 0
        except (ValueError, TypeError, AttributeError):
            valid = False
        if not valid:
            raise ValueError('rtmp_url must be a valid rtmp:// or rtmps:// endpoint')
        if self.mode not in ('pull', 'listen'):
            raise ValueError('mode must be pull or listen')
        if self.mode == 'listen' and parsed.scheme != 'rtmp':
            raise ValueError('listen mode supports rtmp://; use a server for RTMPS')
        if self.encoding not in PIX_FMT:
            raise ValueError('encoding must be rgb8 or bgr8')
        if self.decoder not in DECODERS:
            raise ValueError(f'decoder must be one of {", ".join(DECODERS)}')
        for key in ('width', 'height'):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 < value <= 8192:
                raise ValueError(f'{key} must be an integer in 1..8192')
        if self.width*self.height*3 > 64*1024*1024:
            raise ValueError('decoded frame must be at most 64 MiB')
        if (self.secondary_width == 0) != (self.secondary_height == 0):
            raise ValueError('secondary_width and secondary_height must be set together')
        for key in ('secondary_width', 'secondary_height'):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 8192:
                raise ValueError(f'{key} must be an integer in 0..8192')
        if self.secondary_width and (self.secondary_width >= self.width or
                                     self.secondary_height >= self.height):
            raise ValueError('secondary size must be smaller than the primary size')
        if self.secondary_enabled and (self.width % self.secondary_width or
                                       self.height % self.secondary_height or
                                       self.width//self.secondary_width != self.height//self.secondary_height):
            raise ValueError('secondary size must be an integer decimation of the primary '
                             'size (e.g. 1920x1080 -> 640x360)')
        if self.secondary_width*self.secondary_height*3 > 64*1024*1024:
            raise ValueError('secondary frame must be at most 64 MiB')
        for key in ('publish_fps', 'read_timeout_s', 'reconnect_delay_s', 'max_frame_age_s',
                    'stats_period_s'):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if self.publish_fps > 120:
            raise ValueError('publish_fps must be <= 120')
        if isinstance(self.read_chunk_bytes, bool) or not isinstance(self.read_chunk_bytes, int) or self.read_chunk_bytes < 4096:
            raise ValueError('read_chunk_bytes must be an integer >= 4096')
        if not isinstance(self.ffmpeg_path, str) or not self.ffmpeg_path:
            raise ValueError('ffmpeg_path must be a non-empty executable path')
        if not isinstance(self.gst_launch_path, str) or not self.gst_launch_path:
            raise ValueError('gst_launch_path must be a non-empty executable path')

    @property
    def frame_bytes(self):
        return self.width*self.height*3

    @property
    def secondary_frame_bytes(self):
        return self.secondary_width*self.secondary_height*3

    @property
    def secondary_enabled(self):
        return self.secondary_width > 0 and self.secondary_height > 0


def ffmpeg_command(settings, decode_args=()):
    executable = shutil.which(settings.ffmpeg_path)
    if executable is None:
        raise ValueError('FFmpeg executable not found; install ffmpeg or set ffmpeg_path')
    # Measured on this board: '-fflags nobuffer -flags low_delay' makes FFmpeg drop the
    # H.264 sequence header of a live RTMP stream (decode_slice_header error, zero frames),
    # so both are deliberately absent. The remaining flags only bound startup and reconnects.
    command = [executable, '-hide_banner', '-loglevel', 'error', '-nostdin',
               '-threads', '2', '-rw_timeout', str(max(1, int(settings.read_timeout_s*1e6))),
               '-analyzeduration', '1000000', '-probesize', '1000000', '-rtmp_buffer', '0']
    command.extend(decode_args)
    if settings.mode == 'listen':
        command.extend(['-listen', '1'])
    else:
        command.extend(['-rtmp_live', 'live'])
    command.extend(['-i', settings.rtmp_url, '-an', '-sn', '-dn'])
    # A second rawvideo pipe halves throughput (measured 15.3 -> 7.0 fps @1080p), so the
    # small output is produced in-process by integer decimation of the primary frame.
    command.extend(['-map', '0:v:0', '-vf', f'scale={settings.width}:{settings.height}',
                    '-pix_fmt', PIX_FMT[settings.encoding],
                    '-threads', '1', '-vsync', '0', '-f', 'rawvideo', 'pipe:1'])
    return command


def probe_decoder(ffmpeg_path, decode_args, timeout=20.):
    """Decode synthetic H.264 in-process: proves the decoder opens and returns frames.

    A hwaccel entry in ``ffmpeg -hwaccels`` is not evidence that decoding works; this
    probe fails for the common Jetson case of a missing ``libnvcuvid.so.1``.
    """
    executable = shutil.which(ffmpeg_path)
    if executable is None:
        return False
    try:
        encode = subprocess.run(
            [executable, '-hide_banner', '-loglevel', 'error', '-f', 'lavfi',
             '-i', 'testsrc2=size=320x240:rate=5:duration=1',
             '-c:v', 'libx264', '-preset', 'ultrafast', '-pix_fmt', 'yuv420p',
             '-f', 'h264', 'pipe:1'], capture_output=True, timeout=timeout)
        if encode.returncode != 0 or not encode.stdout:
            return False
        decode = subprocess.run(
            [executable, '-hide_banner', '-loglevel', 'error', *decode_args,
             '-f', 'h264', '-i', 'pipe:0', '-frames:v', '1', '-f', 'null', '-'],
            input=encode.stdout, capture_output=True, timeout=timeout)
        return decode.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def gst_elements(inspect_path='gst-inspect-1.0', names=()):
    """Probe the requested elements one by one; a bare listing is unreliable on L4T."""
    executable = shutil.which(inspect_path)
    if executable is None:
        return set()
    found = set()
    for name in names:
        try:
            probe = subprocess.run([executable, name], capture_output=True, text=True, timeout=10.)
        except (OSError, subprocess.SubprocessError):
            continue
        if probe.returncode == 0 and 'No such element' not in probe.stderr:
            found.add(name)
    return found


def gst_command(settings):
    """NVDEC pipeline writing BGR/RGB frames to stdout."""
    executable = shutil.which(settings.gst_launch_path)
    if executable is None:
        raise ValueError('gst-launch-1.0 not found; install gstreamer1.0-tools')
    caps = 'BGR' if settings.encoding == 'bgr8' else 'RGB'
    return [executable, '-q', 'rtmpsrc', f'location={settings.rtmp_url}', '!', 'flvdemux', '!',
            'h264parse', '!', 'nvv4l2decoder', '!', 'nvvidconv', '!',
            'video/x-raw,format=BGRx,width=%d,height=%d' % (settings.width, settings.height), '!',
            'videoconvert', '!', f'video/x-raw,format={caps}', '!', 'fdsink', 'fd=1']


def select_backend(settings, report=lambda level, text: None):
    """Return (backend name, command). Hardware is used only when a probe succeeded.

    GStreamer's rtmpsrc can only consume an existing RTMP stream; it cannot listen, so
    ``mode=listen`` always uses FFmpeg's ``-listen 1`` receiver.
    """
    gst_missing = sorted(set(GST_REQUIRED) - gst_elements(names=GST_REQUIRED))
    gst_ready = not gst_missing and settings.mode == 'pull'
    if settings.decoder == 'gstreamer':
        if settings.mode != 'pull':
            raise RuntimeError('gstreamer backend needs an RTMP server (mode=pull); '
                               'use decoder=ffmpeg for listen mode')
        if gst_missing:
            raise RuntimeError('gstreamer decoder missing elements: ' + ', '.join(gst_missing))
        return 'gstreamer', gst_command(settings)
    if settings.decoder == 'ffmpeg':
        return 'ffmpeg', ffmpeg_command(settings)
    if settings.decoder == 'ffmpeg-hw':
        args = ('-c:v', 'h264_cuvid')
        if probe_decoder(settings.ffmpeg_path, args):
            return 'ffmpeg-hw', ffmpeg_command(settings, args)
        raise RuntimeError('ffmpeg-hw requested but h264_cuvid cannot decode')
    # auto: verified FFmpeg hardware -> NVDEC via GStreamer (pull only) -> software FFmpeg.
    hw_args = ('-c:v', 'h264_cuvid')
    if probe_decoder(settings.ffmpeg_path, hw_args):
        report('info', 'decoder auto-select: ffmpeg-hw (h264_cuvid probe passed)')
        return 'ffmpeg-hw', ffmpeg_command(settings, hw_args)
    if gst_ready:
        report('info', 'decoder auto-select: gstreamer (nvv4l2decoder)')
        return 'gstreamer', gst_command(settings)
    if settings.mode != 'pull':
        report('info', 'decoder auto-select: ffmpeg software decode '
                       '(listen mode: GStreamer cannot act as an RTMP server)')
    else:
        report('info', 'decoder auto-select: ffmpeg software decode '
                       f'(cuvid probe failed; gstreamer missing {", ".join(gst_missing)})')
    return 'ffmpeg', ffmpeg_command(settings)


class LatestFrame:
    """One pending frame; a newer frame replaces an unread one instead of queueing."""
    def __init__(self):
        self.lock = threading.Lock()
        self.frame = None
        self.replaced = 0

    def put(self, frame):
        with self.lock:
            if self.frame is not None:
                self.replaced += 1
            self.frame = frame

    def take(self):
        with self.lock:
            frame, self.frame = self.frame, None
            return frame

    def clear(self):
        with self.lock:
            self.frame = None


class StreamWorker:
    def __init__(self, settings, stamp, report):
        self.settings = settings
        self.stamp = stamp
        self.report = report
        self.backend, self.command = select_backend(settings, report)
        self.frames = LatestFrame()
        self.frames_secondary = LatestFrame() if settings.secondary_enabled else None
        self.stop_event = threading.Event()
        self.process_lock = threading.Lock()
        self.process = None
        self.lock = threading.Lock()
        self.decode_latency_ms = 0.
        self.frame_count = 0
        self.byte_count = 0
        self.first_frame_time = None
        self.last_frame_time = None
        self.frames_delivered = 0
        self.secondary_frame_count = 0
        self.fallback_reason = ''
        self.thread = threading.Thread(target=self.run, name='rtmp_decoder', daemon=True)

    def start(self):
        self.thread.start()

    def _fallback_to_ffmpeg(self, reason):
        """GStreamer can open some RTMP URLs but never deliver a frame (librtmp needs a
        playpath, e.g. rtmp://host:port/app). Switch back to FFmpeg instead of looping."""
        self.backend = 'ffmpeg'
        self.command = ffmpeg_command(self.settings)
        self.fallback_reason = reason
        self.report('warning', f'{reason}; falling back to FFmpeg software decode')

    def run(self):
        attempts = 0
        while not self.stop_event.is_set():
            before = self.frames_delivered
            try:
                self.decode()
            except (OSError, RuntimeError) as exc:
                # Do not expose stream credentials or FFmpeg's URL-bearing stderr.
                self.report('warning', f'Video unavailable ({type(exc).__name__}); retrying')
            finally:
                self.frames.clear()
            if self.backend == 'gstreamer' and self.frames_delivered == before:
                attempts += 1
                if attempts >= 2:
                    self._fallback_to_ffmpeg('gstreamer delivered no frame')
            else:
                attempts = 0
            self.stop_event.wait(self.settings.reconnect_delay_s)

    def snapshot(self):
        """Decoder-side counters since the last call (input fps, per-frame read cost)."""
        now = time.monotonic()
        with self.lock:
            frames, self.frame_count = self.frame_count, 0
            secondary, self.secondary_frame_count = self.secondary_frame_count, 0
            latency, self.decode_latency_ms = self.decode_latency_ms, 0.
            first, self.first_frame_time = self.first_frame_time, None
        span = (now - first) if first is not None else 0.
        fps = frames/span if span > 1e-3 else 0.
        return dict(frames=frames, secondary=secondary, fps=fps, decode_latency_ms=latency,
                    replaced=self.frames.replaced,
                    secondary_replaced=self.frames_secondary.replaced if self.frames_secondary else 0,
                    backend=self.backend, fallback_reason=self.fallback_reason)

    @staticmethod
    def _decimate(payload, width, height, secondary_width, secondary_height):
        """Point-sample the primary frame to the secondary size (integer factors only)."""
        try:
            import numpy as np
        except ImportError:
            return None
        fy = height // secondary_height
        fx = width // secondary_width
        frame = np.frombuffer(payload, dtype=np.uint8)
        if frame.size != width*height*3:
            return None
        small = frame.reshape((height, width, 3))[::fy, ::fx]
        return np.ascontiguousarray(small).tobytes()

    def decode(self):
        command = list(self.command)
        process = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=0)
        with self.process_lock:
            self.process = process
        selector = selectors.DefaultSelector()
        primary_fd = process.stdout.fileno()
        sinks = {primary_fd: {'file': process.stdout, 'bytes': self.settings.frame_bytes,
                              'latest': self.frames, 'buffer': bytearray(), 'offset': 0}}
        frame_started = {primary_fd: None}
        last_frame = time.monotonic()
        first = True
        try:
            for pipe in (process.stdout, process.stderr):
                os.set_blocking(pipe.fileno(), False)
                selector.register(pipe, selectors.EVENT_READ)
            while not self.stop_event.is_set():
                if time.monotonic()-last_frame > self.settings.read_timeout_s:
                    self.report('warning', 'No complete video frame before timeout; reconnecting')
                    break
                ended = False
                for key, _ in selector.select(timeout=.1):
                    fd = key.fileobj.fileno()
                    chunk = os.read(fd, self.settings.read_chunk_bytes)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        if key.fileobj is process.stdout:
                            ended = True
                        continue
                    if key.fileobj is process.stderr:
                        continue  # Drain stderr to prevent a blocked decoder.
                    sink = sinks.get(fd)
                    if sink is None:
                        continue
                    buffer, frame_bytes = sink['buffer'], sink['bytes']
                    if len(buffer) == sink['offset'] and frame_started[fd] is None:
                        frame_started[fd] = time.monotonic()
                    buffer.extend(chunk)
                    while len(buffer)-sink['offset'] >= frame_bytes:
                        payload = bytes(buffer[sink['offset']:sink['offset']+frame_bytes])
                        sink['offset'] += frame_bytes
                        now = time.monotonic()
                        started = frame_started[fd]
                        cost_ms = (now-started)*1000. if started is not None else 0.
                        frame_started[fd] = None
                        stamp = self.stamp()
                        # Publish the small frame first: consumers take the primary then the
                        # secondary, so it must already be present when the primary lands.
                        secondary_ok = False
                        if self.frames_secondary is not None:
                            small = self._decimate(payload, self.settings.width,
                                                   self.settings.height,
                                                   self.settings.secondary_width,
                                                   self.settings.secondary_height)
                            if small is not None:
                                self.frames_secondary.put((small, stamp, now))
                                secondary_ok = True
                        sink['latest'].put((payload, stamp, now))
                        with self.lock:
                            self.byte_count += len(payload)
                            # input/decode fps must describe the primary stream only.
                            self.frame_count += 1
                            self.decode_latency_ms = cost_ms
                            self.frames_delivered += 1
                            if secondary_ok:
                                self.secondary_frame_count += 1
                            if self.first_frame_time is None:
                                self.first_frame_time = now
                            self.last_frame_time = now
                        last_frame = now
                        if first:
                            self.report('info', 'Video connected; complete frames received '
                                                f'(backend={self.backend}, secondary='
                                                f'{"on" if self.frames_secondary else "off"})')
                            first = False
                    if sink['offset'] and (sink['offset'] >= 4*frame_bytes or
                                           sink['offset'] == len(buffer)):
                        del buffer[:sink['offset']]
                        sink['offset'] = 0
                if ended or process.poll() is not None:
                    self.report('warning', 'Video decoder disconnected; reconnecting')
                    break
        finally:
            selector.close()
            self.terminate(process)
            process.stdout.close()
            process.stderr.close()
            with self.process_lock:
                if self.process is process:
                    self.process = None

    @staticmethod
    def terminate(process):
        if process.poll() is None:
            try:
                process.terminate()
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                try:
                    process.kill()
                except ProcessLookupError:
                    pass
                process.wait(timeout=2.)
        else:
            process.wait()

    def close(self):
        self.stop_event.set()
        # The reader owns process teardown; select wakes at most 0.1 s later.
        if self.thread.ident is not None:
            self.thread.join(timeout=4.)
            if self.thread.is_alive():
                raise RuntimeError('RTMP decoder worker did not stop')
        self.frames.clear()
