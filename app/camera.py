"""Camera services: Basler (pypylon) and a folder-based simulator.

Both expose the same interface so the rest of the app never cares which
one is running:

    start() / stop()
    set_mode('live' | 'hardware' | 'software')
    software_trigger()
    latest_frame  -> Frame | None   (most recent frame, any mode)
    on_triggered_frame              (callback fired for frames captured in
                                     hardware/software trigger mode -- these
                                     are the frames that drive an inspection)
"""

from __future__ import annotations

import threading
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np


@dataclass
class Frame:
    image: np.ndarray                 # BGR
    frame_id: int
    timestamp: float                  # time.perf_counter() at arrival
    wall_time: float                  # time.time() at arrival
    triggered: bool = False           # captured via hw/sw trigger (inspection frame)
    source: str = "camera"


class CameraError(Exception):
    pass


class BaseCamera:
    def __init__(self) -> None:
        self.latest_frame: Optional[Frame] = None
        self.on_triggered_frame: Optional[Callable[[Frame], None]] = None
        self.mode: str = "live"
        self.connected: bool = False
        self.last_error: str = ""
        self._frame_counter = 0
        self._lock = threading.Lock()

    # -- interface ---------------------------------------------------------
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def set_mode(self, mode: str) -> None: ...
    def software_trigger(self) -> None: ...

    def get_exposure_gain(self) -> dict:
        return {}

    def set_exposure_gain(self, exposure_us: Optional[float], gain_db: Optional[float]) -> None: ...

    def info(self) -> dict:
        return {
            "connected": self.connected,
            "mode": self.mode,
            "last_error": self.last_error,
            "type": type(self).__name__,
            "debayer_ms": round(getattr(self, "last_debayer_ms", 0.0), 1),
        }

    # -- shared helpers ----------------------------------------------------
    def _publish(self, image: np.ndarray, triggered: bool, source: str) -> Frame:
        with self._lock:
            self._frame_counter += 1
            frame = Frame(
                image=image,
                frame_id=self._frame_counter,
                timestamp=time.perf_counter(),
                wall_time=time.time(),
                triggered=triggered,
                source=source,
            )
            self.latest_frame = frame
        if triggered and self.on_triggered_frame is not None:
            try:
                self.on_triggered_frame(frame)
            except Exception:
                traceback.print_exc()
        return frame


class BaslerCamera(BaseCamera):
    """Basler ace 2 GigE camera via pypylon.

    Modes:
      live      -- free run, throttled frame rate, for setup/preview
      hardware  -- FrameStart trigger on an opto input line (production)
      software  -- FrameStart trigger fired from the app (testing)
    """

    def __init__(self, serial: str = "", trigger_line: str = "Line1",
                 preview_fps: float = 15.0,
                 input_filter_us: float = 1000.0,
                 input_holdoff_us: float = 50000.0) -> None:
        super().__init__()
        self.serial = serial
        self.trigger_line = trigger_line
        self.preview_fps = preview_fps
        # debounce for mechanical switches on the trigger line: the filter
        # delays the trigger by its full duration; hold-off suppresses
        # bounce re-triggers after the first accepted edge
        self.input_filter_us = input_filter_us
        self.input_holdoff_us = input_holdoff_us
        self._camera = None
        self._converter = None
        self._grab_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._mode_lock = threading.Lock()

    def start(self) -> None:
        from pypylon import pylon

        tl = pylon.TlFactory.GetInstance()
        devices = tl.EnumerateDevices()
        if not devices:
            raise CameraError("No Basler camera found on the network")
        dev = None
        if self.serial:
            for d in devices:
                if d.GetSerialNumber() == self.serial:
                    dev = d
                    break
            if dev is None:
                raise CameraError(f"Camera with serial {self.serial} not found")
        else:
            dev = devices[0]

        self._camera = pylon.InstantCamera(tl.CreateDevice(dev))
        self._camera.Open()

        cam = self._camera
        # Bayer format keeps GigE transfer small (1 byte/px); we debayer on
        # the PC which is faster than shipping BGR8 over the wire.
        try:
            cam.PixelFormat.SetValue("BayerRG8")
            self._bayer_code = cv2.COLOR_BayerRG2BGR
        except Exception:
            # fall back to whatever color format the device offers
            self._bayer_code = None
            self._converter = pylon.ImageFormatConverter()
            self._converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self._converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        # Larger packets reduce CPU load (requires jumbo frames on the NIC;
        # verified working on this 5 GbE link). Falls back to default 1500
        # if the camera rejects it.
        try:
            cam.GevSCPSPacketSize.SetValue(8192)
        except Exception:
            pass

        # Deterministic imaging: auto exposure/gain/white-balance drift with
        # ambient light and silently degrade template scores. Lock them all;
        # exposure/gain are set explicitly per recipe.
        for feat in ("ExposureAuto", "GainAuto", "BalanceWhiteAuto"):
            try:
                getattr(cam, feat).SetValue("Off")
            except Exception:
                pass

        self.connected = True
        self._apply_mode(self.mode)
        self._stop_evt.clear()
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._grab_thread is not None:
            self._grab_thread.join(timeout=3.0)
            self._grab_thread = None
        if self._camera is not None:
            try:
                if self._camera.IsGrabbing():
                    self._camera.StopGrabbing()
                self._camera.Close()
            except Exception:
                pass
            self._camera = None
        self.connected = False

    def set_mode(self, mode: str) -> None:
        if mode not in ("live", "hardware", "software"):
            raise ValueError(f"unknown camera mode {mode!r}")
        with self._mode_lock:
            self.mode = mode
            if self._camera is not None and self._camera.IsOpen():
                was_grabbing = self._camera.IsGrabbing()
                if was_grabbing:
                    self._camera.StopGrabbing()
                self._apply_mode(mode)
                if was_grabbing:
                    self._start_grabbing()

    def _apply_mode(self, mode: str) -> None:
        cam = self._camera
        cam.TriggerSelector.SetValue("FrameStart")
        if mode == "live":
            cam.TriggerMode.SetValue("Off")
            try:
                cam.AcquisitionFrameRateEnable.SetValue(True)
                cam.AcquisitionFrameRate.SetValue(self.preview_fps)
            except Exception:
                pass
        elif mode == "hardware":
            cam.TriggerMode.SetValue("On")
            cam.TriggerSource.SetValue(self.trigger_line)
            cam.TriggerActivation.SetValue("RisingEdge")
            # debounce the line (ace 2: BslInputFilterTime / BslInputHoldOffTime)
            try:
                cam.LineSelector.SetValue(self.trigger_line)
                for feat, want in (("BslInputFilterTime", self.input_filter_us),
                                   ("BslInputHoldOffTime", self.input_holdoff_us)):
                    node = getattr(cam, feat, None)
                    if node is not None and want > 0:
                        node.SetValue(max(node.GetMin(), min(want, node.GetMax())))
            except Exception:
                pass
        elif mode == "software":
            cam.TriggerMode.SetValue("On")
            cam.TriggerSource.SetValue("Software")

    def _start_grabbing(self) -> None:
        from pypylon import pylon
        self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def software_trigger(self) -> None:
        if self._camera is None or self.mode != "software":
            raise CameraError("software trigger only valid in 'software' mode")
        self._camera.ExecuteSoftwareTrigger()

    def get_exposure_gain(self) -> dict:
        cam = self._camera
        if cam is None:
            return {}
        out = {}
        try:
            out["exposure_us"] = float(cam.ExposureTime.GetValue())
            out["exposure_min"] = float(cam.ExposureTime.GetMin())
            out["exposure_max"] = min(float(cam.ExposureTime.GetMax()), 1_000_000.0)
        except Exception:
            pass
        try:
            out["gain_db"] = float(cam.Gain.GetValue())
            out["gain_min"] = float(cam.Gain.GetMin())
            out["gain_max"] = float(cam.Gain.GetMax())
        except Exception:
            pass
        return out

    def set_exposure_gain(self, exposure_us=None, gain_db=None) -> None:
        cam = self._camera
        if cam is None:
            return
        if exposure_us is not None:
            cam.ExposureTime.SetValue(float(exposure_us))
        if gain_db is not None:
            cam.Gain.SetValue(float(gain_db))

    def _grab_loop(self) -> None:
        from pypylon import pylon

        try:
            self._start_grabbing()
        except Exception as exc:
            self.last_error = f"StartGrabbing failed: {exc}"
            self.connected = False
            return

        while not self._stop_evt.is_set():
            try:
                if not self._camera.IsGrabbing():
                    time.sleep(0.05)
                    continue
                res = self._camera.RetrieveResult(250, pylon.TimeoutHandling_Return)
                if not res:
                    continue
                try:
                    if not res.GrabSucceeded():
                        self.last_error = f"grab error {res.ErrorCode}: {res.ErrorDescription}"
                        continue
                    t0 = time.perf_counter()
                    if self._bayer_code is not None:
                        img = cv2.cvtColor(res.Array, self._bayer_code)
                    else:
                        img = self._converter.Convert(res).GetArray()
                    self.last_debayer_ms = (time.perf_counter() - t0) * 1000.0
                finally:
                    res.Release()
                triggered = self.mode in ("hardware", "software")
                self._publish(img, triggered=triggered, source="basler")
                self.last_error = ""   # healthy again after a good frame
            except Exception as exc:
                self.last_error = str(exc)
                time.sleep(0.1)


class RtspCamera(BaseCamera):
    """Generic IP camera (RTSP/HTTP stream) via OpenCV -- e.g. a Lorex.
    Good for monitoring/trending zones at seconds timescale; no hardware
    trigger, no locked exposure, ~1-3 s stream latency. 'software' trigger
    republishes the latest frame as an inspection frame; 'hardware' mode
    behaves like live (there is no trigger line)."""

    def __init__(self, url: str, publish_fps: float = 5.0) -> None:
        super().__init__()
        self.url = url
        self.publish_fps = publish_fps
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

    def start(self) -> None:
        if not self.url:
            raise CameraError("no RTSP url configured")
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=4.0)
            self._thread = None
        self.connected = False

    def set_mode(self, mode: str) -> None:
        self.mode = mode

    def software_trigger(self) -> None:
        f = self.latest_frame
        if f is not None:
            self._publish(f.image, triggered=True, source="rtsp")

    def _loop(self) -> None:
        last_pub = 0.0
        while not self._stop_evt.is_set():
            cap = cv2.VideoCapture(self.url)
            if not cap.isOpened():
                self.connected = False
                self.last_error = f"cannot open stream: {self.url}"
                self._stop_evt.wait(5.0)   # reconnect backoff
                continue
            self.connected = True
            self.last_error = ""
            while not self._stop_evt.is_set():
                # read every frame to drain the buffer; publish throttled
                ok, img = cap.read()
                if not ok:
                    self.last_error = "stream dropped; reconnecting"
                    self.connected = False
                    break
                now = time.perf_counter()
                if now - last_pub >= 1.0 / max(self.publish_fps, 0.1):
                    last_pub = now
                    self._publish(img, triggered=False, source="rtsp")
            cap.release()
            self._stop_evt.wait(2.0)


class SimulatedCamera(BaseCamera):
    """Serves images from a folder -- for development without hardware and
    for deterministic testing. 'live' cycles through the folder at low fps;
    'software' trigger serves the next image; 'hardware' behaves like
    software (there is no real trigger line)."""

    def __init__(self, folder: str, fps: float = 5.0) -> None:
        super().__init__()
        self.folder = Path(folder)
        self.fps = fps
        self._files: list[Path] = []
        self._index = 0
        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

    def _scan(self) -> None:
        exts = {".bmp", ".png", ".jpg", ".jpeg", ".tif", ".tiff"}
        self._files = sorted(
            p for p in self.folder.iterdir() if p.suffix.lower() in exts
        ) if self.folder.is_dir() else []

    def start(self) -> None:
        self._scan()
        if not self._files:
            raise CameraError(f"No images found in {self.folder}")
        self.connected = True
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.connected = False

    def set_mode(self, mode: str) -> None:
        self.mode = mode

    def software_trigger(self) -> None:
        self._serve_next(triggered=True)

    def _serve_next(self, triggered: bool) -> None:
        if not self._files:
            return
        path = self._files[self._index % len(self._files)]
        self._index += 1
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            self.last_error = f"could not read {path}"
            return
        self._publish(img, triggered=triggered, source=f"sim:{path.name}")

    def _loop(self) -> None:
        while not self._stop_evt.is_set():
            if self.mode == "live":
                self._serve_next(triggered=False)
            time.sleep(1.0 / max(self.fps, 0.1))
