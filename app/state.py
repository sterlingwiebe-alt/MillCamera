"""Application state and orchestration.

The flow in production:
  PLC digital output -> camera hardware trigger -> frame arrives here ->
  inspection -> ADS write to CX8200 -> UI broadcast -> optional logging
"""

from __future__ import annotations

import json
import threading
import time
import traceback
from pathlib import Path
from queue import Queue, Empty
from typing import Optional

from .camera import BaseCamera, BaslerCamera, RtspCamera, SimulatedCamera, Frame
from .inspection import InspectionEngine, InspectionResult, Roi, annotate
from .logging_store import InspectionLogger
from .plc import AdsPlc, BasePlc, MockPlc, RockwellPlc
from .recipes import Recipe, RecipeStore

ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = ROOT / "config.json"

DEFAULT_CONFIG = {
    "camera": {
        "source": "basler",            # 'basler' | 'sim'
        "serial": "",                  # empty = first camera found
        "trigger_line": "Line1",
        "preview_fps": 15.0,
        "input_filter_us": 1000.0,
        "input_holdoff_us": 50000.0,
        "sim_folder": "Sample Images",
        "rtsp_url": "",
    },
    "plc": {
        "backend": "mock",             # 'mock' | 'ads' | 'eip'
        "ams_net_id": "",              # ADS: e.g. '5.51.123.86.1.1'
        "ip_address": "",              # ADS: CX IP / EIP: controller IP
        "port": 851,                   # ADS port
        "slot": 0,                     # EIP: controller slot (0 = embedded port)
        "heartbeat_interval_s": 1.0,
    },
    "active_recipe": "",
}


def load_config() -> dict:
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))  # deep copy
    if CONFIG_PATH.exists():
        try:
            user = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            for section, values in user.items():
                if isinstance(values, dict) and section in cfg:
                    cfg[section].update(values)
                else:
                    cfg[section] = values
        except Exception:
            traceback.print_exc()
    return cfg


def save_config(cfg: dict) -> None:
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2), encoding="utf-8")


class AppState:
    def __init__(self) -> None:
        self.config = load_config()
        self.recipes = RecipeStore(str(ROOT / "recipes"))
        self.engine = InspectionEngine()
        self.logger = InspectionLogger(str(ROOT / "logs"))
        self.camera: Optional[BaseCamera] = None
        self.plc: BasePlc = MockPlc()
        self.recipe: Optional[Recipe] = None
        self.result_id = 0
        self.last_result: Optional[InspectionResult] = None
        self.last_result_frame: Optional[Frame] = None
        self.run_mode = False          # True = inspecting triggered frames
        self.counters = {"total": 0, "pass": 0, "fail": 0}
        self._inspect_queue: "Queue[Frame]" = Queue(maxsize=4)
        self._ws_clients: list = []    # asyncio queues for websocket push
        self._ws_lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ boot
    def start(self) -> None:
        self._start_camera()
        self._start_plc()
        active = self.config.get("active_recipe", "")
        if active and active in self.recipes.list_names():
            self.set_active_recipe(active)
        self._stop_evt.clear()
        self._worker = threading.Thread(target=self._inspect_loop, daemon=True)
        self._worker.start()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def shutdown(self) -> None:
        self._stop_evt.set()
        if self.camera is not None:
            self.camera.stop()
        self.plc.close()

    def _start_camera(self) -> None:
        cam_cfg = self.config["camera"]
        if self.camera is not None:
            self.camera.stop()
        if cam_cfg["source"] == "sim":
            folder = ROOT / cam_cfg.get("sim_folder", "Sample Images")
            self.camera = SimulatedCamera(str(folder))
        elif cam_cfg["source"] == "rtsp":
            self.camera = RtspCamera(cam_cfg.get("rtsp_url", ""),
                                     publish_fps=cam_cfg.get("preview_fps", 5.0))
        else:
            self.camera = BaslerCamera(
                serial=cam_cfg.get("serial", ""),
                trigger_line=cam_cfg.get("trigger_line", "Line1"),
                preview_fps=cam_cfg.get("preview_fps", 15.0),
                input_filter_us=cam_cfg.get("input_filter_us", 5000.0),
                input_holdoff_us=cam_cfg.get("input_holdoff_us", 50000.0),
            )
        self.camera.on_triggered_frame = self._on_triggered_frame
        try:
            self.camera.start()
        except Exception as exc:
            self.camera.last_error = str(exc)
            traceback.print_exc()

    def _start_plc(self) -> None:
        plc_cfg = self.config["plc"]
        try:
            self.plc.close()
        except Exception:
            pass
        if plc_cfg["backend"] == "ads" and plc_cfg.get("ams_net_id"):
            self.plc = AdsPlc(plc_cfg["ams_net_id"], plc_cfg.get("ip_address", ""),
                              plc_cfg.get("port", 851))
        elif plc_cfg["backend"] == "eip" and plc_cfg.get("ip_address"):
            self.plc = RockwellPlc(plc_cfg["ip_address"],
                                   plc_cfg.get("slot", 0))
        else:
            self.plc = MockPlc()
        try:
            self.plc.connect()
        except Exception as exc:
            self.plc.last_error = str(exc)
            traceback.print_exc()

    # ---------------------------------------------------------------- recipe
    def set_active_recipe(self, name: str) -> None:
        self.recipe = self.recipes.load(name)
        self.config["active_recipe"] = name
        save_config(self.config)
        # apply stored camera settings (exposure/gain) if present
        cs = self.recipe.camera_settings
        if cs and self.camera is not None:
            try:
                self.camera.set_exposure_gain(cs.get("exposure_us"), cs.get("gain_db"))
            except Exception:
                pass

    def template_dir(self) -> str:
        if self.recipe is None:
            return str(ROOT / "recipes")
        return str(self.recipes.folder(self.recipe.name))

    # --------------------------------------------------------------- trigger
    def _on_triggered_frame(self, frame: Frame) -> None:
        """Called from the camera grab thread -- keep it fast."""
        if not self.run_mode or self.recipe is None:
            return
        try:
            self._inspect_queue.put_nowait(frame)
        except Exception:
            # queue full: machine is triggering faster than we inspect
            pass

    def _inspect_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                frame = self._inspect_queue.get(timeout=0.25)
            except Empty:
                continue
            try:
                self.inspect_frame(frame, write_plc=True)
            except Exception:
                traceback.print_exc()

    def inspect_frame(self, frame: Frame, write_plc: bool) -> InspectionResult:
        if self.recipe is None:
            raise RuntimeError("no active recipe")
        result = self.engine.run(frame.image, self.recipe.rois, self.template_dir())
        self.result_id += 1
        result.result_id = self.result_id
        result.frame_id = frame.frame_id
        result.wall_time = frame.wall_time
        result.source = frame.source
        self.last_result = result
        self.last_result_frame = frame
        self.counters["total"] += 1
        self.counters["pass" if result.passed else "fail"] += 1
        if write_plc:
            try:
                self.plc.write_result(result.result_id, result.passed,
                                      result.rois, result.elapsed_ms)
            except Exception as exc:
                self.plc.last_error = str(exc)
        self.logger.maybe_log(frame.image, result)
        self._broadcast({"type": "result", "data": result.to_dict(),
                         "counters": dict(self.counters)})
        return result

    # ------------------------------------------------------------- heartbeat
    def _heartbeat_loop(self) -> None:
        beat = 0
        interval = float(self.config["plc"].get("heartbeat_interval_s", 1.0))
        while not self._stop_evt.is_set():
            beat += 1
            try:
                if not self.plc.connected:
                    self.plc.connect()      # self-heal transient ADS drops
                    self.plc.last_error = ""
                self.plc.write_heartbeat(beat)
            except Exception as exc:
                if getattr(exc, "err_code", None) == 1808:
                    # link is fine, PLC program just lacks GVL_Vision
                    self.plc.last_error = ("GVL_Vision not found on the PLC "
                                           "(import plc/GVL_Vision.TcGVL and activate)")
                else:
                    self.plc.connected = False
                    self.plc.last_error = str(exc)
            self._stop_evt.wait(interval)

    # ------------------------------------------------------------ websockets
    def register_ws(self, queue) -> None:
        with self._ws_lock:
            self._ws_clients.append(queue)

    def unregister_ws(self, queue) -> None:
        with self._ws_lock:
            if queue in self._ws_clients:
                self._ws_clients.remove(queue)

    def _broadcast(self, message: dict) -> None:
        data = json.dumps(message)
        with self._ws_lock:
            clients = list(self._ws_clients)
        for q in clients:
            try:
                q.put_nowait(data)
            except Exception:
                pass

    # ---------------------------------------------------------------- status
    def status(self) -> dict:
        return {
            "camera": self.camera.info() if self.camera else {"connected": False},
            "plc": self.plc.info(),
            "recipe": self.recipe.name if self.recipe else None,
            "run_mode": self.run_mode,
            "counters": dict(self.counters),
            "logging": self.logger.status(),
            "last_result": self.last_result.to_dict() if self.last_result else None,
        }


STATE = AppState()
