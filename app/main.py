"""FastAPI server: web UI + REST + MJPEG stream + websocket."""

from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .inspection import Roi, annotate
from .recipes import Recipe
from .state import STATE, ROOT, save_config

app = FastAPI(title="Vision Inspection")

STATIC_DIR = Path(__file__).resolve().parent / "static"


@app.on_event("startup")
def _startup() -> None:
    STATE.start()


@app.on_event("shutdown")
def _shutdown() -> None:
    STATE.shutdown()


# ------------------------------------------------------------------ pages
@app.get("/")
def index():
    # no-cache so UI updates always reach the browser on refresh
    return FileResponse(STATIC_DIR / "index.html",
                        headers={"Cache-Control": "no-cache"})


app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


# ----------------------------------------------------------------- status
@app.get("/api/status")
def status():
    return STATE.status()


# ----------------------------------------------------------------- camera
class CameraMode(BaseModel):
    mode: str  # live | hardware | software


@app.post("/api/camera/mode")
def camera_mode(body: CameraMode):
    if STATE.camera is None:
        raise HTTPException(503, "camera not started")
    STATE.camera.set_mode(body.mode)
    return {"ok": True, "mode": body.mode}


@app.post("/api/camera/trigger")
def camera_trigger():
    """Software trigger -> full inspection path (PLC write included)."""
    if STATE.camera is None:
        raise HTTPException(503, "camera not started")
    if STATE.camera.mode != "software":
        STATE.camera.set_mode("software")
        time.sleep(0.1)
    STATE.camera.software_trigger()
    return {"ok": True}


class ExposureGain(BaseModel):
    exposure_us: float | None = None
    gain_db: float | None = None


@app.get("/api/camera/exposure")
def get_exposure():
    if STATE.camera is None:
        return {}
    return STATE.camera.get_exposure_gain()


@app.post("/api/camera/exposure")
def set_exposure(body: ExposureGain):
    if STATE.camera is None:
        raise HTTPException(503, "camera not started")
    STATE.camera.set_exposure_gain(body.exposure_us, body.gain_db)
    if STATE.recipe is not None:
        STATE.recipe.camera_settings.update(
            {k: v for k, v in body.model_dump().items() if v is not None})
        STATE.recipes.save(STATE.recipe)
    return {"ok": True}


class CameraConfig(BaseModel):
    source: str | None = None
    serial: str | None = None
    trigger_line: str | None = None
    sim_folder: str | None = None
    rtsp_url: str | None = None


@app.post("/api/camera/config")
def camera_config(body: CameraConfig):
    for k, v in body.model_dump().items():
        if v is not None:
            STATE.config["camera"][k] = v
    save_config(STATE.config)
    STATE._start_camera()
    return STATE.camera.info()


# ------------------------------------------------------------------ frames
def _jpeg(image: np.ndarray, quality: int = 85) -> bytes:
    ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else b""


def _current_annotated(scale: float, draw: bool) -> bytes | None:
    frame = STATE.last_result_frame if (draw and STATE.last_result_frame is not None
                                        and STATE.run_mode) else \
        (STATE.camera.latest_frame if STATE.camera else None)
    if frame is None:
        return None
    img = frame.image
    if draw and STATE.recipe is not None:
        result = STATE.last_result if (STATE.last_result and
                                       STATE.last_result.frame_id == frame.frame_id) else None
        img = annotate(img, STATE.recipe.rois, result)
    if scale != 1.0:
        img = cv2.resize(img, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_AREA)
    return _jpeg(img)


@app.get("/api/frame.jpg")
def frame_jpg(annotated: int = 0, scale: float = 1.0):
    data = _current_annotated(scale, bool(annotated))
    if data is None:
        raise HTTPException(404, "no frame available")
    return Response(content=data, media_type="image/jpeg")


@app.get("/stream")
async def stream(annotated: int = 1, scale: float = 0.5, fps: float = 10.0):
    boundary = "frameboundary"

    async def gen():
        interval = 1.0 / max(fps, 1.0)
        while True:
            data = _current_annotated(scale, bool(annotated))
            if data is not None:
                yield (f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                       f"Content-Length: {len(data)}\r\n\r\n").encode() + data + b"\r\n"
            await asyncio.sleep(interval)

    return StreamingResponse(gen(),
                             media_type=f"multipart/x-mixed-replace; boundary={boundary}")


# ----------------------------------------------------------------- recipes
class RecipeBody(BaseModel):
    rois: list[dict] = []


@app.get("/api/recipes")
def list_recipes():
    return {"recipes": STATE.recipes.list_names(),
            "active": STATE.recipe.name if STATE.recipe else None}


@app.post("/api/recipes/{name}")
def create_recipe(name: str):
    if name in STATE.recipes.list_names():
        raise HTTPException(409, "recipe already exists")
    STATE.recipes.save(Recipe(name=name))
    return {"ok": True}


@app.get("/api/recipes/{name}")
def get_recipe(name: str):
    try:
        return STATE.recipes.load(name).to_dict()
    except FileNotFoundError:
        raise HTTPException(404, "recipe not found")


@app.put("/api/recipes/{name}")
def save_recipe(name: str, body: RecipeBody):
    try:
        recipe = STATE.recipes.load(name)
    except FileNotFoundError:
        raise HTTPException(404, "recipe not found")
    recipe.rois = [Roi.from_dict(d) for d in body.rois]
    STATE.recipes.save(recipe)
    if STATE.recipe is not None and STATE.recipe.name == name:
        STATE.recipe = recipe
    return {"ok": True}


@app.delete("/api/recipes/{name}")
def delete_recipe(name: str):
    STATE.recipes.delete(name)
    if STATE.recipe is not None and STATE.recipe.name == name:
        STATE.recipe = None
        STATE.config["active_recipe"] = ""
        save_config(STATE.config)
    return {"ok": True}


@app.post("/api/recipes/{name}/activate")
def activate_recipe(name: str):
    try:
        STATE.set_active_recipe(name)
    except FileNotFoundError:
        raise HTTPException(404, "recipe not found")
    return {"ok": True, "active": name}


class TeachBody(BaseModel):
    roi_name: str
    rect: list          # [x, y, w, h] region to save as template
    source: str = "live"  # 'live' (camera frame) | 'reference' (recipe image)


@app.post("/api/recipes/{name}/teach")
def teach_template(name: str, body: TeachBody):
    if body.source == "reference":
        try:
            rcp = STATE.recipes.load(name)
        except FileNotFoundError:
            raise HTTPException(404, "recipe not found")
        if not rcp.reference_image:
            raise HTTPException(400, "recipe has no reference image")
        p = Path(rcp.reference_image)
        if not p.is_absolute():
            p = ROOT / p
        image = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if image is None:
            raise HTTPException(400, f"unreadable reference image: {p}")
    else:
        frame = STATE.camera.latest_frame if STATE.camera else None
        if frame is None:
            raise HTTPException(503, "no frame available to teach from")
        image = frame.image
    x, y, w, h = [int(v) for v in body.rect]
    crop = image[y:y + h, x:x + w]
    if crop.size == 0:
        raise HTTPException(400, "teach rect is empty")
    try:
        recipe = STATE.recipes.load(name)
    except FileNotFoundError:
        raise HTTPException(404, "recipe not found")
    fname = STATE.recipes.save_template(name, body.roi_name, crop)
    for roi in recipe.rois:
        if roi.name == body.roi_name:
            roi.template_file = fname
            break
    else:
        raise HTTPException(404, f"ROI {body.roi_name!r} not in recipe")
    STATE.recipes.save(recipe)
    if STATE.recipe is not None and STATE.recipe.name == name:
        STATE.recipe = recipe
    return {"ok": True, "template_file": fname, "size": [w, h]}


class ReferenceBody(BaseModel):
    path: str  # image path relative to the project root ("" clears it)


@app.post("/api/recipes/{name}/reference")
def set_reference(name: str, body: ReferenceBody):
    try:
        recipe = STATE.recipes.load(name)
    except FileNotFoundError:
        raise HTTPException(404, "recipe not found")
    if body.path:
        p = Path(body.path)
        if not p.is_absolute():
            p = ROOT / p
        if not p.exists():
            raise HTTPException(404, f"image not found: {p}")
    recipe.reference_image = body.path
    STATE.recipes.save(recipe)
    if STATE.recipe is not None and STATE.recipe.name == name:
        STATE.recipe = recipe
    return {"ok": True, "reference_image": recipe.reference_image}


@app.get("/api/recipes/{name}/reference.jpg")
def get_reference(name: str, annotated: int = 1, scale: float = 0.5):
    try:
        recipe = STATE.recipes.load(name)
    except FileNotFoundError:
        raise HTTPException(404, "recipe not found")
    if not recipe.reference_image:
        raise HTTPException(404, "recipe has no reference image")
    p = Path(recipe.reference_image)
    if not p.is_absolute():
        p = ROOT / p
    img = cv2.imread(str(p), cv2.IMREAD_COLOR)
    if img is None:
        raise HTTPException(400, f"unreadable image: {p}")
    if annotated:
        result = STATE.engine.run(img, recipe.rois,
                                  str(STATE.recipes.folder(name)))
        img = annotate(img, recipe.rois, result)
    if scale != 1.0:
        img = cv2.resize(img, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_AREA)
    return Response(content=_jpeg(img), media_type="image/jpeg")


@app.get("/api/recipes/{name}/template/{roi_name}")
def get_template_image(name: str, roi_name: str):
    try:
        recipe = STATE.recipes.load(name)
    except FileNotFoundError:
        raise HTTPException(404, "recipe not found")
    for roi in recipe.rois:
        if roi.name == roi_name and roi.template_file:
            path = STATE.recipes.folder(name) / roi.template_file
            if path.exists():
                return FileResponse(path)
    raise HTTPException(404, "no template taught for this ROI")


# --------------------------------------------------------------- inspection
@app.post("/api/inspect/test")
def inspect_test(source: str = "live"):
    """Run the active recipe against the current frame (or the recipe's
    reference image). No PLC write."""
    if STATE.recipe is None:
        raise HTTPException(400, "no active recipe")
    if source == "reference":
        if not STATE.recipe.reference_image:
            raise HTTPException(400, "recipe has no reference image")
        p = Path(STATE.recipe.reference_image)
        if not p.is_absolute():
            p = ROOT / p
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            raise HTTPException(400, f"unreadable reference image: {p}")
        result = STATE.engine.run(img, STATE.recipe.rois, STATE.template_dir())
        return result.to_dict()
    frame = STATE.camera.latest_frame if STATE.camera else None
    if frame is None:
        raise HTTPException(503, "no frame available")
    result = STATE.engine.run(frame.image, STATE.recipe.rois, STATE.template_dir())
    STATE.last_result = result
    STATE.last_result_frame = frame
    result.frame_id = frame.frame_id
    return result.to_dict()


class RunBody(BaseModel):
    enabled: bool


@app.post("/api/run")
def set_run(body: RunBody):
    if body.enabled and STATE.recipe is None:
        raise HTTPException(400, "activate a recipe before starting run mode")
    STATE.run_mode = body.enabled
    if STATE.camera is not None:
        STATE.camera.set_mode("hardware" if body.enabled else "live")
    return {"ok": True, "run_mode": STATE.run_mode}


@app.post("/api/counters/reset")
def reset_counters():
    STATE.counters = {"total": 0, "pass": 0, "fail": 0}
    return {"ok": True}


# ------------------------------------------------------------------ logging
class LoggingBody(BaseModel):
    mode: str          # off | failures | next_n
    count: int = 0


@app.post("/api/logging")
def set_logging(body: LoggingBody):
    if body.mode not in ("off", "failures", "next_n"):
        raise HTTPException(400, "mode must be off | failures | next_n")
    STATE.logger.set_mode(body.mode, body.count)
    return STATE.logger.status()


# ------------------------------------------------------------------- review
class ReviewBody(BaseModel):
    folder: str


@app.post("/api/review/run")
def review_run(body: ReviewBody):
    """Run the active recipe over every image in a folder."""
    if STATE.recipe is None:
        raise HTTPException(400, "no active recipe")
    folder = Path(body.folder)
    if not folder.is_absolute():
        folder = ROOT / folder
    if not folder.is_dir():
        raise HTTPException(404, f"folder not found: {folder}")
    exts = {".bmp", ".png", ".jpg", ".jpeg", ".tif", ".tiff"}
    out = []
    for p in sorted(folder.iterdir()):
        if p.suffix.lower() not in exts:
            continue
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            out.append({"file": p.name, "error": "unreadable"})
            continue
        result = STATE.engine.run(img, STATE.recipe.rois, STATE.template_dir())
        d = result.to_dict()
        d["file"] = p.name
        out.append(d)
    return {"folder": str(folder), "count": len(out), "results": out}


@app.get("/api/review/image")
def review_image(folder: str, file: str, annotated: int = 1, scale: float = 0.5):
    fpath = Path(folder)
    if not fpath.is_absolute():
        fpath = ROOT / fpath
    path = (fpath / file).resolve()
    if not path.exists():
        raise HTTPException(404, "image not found")
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise HTTPException(400, "unreadable image")
    if annotated and STATE.recipe is not None:
        result = STATE.engine.run(img, STATE.recipe.rois, STATE.template_dir())
        img = annotate(img, STATE.recipe.rois, result)
    if scale != 1.0:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return Response(content=_jpeg(img), media_type="image/jpeg")


# ------------------------------------------------------------------- labels
def _resolve_folder(folder: str) -> Path:
    p = Path(folder)
    if not p.is_absolute():
        p = ROOT / p
    if not p.is_dir():
        raise HTTPException(404, f"folder not found: {p}")
    return p


def _load_labels(folder: Path) -> dict:
    f = folder / "labels.json"
    if f.exists():
        try:
            return json.loads(f.read_text(encoding="utf-8"))
        except Exception:
            pass
    return {"categories": [], "labels": {}}


@app.get("/api/labels")
def get_labels(folder: str):
    """Ground-truth labels for a review folder (stored as labels.json
    beside the images). categories: [{name, expect: ''|'pass'|'fail'}],
    labels: {filename: category_name}."""
    return _load_labels(_resolve_folder(folder))


class LabelsBody(BaseModel):
    folder: str
    categories: list | None = None    # full replacement list, if provided
    set_label: dict | None = None     # {"file": ..., "category": ""} clears


@app.post("/api/labels")
def post_labels(body: LabelsBody):
    fol = _resolve_folder(body.folder)
    doc = _load_labels(fol)
    if body.categories is not None:
        doc["categories"] = body.categories
    if body.set_label is not None:
        fname = body.set_label.get("file", "")
        cat = body.set_label.get("category", "")
        if fname:
            if cat:
                doc["labels"][fname] = cat
            else:
                doc["labels"].pop(fname, None)
    (fol / "labels.json").write_text(json.dumps(doc, indent=2),
                                     encoding="utf-8")
    return doc


# --------------------------------------------------------------------- plc
class PlcConfig(BaseModel):
    backend: str | None = None       # mock | ads | eip
    ams_net_id: str | None = None
    ip_address: str | None = None
    port: int | None = None
    slot: int | None = None          # eip only


@app.post("/api/plc/config")
def plc_config(body: PlcConfig):
    for k, v in body.model_dump().items():
        if v is not None:
            STATE.config["plc"][k] = v
    save_config(STATE.config)
    STATE._start_plc()
    return STATE.plc.info()


@app.post("/api/plc/test")
def plc_test():
    """Write a dummy result so the PLC side can be checked end to end."""
    from .inspection import RoiResult, Match
    dummy = [RoiResult(name="test", kind="template", passed=True,
                       matches=[Match(x=1224, y=1024, score=0.99)])]
    STATE.result_id += 1
    try:
        STATE.plc.write_result(STATE.result_id, True, dummy, 1.0)
    except Exception as exc:
        raise HTTPException(502, f"PLC write failed: {exc}")
    return {"ok": True, "result_id": STATE.result_id, "plc": STATE.plc.info()}


# --------------------------------------------------------------- websocket
@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    import queue as _queue
    await ws.accept()
    q: "_queue.Queue[str]" = _queue.Queue(maxsize=64)  # thread-safe: results
    STATE.register_ws(q)                               # arrive from worker threads
    try:
        last_status = 0.0
        while True:
            sent = False
            try:
                while True:
                    await ws.send_text(q.get_nowait())
                    sent = True
            except _queue.Empty:
                pass
            now = time.time()
            if not sent and now - last_status > 2.0:
                await ws.send_text(json.dumps(
                    {"type": "status", "data": STATE.status()}))
                last_status = now
            await asyncio.sleep(0.1)
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        STATE.unregister_ws(q)
