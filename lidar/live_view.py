"""LIVE VIEW -- camera picture with zone outlines, per-zone presence and
the zone-4 diameter riding along, plus a side panel with the pass in
progress, the last optimizer pairing and the running score.

Reads the camera over RTSP itself and the LiDAR state from
<dump>/live_state.json, which the flow tracker inside interlace_capture.py
rewrites ~10x per second. So it runs ALONGSIDE the capture tool without
touching the LiDAR port. The camera is decoded only while a browser has
/stream open (see Camera); with no viewer the process is idle.

    python lidar/live_view.py                 # http://localhost:8200
    python lidar/live_view.py --port 8201 --dump lidar/z4captures/interlaced
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

ROOT = Path(__file__).resolve().parent.parent
from lidar.camera_url import cam_url  # noqa: E402
CAM_URL = cam_url()                    # from the CAM_URL environment variable
ZONES_JSON = ROOT / "lidar" / "camera_zones.json"
OUT_W, OUT_H = 1024, 768

# camera fan band -> LiDAR zone name (camera map predates the 7-zone renumber)
CAM_TO_LIDAR = {1: "1_OutfeedConveyor", 2: "2_CamWaiting", 3: "3_CamStaging",
                4: "4_TopOfScrew", 5: "6_ScrewDeck", 6: "7_InfeedDeck"}
GAP_TO_LIDAR = "5_MidScrew"          # the don't-care sector between cam 4 and 5
Z5_SPLIT_DEG = 80.2                  # zone 5 = gap sector from here up to the zone-4 edge
SHORT = {"1_OutfeedConveyor": "Z1 outfeed", "2_CamWaiting": "Z2 cam wait",
         "3_CamStaging": "Z3 cam stage", "4_TopOfScrew": "Z4 TOP OF SCREW",
         "5_MidScrew": "Z5 mid screw", "6_ScrewDeck": "Z6 screw deck",
         "7_InfeedDeck": "Z7 infeed"}


class Camera(threading.Thread):
    """Decodes the RTSP stream only while someone is watching /stream.

    Decoding + resizing 640x480 at 30 fps costs 25-35 % of a core on the
    mill PC, and it was paid around the clock whether or not a browser was
    open. Now the MJPEG generator counts its clients (acquire/release) and
    this thread holds the capture open only while that count is > 0,
    dropping it GRACE seconds after the last client leaves. `frame` is
    None whenever the capture is closed, so the generator falls back to the
    blank picture with the zone overlay until the stream is up again.
    """
    GRACE = 5.0          # s to keep the stream open after the last client left

    def __init__(self, url):
        super().__init__(daemon=True)
        self.url = url
        self.frame = None
        self.t = 0.0
        self.clients = 0
        self._lock = threading.Lock()
        self._wanted = threading.Event()   # set whenever a client arrives

    def acquire(self):
        with self._lock:
            self.clients += 1
        self._wanted.set()

    def release(self):
        with self._lock:
            self.clients -= 1

    def run(self):
        os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                              "rtsp_transport;tcp|stimeout;5000000")
        while True:
            while self.clients <= 0:        # nobody watching: no decode at all
                self._wanted.wait()
                self._wanted.clear()
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
            idle_since = None
            while cap.isOpened():
                ok, fr = cap.read()
                if not ok:
                    break
                self.frame = cv2.resize(fr, (OUT_W, OUT_H))
                self.t = time.time()
                if self.clients > 0:
                    idle_since = None
                elif idle_since is None:
                    idle_since = time.time()
                elif time.time() - idle_since > self.GRACE:
                    break
            cap.release()
            self.frame = None
            if self.clients > 0:            # stream dropped while watched: reconnect
                time.sleep(2.0)


class State(threading.Thread):
    def __init__(self, path):
        super().__init__(daemon=True)
        self.path = Path(path)
        self.state = {}
        self._mtime = 0.0

    def run(self):
        while True:
            try:
                m = self.path.stat().st_mtime
                if m != self._mtime:
                    with open(self.path) as f:
                        self.state = json.load(f)
                    self._mtime = m
            except Exception:
                pass
            time.sleep(0.05)


def load_bands():
    cz = json.loads(ZONES_JSON.read_text())
    sx, sy = OUT_W / cz["image_size"][0], OUT_H / cz["image_size"][1]
    apex = (cz["apex"][0] * sx, cz["apex"][1] * sy)
    bands = []
    for b in cz["bands"]:
        if b["zone"] == 1:
            continue        # zone 1 (outfeed) has no presence logic; not drawn (user, 2026-10-15)
        if b["zone"] is not None:
            bands.append((CAM_TO_LIDAR[b["zone"]], b["lo"], b["hi"]))
        elif 59 < b["lo"] < 60:
            # screw-middle gap: only its LEFT portion (next to zone 4) is the
            # zone-5 mid-screw station -- user's red line, 2026-10-15, ~80 deg
            bands.append((GAP_TO_LIDAR, Z5_SPLIT_DEG, b["hi"]))
    return apex, bands, (sx, sy)


def sector(apex, lo, hi, r=1800, steps=12):
    pts = [apex]
    for k in range(steps + 1):
        a = math.radians(lo + (hi - lo) * k / steps)
        pts.append((apex[0] + r * math.cos(a), apex[1] + r * math.sin(a)))
    return np.array(pts, np.int32)


def draw(frame, state, apex, bands):
    img = frame.copy()
    overlay = img.copy()
    zones = state.get("zones", {})
    stale = (time.time() - state.get("t", 0)) > 2.0
    for name, lo, hi in bands:
        z = zones.get(name)
        present = bool(z and z.get("present")) and not stale
        moving = bool(z and z.get("cam_moving")) and not stale
        if moving:
            col = (0, 200, 255)                         # amber: cam sweeping, no call
        elif name == "4_TopOfScrew":
            col = (0, 220, 0) if present else (0, 180, 255)
        else:
            col = (0, 200, 0) if present else (120, 120, 120)
        poly = sector(apex, lo, hi)
        if present or moving:
            cv2.fillPoly(overlay, [poly], col)
        cv2.polylines(img, [poly], True, col, 2)
    cv2.addWeighted(overlay, 0.22, img, 0.78, 0, img)
    # labels at a fixed radius along each band's bisector
    LABEL_R = {"1_OutfeedConveyor": 540, "2_CamWaiting": 600, "3_CamStaging": 660,
               "4_TopOfScrew": 470, "5_MidScrew": 560, "6_ScrewDeck": 540,
               "7_InfeedDeck": 500}
    for name, lo, hi in bands:
        a = math.radians((lo + hi) / 2)
        r = LABEL_R.get(name, 520)
        x, y = int(apex[0] + r * math.cos(a)), int(apex[1] + r * math.sin(a))
        z = zones.get(name)
        present = bool(z and z.get("present")) and not stale
        moving = bool(z and z.get("cam_moving")) and not stale
        txt = SHORT[name]
        if moving:
            txt += "  cam moving"
        if present and name != "5_MidScrew" and z.get("dia"):
            txt += f"  {z['dia']} mm"
        if present and z.get("y") is not None:
            txt += f"  (y{z['y']:+.2f})"       # LiDAR along-deck position, for band calibration
        raw = state.get("raw", {}).get(name)
        if not present and not moving and raw and raw.get("arcs"):
            ytxt = f" y{raw['y']:+.2f}" if raw.get("y") is not None else ""
            txt += f"  [{raw['arcs']} arcs{ytxt}]"   # what the zone sees while not calling presence
        col = (0, 200, 255) if moving else (0, 255, 0) if present else (200, 200, 200)
        _label(img, txt, (x, y), col, 0.55 if name != "4_TopOfScrew" else 0.7)
    # zone-4 headline
    cur = state.get("pass_in_progress")
    live = state.get("z4_live")
    if cur and not stale:
        head = f"ZONE 4  {cur['dia']} mm   (median of {cur['n']} frames, " \
               f"{'moving' if abs(cur['travel_mm']) > 60 else 'stopped'})"
        hcol = (0, 255, 0)
    elif live and not stale:
        head = f"ZONE 4  {live} mm"
        hcol = (0, 255, 0)
    else:
        head = "ZONE 4  -- empty --" if not stale else "LiDAR state stale"
        hcol = (0, 180, 255) if not stale else (0, 0, 255)
    _label(img, head, (20, 40), hcol, 0.9, thick=2)
    lp = state.get("last_pair")
    if lp:
        _label(img, f"last paired: LiDAR {lp['lidar']} vs optimizer "
                    f"{lp['sed']}/{lp['led']} (band err {lp['band_err']:+.0f} mm)",
               (20, 72), (255, 255, 255), 0.6)
    sc = state.get("score")
    if sc and sc.get("paired"):
        _label(img, f"last {sc['window']} passes: {sc['inside']}/{sc['paired']} inside "
                    f"optimizer band, mean err {sc['mae']} mm",
               (20, 98), (255, 255, 255), 0.6)
    _label(img, time.strftime("%H:%M:%S"), (OUT_W - 130, OUT_H - 16), (255, 255, 255), 0.6)
    if state.get("cam_pos") is not None:
        _label(img, f"cam {state['cam_pos']:.0f} deg{'  MOVING' if state.get('cam_moving') else ''}",
               (20, OUT_H - 16), (0, 200, 255) if state.get("cam_moving") else (255, 255, 255), 0.6)
    return img


def _label(img, txt, org, col, scale, thick=1):
    (w, h), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, scale, thick + 1)
    x, y = org
    cv2.rectangle(img, (x - 4, y - h - 6), (x + w + 4, y + 6), (0, 0, 0), -1)
    cv2.putText(img, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, thick + 1, cv2.LINE_AA)


PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>Log Deck Live</title>
<style>
body{margin:0;background:#111;color:#eee;font-family:Segoe UI,Arial,sans-serif;display:flex;height:100vh}
#cam{flex:1;display:flex;align-items:center;justify-content:center;background:#000}
#cam img{max-width:100%;max-height:100vh}
#panel{width:340px;padding:18px;box-sizing:border-box;border-left:1px solid #333;overflow:auto}
h1{font-size:16px;margin:0 0 12px;color:#9cf;letter-spacing:1px}
.big{font-size:54px;font-weight:700;color:#4f4;line-height:1}
.sub{color:#aaa;font-size:13px;margin-bottom:16px}
.row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #222;font-size:14px}
.row b{color:#fff}
.zone{display:flex;align-items:center;gap:8px;padding:4px 0;font-size:14px}
.dot{width:12px;height:12px;border-radius:6px;background:#444}
.on{background:#4f4;box-shadow:0 0 8px #4f4}
.stale{color:#f66}
</style></head><body>
<div id="cam"><img id="live" src="/stream" onerror="setTimeout(()=>{this.src='/stream?r='+Date.now()},1000)"></div>
<div id="panel">
<h1>LOG DECK LIDAR &mdash; LIVE</h1>
<div class="big" id="z4">--</div>
<div class="sub" id="z4sub">zone 4 diameter</div>
<div id="zones"></div>
<div style="height:14px"></div>
<div class="row"><span>last pass</span><b id="lastpass">--</b></div>
<div class="row"><span>last optimizer pairing</span><b id="lastpair">--</b></div>
<div class="row"><span>band error</span><b id="banderr">--</b></div>
<div class="row"><span>inside band (recent)</span><b id="score">--</b></div>
<div class="row"><span>mean error</span><b id="mae">--</b></div>
<div class="row"><span>passes this run</span><b id="total">--</b></div>
<div class="sub" id="ts" style="margin-top:14px"></div>
</div>
<script>
const names={"1_OutfeedConveyor":"Z1 outfeed","2_CamWaiting":"Z2 cam waiting","3_CamStaging":"Z3 cam staging",
"4_TopOfScrew":"Z4 top of screw","5_MidScrew":"Z5 mid screw","6_ScrewDeck":"Z6 screw deck","7_InfeedDeck":"Z7 infeed"};
async function tick(){
 try{
  const s=await (await fetch('/state')).json();
  const stale=(Date.now()/1000-s.t)>2;
  const cur=s.pass_in_progress;
  const z4=document.getElementById('z4');
  if(stale){z4.textContent='--';z4.className='big stale';document.getElementById('z4sub').textContent='LiDAR state stale';}
  else if(cur){z4.textContent=cur.dia+' mm';z4.className='big';
   document.getElementById('z4sub').textContent='zone 4 · '+cur.n+' frames · '+(Math.abs(cur.travel_mm)>60?'moving':'stopped');}
  else if(s.z4_live){z4.textContent=s.z4_live+' mm';z4.className='big';document.getElementById('z4sub').textContent='zone 4';}
  else {z4.textContent='empty';z4.className='big';z4.style.color='#fa0';document.getElementById('z4sub').textContent='zone 4';}
  if(!stale && (cur||s.z4_live)) z4.style.color='#4f4';
  let h='';
  for(const k of Object.keys(names)){const z=s.zones&&s.zones[k];const on=z&&z.present&&!stale;
   h+='<div class="zone"><div class="dot '+(on?'on':'')+'"></div><span>'+names[k]+'</span>'+(on&&k!='5_MidScrew'?'<span style="margin-left:auto;color:#4f4">'+z.dia+' mm</span>':'')+'</div>';}
  document.getElementById('zones').innerHTML=h;
  const lp=s.last_pass; document.getElementById('lastpass').textContent=lp?(lp.dia+' mm · '+(lp.moving?'moving':'stopped')+' · '+lp.n+' fr'):'--';
  const pr=s.last_pair; document.getElementById('lastpair').textContent=pr?('LiDAR '+pr.lidar+' vs '+pr.sed+'/'+pr.led):'--';
  document.getElementById('banderr').textContent=pr?((pr.band_err>=0?'+':'')+pr.band_err+' mm'):'--';
  const sc=s.score; document.getElementById('score').textContent=sc&&sc.paired?(sc.inside+' / '+sc.paired):'--';
  document.getElementById('mae').textContent=sc&&sc.mae!=null?(sc.mae+' mm'):'--';
  document.getElementById('total').textContent=s.passes_total??'--';
  document.getElementById('ts').textContent='LiDAR state '+(s.iso||'')+(stale?' (stale)':'');
 }catch(e){}
}
setInterval(tick,300);tick();
</script></body></html>"""


def main():
    from fastapi import FastAPI
    from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
    import uvicorn

    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8200)
    ap.add_argument("--dump", default="lidar/z4captures/interlaced")
    ap.add_argument("--cam", default=CAM_URL,
                    help="RTSP URL, or just a stream name (e.g. cam0_1) on the CAM_URL camera")
    args = ap.parse_args()

    apex, bands, _ = load_bands()
    cam = Camera(cam_url(args.cam)); cam.start()
    st = State(Path(args.dump) / "live_state.json"); st.start()
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    def index():
        return PAGE

    @app.get("/state")
    def state():
        return JSONResponse(st.state)

    def gen():
        blank = np.zeros((OUT_H, OUT_W, 3), np.uint8)
        while True:
            fr = cam.frame
            waiting = fr is None            # capture not open (yet): overlay on black
            if waiting:
                fr = blank
            try:
                img = draw(fr, st.state, apex, bands)
            except Exception as e:          # noqa: BLE001 -- keep streaming the bare camera frame
                n = gen.errors = getattr(gen, "errors", 0) + 1
                if n in (1, 10, 100) or n % 1000 == 0:
                    print(f"live view: draw failed ({n}x): {type(e).__name__}: {e}", flush=True)
                img = fr.copy()
            if waiting:
                _label(img, "camera starting...", (OUT_W // 2 - 110, OUT_H // 2),
                       (200, 200, 200), 0.8)
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
            time.sleep(0.1)

    class CountedStream(StreamingResponse):
        """StreamingResponse that counts itself as a camera client for as long
        as it is being sent. The count is NOT kept in gen(): on a client
        disconnect Starlette cancels the sending task and abandons the sync
        generator without closing it, so a `finally` in gen() does not run
        (measured: the decode kept going at 25 % after the browser left).
        """
        async def __call__(self, scope, receive, send):
            cam.acquire()                   # starts the RTSP decode if it is not running
            try:
                await super().__call__(scope, receive, send)
            finally:
                cam.release()               # Camera drops the stream GRACE s later
                try:
                    await self.body_iterator.aclose()   # free the abandoned generator now
                except Exception:           # noqa: BLE001
                    pass

    @app.get("/stream")
    def stream():
        return CountedStream(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    print(f"live view: http://localhost:{args.port}", flush=True)
    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
