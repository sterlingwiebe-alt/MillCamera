"""List every log-radius arc the presence code would consider for a zone in a
burst capture: fitted centre y, diameter, and the y range of the run's own
points. Shows whether an arc 'in the zone' actually has points there.

    python lidar/zone_arcs.py <cap.pkl> [zone] [frames]
"""
import pickle, sys
import numpy as np
sys.path.insert(0, r"C:\Claude")
from lidar.flow_track import FlowTracker, PRES_R_LO, PRES_R_HI   # noqa: E402
from lidar.arcfit import zone_round_runs                            # noqa: E402

pkl, zone = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "2_CamWaiting")
nfr = int(sys.argv[3]) if len(sys.argv) > 3 else 3
d = pickle.load(open(pkl, "rb"))
frames = d["frames"]
print(f"{pkl}: {len(frames)} frames, cam={d.get('cam')}")
bands = {z: (win, wide) for z, win, wide, _ in FlowTracker._presence_bands()}
win, wide = bands[zone]
y0, y1 = win["y"]
print(f"{zone}: tight y {win['y']}  point band y {wide['y']}  x {win['x']}")
for k, (fn, segs) in enumerate(sorted(frames.items())):
    if k >= nfr:
        break
    print(f"--- frame {fn}")
    for run, r, c3 in zone_round_runs(segs, wide):
        if not (PRES_R_LO <= r <= PRES_R_HI):
            continue
        ys = run[:, 1]
        cin = y0 <= c3[1] <= y1
        pin = y0 <= float(np.median(ys)) <= y1
        print(f"  centre y {c3[1]:+.2f} x {c3[0]:+.2f} dia {r*2000:4.0f} mm  n={len(run):2d}  "
              f"points y {ys.min():+.2f}..{ys.max():+.2f} (median {np.median(ys):+.2f})  "
              f"{'CENTRE-IN' if cin else 'centre-out'} {'points-in' if pin else 'POINTS-OUT'}")
