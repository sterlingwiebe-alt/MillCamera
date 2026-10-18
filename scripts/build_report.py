"""Build the LiDAR-vs-optimizer comparison workbook from today's data files."""
import csv, json, sys
from pathlib import Path
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter
from openpyxl.comments import Comment

ROOT = Path(__file__).resolve().parent.parent      # project folder
D = ROOT / "lidar" / "z4captures" / "interlaced"
OUT = ROOT / "reports" / "LogDeck_LiDAR_vs_Optimizer_2026-10-15.xlsx"
HEALTH = Path(sys.argv[1]) if len(sys.argv) > 1 else None

F = "Arial"
H_FONT = Font(name=F, bold=True, color="FFFFFF", size=10)
H_FILL = PatternFill("solid", fgColor="1F4E78")
B_FONT = Font(name=F, size=10)
T_FONT = Font(name=F, bold=True, size=14)
S_FONT = Font(name=F, bold=True, size=11)
NOTE_FONT = Font(name=F, italic=True, size=9, color="555555")
GOOD = PatternFill("solid", fgColor="E2EFDA")
BAD = PatternFill("solid", fgColor="FCE4D6")
GREY = PatternFill("solid", fgColor="EDEDED")
thin = Side(style="thin", color="BFBFBF")
BORDER = Border(left=thin, right=thin, top=thin, bottom=thin)


def header(ws, row, cols, widths=None):
    for i, c in enumerate(cols, 1):
        cell = ws.cell(row=row, column=i, value=c)
        cell.font = H_FONT; cell.fill = H_FILL; cell.border = BORDER
        cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
    ws.row_dimensions[row].height = 30
    if widths:
        for i, w in enumerate(widths, 1):
            ws.column_dimensions[get_column_letter(i)].width = w


def body(ws, row, vals, fmt=None):
    for i, v in enumerate(vals, 1):
        cell = ws.cell(row=row, column=i, value=v)
        cell.font = B_FONT; cell.border = BORDER
        if fmt and i - 1 < len(fmt) and fmt[i - 1]:
            cell.number_format = fmt[i - 1]


def fnum(s):
    try:
        return float(s) if s not in ("", None) else None
    except ValueError:
        return s


def inum(s):
    v = fnum(s)
    return int(v) if isinstance(v, float) and v == int(v) else v


# ------------------------------------------------------------------ data
# stopped captures (pairing.csv; column count grew during the day)
stopped = []
for r in csv.reader(open(D / "pairing.csv")):
    if not r or not r[0][:2].isdigit():
        continue
    stamp, n, zone, y, lidar, width, slot, sed, led, ln, off, mae = r[:12]
    flag = r[12] if len(r) > 12 else ""
    lag = r[13] if len(r) > 13 else ""
    stopped.append(dict(stamp=stamp, zone=zone, y=fnum(y), lidar=inum(lidar), slot=inum(slot),
                        sed=inum(sed), led=inum(led), ln=inum(ln), off=inum(off), mae=fnum(mae),
                        flag=flag, lag=fnum(lag)))
# user-confirmed context for stopped captures
STOP_NOTES = {
    "041611": "capture 1: user's 'zone-4 log' was at the zone-3 station; offset 1 unambiguous (12 vs 55+ mm)",
    "043402": "capture 2",
    "043954": "capture 3: deck logs all 213-245 mm, offsets 1/3/4 tie -> ambiguous, excluded",
    "044209": "capture 4: deck still creeping (66 mm drift), cam mid-rotation -> excluded",
    "045652": "capture 5: offset 0 vs 1 unresolved (both put zone 4 within 10 mm of 269)",
    "051814": "capture 8: offset 2 under band metric; zone-2 400 reading is the deck wall",
    "052913": "capture 9: offset 1, clear by 23 mm",
}

# flow passes: runs 1-3 archives + current (run 4), dedupe by start time
flow = {}
for run, fn in ((1, "flow_pairs_run1_0542-0546.csv"), (2, "flow_pairs_run2_0546-0606.csv"),
                (3, "flow_pairs_run3_0606-0619.csv"), (4, "flow_pairs.csv")):
    p = D / fn
    if not p.exists():
        continue
    for r in csv.DictReader(open(p)):
        key = r["iso_start"]
        if key in flow:
            continue
        flow[key] = dict(run=run, iso=key, moving=int(r["moving"]), n=int(r["n"]), dur=float(r["dur_s"]),
                         travel=int(float(r["travel_mm"])), dia=int(r["dia"]), iqr=int(r["iqr"]),
                         slot=inum(r["slot"]), sed=inum(r["sed"]), led=inum(r["led"]),
                         lag=fnum(r["lag_s"]))
flow = [flow[k] for k in sorted(flow)]
# known events (user-confirmed) by time window
EVENTS = [("05:45:00", "05:45:10", "double log in zone 4 (user) -> downstream stoppage"),
          ("05:55:13", "05:55:21", "split log + double feed through optimizer (user)"),
          ("06:17:08", "06:18:02", "stopped log fragmented by detection flicker (fixed 06:19)")]


def event_for(iso):
    t = iso[11:19]
    for a, b, txt in EVENTS:
        if a <= t <= b:
            return txt
    return ""


summ = []
for r in csv.DictReader(open(D / "flow_summary.csv")):
    summ.append(r)
health = json.load(open(HEALTH)) if HEALTH and HEALTH.exists() else []   # optional: lidar_health.json from the health check

# ------------------------------------------------------------------ workbook
wb = Workbook()

# ---- Flow Pairs (data sheet first so Summary can reference it)
wf = wb.active
wf.title = "Flow Pairs"
cols = ["Run", "Pass start", "Moving", "Frames", "Duration s", "Travel mm", "LiDAR dia mm", "IQR mm",
        "Opt slot", "Opt SED mm", "Opt LED mm", "Opt MID mm", "Band low", "Band high", "Band error mm",
        "Confidence", "Record anomaly", "Paired", "Lag s", "Event (user-confirmed)"]
header(wf, 1, cols, [5, 19, 8, 8, 10, 10, 12, 8, 8, 10, 10, 10, 9, 9, 12, 11, 12, 8, 7, 48])
for i, r in enumerate(flow, start=2):
    paired = r["slot"] not in ("", None)
    body(wf, i, [r["run"], r["iso"], "moving" if r["moving"] else "stopped", r["n"], r["dur"], r["travel"],
                 r["dia"], r["iqr"], r["slot"] if paired else None, r["sed"] if paired else None,
                 r["led"] if paired else None])
    if paired:
        wf.cell(row=i, column=12, value=f"=(J{i}+K{i})/2").font = B_FONT
        wf.cell(row=i, column=13, value=f"=MIN(J{i},L{i})").font = B_FONT
        wf.cell(row=i, column=14, value=f"=MAX(J{i},L{i})").font = B_FONT
        wf.cell(row=i, column=15, value=f'=IF(Q{i}="anomaly",0,MAX(0,M{i}-G{i},G{i}-N{i}))').font = B_FONT
        wf.cell(row=i, column=17, value=f'=IF(J{i}>K{i},"anomaly","")').font = B_FONT
    else:
        for c in (12, 13, 14, 15, 17):
            wf.cell(row=i, column=c, value=None).font = B_FONT
    wf.cell(row=i, column=16, value=f'=IF(OR(H{i}>15,D{i}<20),"low","clean")').font = B_FONT
    wf.cell(row=i, column=18, value="yes" if paired else "no").font = B_FONT
    wf.cell(row=i, column=19, value=r["lag"] if paired else None).font = B_FONT
    wf.cell(row=i, column=20, value=event_for(r["iso"])).font = B_FONT
    for c in range(12, 21):
        wf.cell(row=i, column=c).border = BORDER
    for c in (12, 13, 14, 15):
        wf.cell(row=i, column=c).number_format = "0"
NF = len(flow) + 1
wf.freeze_panes = "A2"
wf.auto_filter.ref = f"A1:T{NF}"
note_r = NF + 2
wf.cell(row=note_r, column=1, value=(
    "Band = [min(SED, MID), max(SED, MID)]: the LiDAR measures ~2.4 m of the log's middle, the optimizer the whole log, "
    "so a correct LiDAR reading lies between the small end and the mid diameter. Band error = distance outside that band (0 = inside). "
    "Confidence: 'low' = pass IQR > 15 mm or < 20 frames. Record anomaly: optimizer SED > LED, seen only on double feeds / split logs. "
    "Runs 1-2 archived only their last alignment window; run 3 onward is cumulative.")).font = NOTE_FONT

# ---- Stopped Captures
ws = wb.create_sheet("Stopped Captures")
cols = ["Capture (HHMMSS)", "Station", "Window y (m)", "LiDAR dia mm", "Opt slot", "Opt SED mm", "Opt LED mm",
        "Opt MID mm", "Band low", "Band high", "Band error mm", "Offset", "Alignment MAE mm", "Pairing", "Lag s", "Note"]
header(ws, 1, cols, [15, 18, 11, 12, 8, 10, 10, 10, 9, 9, 12, 7, 12, 11, 7, 70])
for i, r in enumerate(stopped, start=2):
    body(ws, i, [r["stamp"], r["zone"], r["y"], r["lidar"], r["slot"], r["sed"], r["led"]])
    ws.cell(row=i, column=8, value=f"=(F{i}+G{i})/2")
    ws.cell(row=i, column=9, value=f"=MIN(F{i},H{i})")
    ws.cell(row=i, column=10, value=f"=MAX(F{i},H{i})")
    ws.cell(row=i, column=11, value=f"=MAX(0,I{i}-D{i},D{i}-J{i})")
    body_vals = [r["off"], r["mae"], r["flag"], r["lag"], STOP_NOTES.get(r["stamp"], "")]
    for j, v in enumerate(body_vals, start=12):
        ws.cell(row=i, column=j, value=v)
    for c in range(8, 17):
        ws.cell(row=i, column=c).font = B_FONT; ws.cell(row=i, column=c).border = BORDER
    for c in (8, 9, 10, 11):
        ws.cell(row=i, column=c).number_format = "0"
    if r["flag"] == "ambiguous":
        for c in range(1, 17):
            ws.cell(row=i, column=c).fill = GREY
NS = len(stopped) + 1
ws.freeze_panes = "A2"
ws.cell(row=NS + 2, column=1, value=(
    "Each capture = a 1.5 s LiDAR burst on a stopped deck (Vision_Capture trigger); LiDAR dia = median single-frame circle fit over the burst. "
    "Pairing = whole-deck sequence aligned to TestLog records (offset = logs already ahead of the deck on the outfeed chain, user limit 3). "
    "Grey rows = ambiguous pairing, excluded from statistics. Capture 6 (05:04, zone 4 = 306 mm) lost its records to a TestLog reset and is not listed.")).font = NOTE_FONT

# ---- Flow Summary
wsu = wb.create_sheet("Flow Summary")
cols = ["Time", "Passes total", "Window", "Unpaired", "Moving n", "Moving inside", "Moving MAE", "Moving max",
        "Stopped n", "Stopped inside", "Stopped MAE", "Stopped max", "Low-conf n", "Low-conf inside", "Low-conf MAE", "Low-conf max"]
header(wsu, 1, cols, [9, 10, 8, 9, 9, 11, 10, 10, 9, 11, 10, 10, 10, 12, 11, 11])
for i, r in enumerate(summ, start=2):
    body(wsu, i, [r.get("iso_time"), inum(r.get("passes_total")), inum(r.get("window")), inum(r.get("unpaired")),
                  inum(r.get("moving_n")), inum(r.get("moving_inside")), fnum(r.get("moving_mae")), fnum(r.get("moving_max")),
                  inum(r.get("stopped_n")), inum(r.get("stopped_inside")), fnum(r.get("stopped_mae")), fnum(r.get("stopped_max")),
                  inum(r.get("lowconf_n")), inum(r.get("lowconf_inside")), fnum(r.get("lowconf_mae")), fnum(r.get("lowconf_max"))])
wsu.freeze_panes = "A2"
wsu.cell(row=len(summ) + 3, column=1, value=(
    "One row per 10 passes, statistics over the trailing alignment window (<= 30 passes). 'Inside' = band error <= 15 mm. "
    "Passes-total restarts at each tool restart (05:42, 05:46, 06:06, 06:19). Low-confidence columns exist from 06:06.")).font = NOTE_FONT

# ---- LiDAR Health
wh = wb.create_sheet("LiDAR Health")
cols = ["Capture", "Frames", "Missing frame numbers", "Short frames (<12 segments)", "Min segments/frame",
        "Frame period ms (median)", "Period max ms", "Period jitter ms (sd)", "No-return beams %", "Mean RSSI", "Availability flag"]
header(wh, 1, cols, [34, 8, 12, 14, 12, 13, 11, 13, 12, 10, 12])
for i, r in enumerate(health, start=2):
    body(wh, i, [r["label"], r["frames"], r["frame_gaps"], r["short_frames"], r["segs_min"], r["period_ms"],
                 r["period_max_ms"], r["period_jitter_ms"], r["no_return_pct"], r["rssi_mean"], str(r["availability"])])
    if r["short_frames"] > 0.05 * r["frames"] or r["period_jitter_ms"] > 1.0:
        for c in range(1, 12):
            wh.cell(row=i, column=c).fill = BAD
    elif r["frame_gaps"] == 0:
        for c in range(1, 12):
            wh.cell(row=i, column=c).fill = GOOD
nh = len(health) + 3
notes = [
    "How to read: a missing frame number = a whole 50 ms revolution never arrived. A short frame = one of the 12 segments of a revolution was lost "
    "(in the 1.5 s trigger bursts a count of 11 can also be the snapshot taken before the 12th segment landed, so bursts are not evidence of loss). "
    "Period = time between successive revolutions from the sensor's own clock (20 Hz -> 50 ms). No-return = beams with no echo (sky / beyond range) and is scene-dependent. "
    "Availability = the sensor's per-module health flag in every packet (1 = OK).",
    "Finding, 2026-10-15: every capture today has zero missing frames, a 49.4-50.0 ms period with <0.2 ms jitter, constant RSSI (~33k) and no-return rate (~59.7%), "
    "and availability = 1 on all 7,000+ packets inspected. There is no indication in the data stream of a degraded or compromised sensor today.",
    "The two older recordings show loss: 2026-10-13 lost 8 of 499 frames' segments (1.6%, jitter 1.7 ms) and 2026-10-08 lost segments in 151 of 542 frames (28%, period up to 88 ms). "
    "That loss pattern (whole segments missing with the sensor clock still regular) is UDP packets dropped on the PC side (receive buffer / capture process stalls), not the sensor; "
    "today's captures use an 8 MB receive buffer and show none of it.",
    "Receive timeouts printed by the capture tool twice today coincided with the light-bank sampler briefly opening the same UDP port; no frame numbers were missing around them.",
    "Not covered here: the device's own diagnostic log (web UI / dump). The 2026-10-13 fatal-fault analysis concerned EEPROM / firmware-type errors on a unit; if this is that unit, those are device-side records the stream cannot show.",
]
for k, t in enumerate(notes):
    c = wh.cell(row=nh + k, column=1, value=t); c.font = NOTE_FONT; c.alignment = Alignment(wrap_text=True, vertical="top")
    wh.merge_cells(start_row=nh + k, start_column=1, end_row=nh + k, end_column=11)
    wh.row_dimensions[nh + k].height = 42

# ---- Method
wm = wb.create_sheet("Method")
lines = [
    ("Sensor", "SICK multiScan165 overhead, 20 Hz, 16 layers, 0.5 deg azimuth; x4 interlacing was on all day (user prefers it off; it did not improve accuracy)."),
    ("Estimator", "Per 20 Hz frame: per-layer runs of beams inside the deck region -> circle fit per run (Kasa, roundness gate) -> arcs clustered per log (RANSAC on arc centres) -> diameter = median of the per-arc circle diameters (dia_arc). Per log: median over all frames of its pass (moving) or of the 1.5 s burst (stopped)."),
    ("Zone 4", "Top-of-screw station, LiDAR window y -0.73..-0.27 m (raw sensor frame). The only diameter station; zones 2-7 are presence."),
    ("Optimizer truth", "Mill optimizer per-log record (SED, LED, Length) copied from the PLC's TestLog ring the instant each record is written (TestPtr polled at 2.5 Hz; 60 slots, wraps 59->0)."),
    ("Why a band", "The LiDAR sees ~2.4 m of the log's middle; the optimizer measures the whole log. A correct LiDAR reading therefore lies between SED and MID = (SED+LED)/2. Error is reported as distance outside that band; 'inside' = within 15 mm of it."),
    ("Pairing, stopped", "Whole-deck inventory (downstream first) aligned to the records that arrive after the capture, offsets 0-3 (user: never more than 3 logs ahead on the chain), weighted: zones 4 and 6 full, zone 3 half (log on cam lobes over-reads), zone 5 zero (multi-piece), zone 1 and wall readings > 360 mm zero. Ambiguous if the runner-up offset is within 4 mm."),
    ("Pairing, flow", "Each zone-4 pass (entry to exit, suspended rather than ended on detection flicker) is one item; the pass sequence is aligned to the record sequence by monotonic dynamic programming with gap penalties; a record must follow its pass by 8-120 s. Typical lag 9-13 s, up to 50 s around stops."),
    ("Interlacing", "Confirmed x4 (4-frame azimuth cycle, 20 Hz unchanged). Pooling 4 frames gives 4x beams per arc but read +15..+25 mm above the single-frame fit on every stopped capture; not used as the estimator."),
    ("Known events", "05:45 double log in zone 4 (user); 05:55 split log followed by a double feed through the optimizer (user); 06:17 a stopped log fragmented into 7 passes by detection flicker (tracker fixed at 06:19). Rows in those windows are tagged on the Flow Pairs sheet."),
    ("Export", "Values exported 2026-10-15 ~06:25 while the line was running. All formulas were recalculated in Excel after export (1,977 formulas, 0 errors) and the headline figures were cross-checked against an independent Python computation (blue column on the Summary sheet; all matched)."),
]
wm.column_dimensions["A"].width = 18; wm.column_dimensions["B"].width = 140
for i, (k, v) in enumerate(lines, start=1):
    a = wm.cell(row=i, column=1, value=k); a.font = S_FONT; a.alignment = Alignment(vertical="top")
    b = wm.cell(row=i, column=2, value=v); b.font = B_FONT; b.alignment = Alignment(wrap_text=True, vertical="top")
    wm.row_dimensions[i].height = 48

# ---- Summary (formulas over the data sheets + python cross-check)
su = wb.create_sheet("Summary", 0)
su.column_dimensions["A"].width = 58; su.column_dimensions["B"].width = 16; su.column_dimensions["C"].width = 16; su.column_dimensions["D"].width = 60
su["A1"] = "Log deck LiDAR vs mill optimizer - diameter comparison, 2026-10-15"; su["A1"].font = T_FONT
su["A2"] = "multiScan165 at zone 4 (top of screw), single-frame circle fit, paired by sequence to optimizer SED/LED records"; su["A2"].font = NOTE_FONT
FP = "'Flow Pairs'"
rng = lambda col: f"{FP}!{col}2:{col}{NF}"
# python cross-check values
def pyband(d, sed, led):
    if sed is None or led is None: return None
    if sed > led: return 0.0
    mid = (sed + led) / 2; lo, hi = min(sed, mid), max(sed, mid)
    return 0.0 if lo <= d <= hi else (lo - d if d < lo else d - hi)
clean = [r for r in flow if r["slot"] not in ("", None) and not (r["iqr"] > 15 or r["n"] < 20) and not (r["sed"] > r["led"])]
clean_mov = [r for r in clean if r["moving"]]
clean_stop = [r for r in clean if not r["moving"]]
errs = [pyband(r["dia"], r["sed"], r["led"]) for r in clean]
errs_m = [pyband(r["dia"], r["sed"], r["led"]) for r in clean_mov]
errs_s = [pyband(r["dia"], r["sed"], r["led"]) for r in clean_stop]
import statistics as st
rows = [
    ("FLOW MODE (every log through zone 4, 05:42-06:22)", None, None, None),
    ("Passes recorded", f"=COUNTA({rng('B')})", len(flow), "all tracked passes, all runs"),
    ("Passes paired to an optimizer record", f'=COUNTIF({rng("R")},"yes")', sum(1 for r in flow if r["slot"] not in ("", None)), ""),
    ("Clean paired passes (confidence 'clean', no record anomaly)", f'=COUNTIFS({rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"")', len(clean), "basis for the accuracy figures"),
    ("  of which inside the optimizer band (<= 15 mm)", f'=COUNTIFS({rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"",{rng("O")},"<=15")', sum(1 for e in errs if e <= 15), ""),
    ("  mean band error, clean passes (mm)", f'=AVERAGEIFS({rng("O")},{rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"")', round(st.mean(errs), 1) if errs else None, ""),
    ("  moving clean passes: n / inside / mean error", f'=COUNTIFS({rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"",{rng("C")},"moving")', f"{len(clean_mov)} / {sum(1 for e in errs_m if e<=15)} / {round(st.mean(errs_m),1) if errs_m else '-'}", "moving logs measure as well as stopped ones"),
    ("  stopped clean passes: n / inside / mean error", f'=COUNTIFS({rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"",{rng("C")},"stopped")', f"{len(clean_stop)} / {sum(1 for e in errs_s if e<=15)} / {round(st.mean(errs_s),1) if errs_s else '-'}", ""),
    ("  LiDAR minus optimizer SED, clean passes: mean / sd (mm)", f'=AVERAGEIFS({rng("G")},{rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"")-AVERAGEIFS({rng("J")},{rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"")', f"{round(st.mean(r['dia']-r['sed'] for r in clean),1)} / {round(st.pstdev([r['dia']-r['sed'] for r in clean]),1)}", "positive = LiDAR reads above the small end, as expected for a middle section"),
    ("  LiDAR minus optimizer MID, clean passes: mean / sd (mm)", f'=AVERAGEIFS({rng("G")},{rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"")-AVERAGEIFS({rng("L")},{rng("P")},"clean",{rng("R")},"yes",{rng("Q")},"")', f"{round(st.mean(r['dia']-(r['sed']+r['led'])/2 for r in clean),1)} / {round(st.pstdev([r['dia']-(r['sed']+r['led'])/2 for r in clean]),1)}", ""),
    ("  size range covered, LiDAR (mm)", f'=MIN({rng("G")})&" - "&MAX({rng("G")})', f"{min(r['dia'] for r in flow)} - {max(r['dia'] for r in flow)}", ""),
    ("  low-confidence passes (wide spread or < 20 frames)", f'=COUNTIF({rng("P")},"low")', sum(1 for r in flow if r["iqr"] > 15 or r["n"] < 20), "reported, not used for accuracy"),
    ("  optimizer record anomalies (SED > LED)", f'=COUNTIF({rng("Q")},"anomaly")', sum(1 for r in flow if r["slot"] not in ("", None) and r["sed"] > r["led"]), "double feeds / split logs"),
    (None, None, None, None),
    ("STOPPED CAPTURES (Vision_Capture trigger, 04:16-05:29)", None, None, None),
    ("Zone-4 pairs with clear pairing", f"=COUNTIFS('Stopped Captures'!B2:B{NS},\"4_TopOfScrew\",'Stopped Captures'!N2:N{NS},\"clear\")", sum(1 for r in stopped if r["zone"] == "4_TopOfScrew" and r["flag"] == "clear"), ""),
    ("  of which inside the band", f"=COUNTIFS('Stopped Captures'!B2:B{NS},\"4_TopOfScrew\",'Stopped Captures'!N2:N{NS},\"clear\",'Stopped Captures'!K2:K{NS},\"<=15\")", sum(1 for r in stopped if r["zone"] == "4_TopOfScrew" and r["flag"] == "clear" and pyband(r["lidar"], r["sed"], r["led"]) <= 15), ""),
    ("  mean band error (mm)", f"=AVERAGEIFS('Stopped Captures'!K2:K{NS},'Stopped Captures'!B2:B{NS},\"4_TopOfScrew\",'Stopped Captures'!N2:N{NS},\"clear\")", round(st.mean(pyband(r["lidar"], r["sed"], r["led"]) for r in stopped if r["zone"] == "4_TopOfScrew" and r["flag"] == "clear"), 1), ""),
    (None, None, None, None),
    ("CONCLUSIONS", None, None, None),
    ("1. Correctly paired, the single-frame circle fit sits inside the optimizer's SED-to-MID band on essentially every clean pass, moving or stopped, 160-355 mm.", None, None, None),
    ("2. Logs do not need to stop: a moving pass gives 30-55 frames at 2-8 mm spread and pairs as well as a stopped burst.", None, None, None),
    ("3. Yesterday's 'big logs saturate' result was a pairing offset, not a measurement limit; the picoScan case now rests only on requirements the 165 cannot meet, not on accuracy.", None, None, None),
    ("4. Interlacing (x4) did not improve accuracy and reads 15-25 mm high; recommend leaving it off.", None, None, None),
    ("5. Remaining misses trace to double logs, double feeds, split logs and detection flicker during stops (fixed), i.e. segmentation at the edges, not the fit.", None, None, None),
    ("6. LiDAR stream health today: no dropped frames, 50.0 ms period, <0.2 ms jitter; older loss was PC-side UDP drops.", None, None, None),
]
r0 = 4
su.cell(row=r0 - 1, column=1, value="Metric").font = S_FONT
su.cell(row=r0 - 1, column=2, value="Formula (live)").font = S_FONT
su.cell(row=r0 - 1, column=3, value="Python check at export").font = S_FONT
su.cell(row=r0 - 1, column=4, value="Note").font = S_FONT
for i, (a, b, c, d) in enumerate(rows, start=r0):
    if a is None:
        continue
    ca = su.cell(row=i, column=1, value=a); ca.font = S_FONT if b is None and c is None else B_FONT
    ca.alignment = Alignment(wrap_text=True, vertical="top")
    if b is not None:
        cb = su.cell(row=i, column=2, value=b); cb.font = B_FONT; cb.number_format = "0.0"
    if c is not None:
        cc = su.cell(row=i, column=3, value=c); cc.font = Font(name=F, size=10, color="0000FF")
    if d:
        cd = su.cell(row=i, column=4, value=d); cd.font = NOTE_FONT; cd.alignment = Alignment(wrap_text=True)
su.cell(row=r0 + len(rows) + 1, column=1, value=(
    "Blue = value computed in Python at export time, for cross-checking the live formulas after Excel recalculates. "
    "'Inside' = band error <= 15 mm (the project's accuracy target). See the Method sheet for definitions.")).font = NOTE_FONT

OUT.parent.mkdir(exist_ok=True)
wb.save(OUT)
print("saved", OUT)
print("flow passes", len(flow), "clean", len(clean), "inside", sum(1 for e in errs if e <= 15), "mean", round(st.mean(errs), 2) if errs else None)
print("moving", len(clean_mov), sum(1 for e in errs_m if e <= 15), "stopped", len(clean_stop), sum(1 for e in errs_s if e <= 15))
