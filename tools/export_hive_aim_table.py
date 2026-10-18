"""Turn the HIVE shot maps into the robot's aim table (HiveAimTableData.java).

Usage (from the repo root):
    python tools/export_hive_aim_table.py <maps dir> [preferred launch deg]
    e.g. python tools/export_hive_aim_table.py "C:/Users/naren/Downloads/30686_render/shooter_analysis/maps_launch43_80"

Input: map_A..map_D.npz from hive_sim_4080.py (launch angle 80..40 deg in 1 deg steps, exit speed bins
3.00 + 0.05*i m/s, 2 in grid of LAUNCH positions in Pedro inches). Keys: <state>_<cell>_up__lo/__hi/__tfl
[iy, ix, angle]; lo/hi = first/last scoring speed bin (255 = no shot); tfl = flight time, 10 ms units.

For every grid spot and each of the four cells (RED_AUD, RED_FAR, BLUE_AUD, BLUE_FAR) this picks one shot:
  1. Each cell is up in two HIVE states (the other HIVE either way). A speed window counts if it scores in
     BOTH states (intersection). If no launch angle scores in both, the spot falls back to angles that score in
     either state and is flagged OTHER_HIVE (scores only with the other HIVE one particular way).
  2. Launch angles from LAUNCH_MIN (43.01765, rounded to the 43 deg column) to 80 deg.
  3. Hood rule from NOTES.md: among angles whose speed window is >= 75% of the widest, the one nearest the
     preferred launch angle (72 deg, about hood 8 deg).
  4. Speed = middle of that angle's window; half window = (hi - lo)/2 + half a bin.
Spots with no shot get the values of the nearest scoring spot (so the turret and flywheel don't jump) and no
VALID flag.

Output record per spot (big-endian), zlib + base64, written as a Java class:
  int16 launch deg * 100, uint16 exit speed mm/s, uint16 flight ms, uint16 half window mm/s, uint8 flags
  flags: 1 = VALID (map says this spot scores), 2 = OTHER_HIVE (only with the other HIVE one way)
"""
import base64
import os
import struct
import sys
import zlib

import numpy as np

MAPS = sys.argv[1] if len(sys.argv) > 1 else "maps_launch43_80"
PREF_DEG = float(sys.argv[2]) if len(sys.argv) > 2 else 72.0
LAUNCH_MIN, LAUNCH_MAX = 43.0, 80.0          # 43.01765 rounded down to the map's 43 deg column
KEEP_FRAC = 0.75
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "TeamCode", "src", "main", "java", "org",
                   "firstinspires", "ftc", "teamcode", "subsystem", "control", "HiveAimTableData.java")
if len(sys.argv) > 3:
    OUT = sys.argv[3]
CELLS = ["red_aud", "red_far", "blue_aud", "blue_far"]   # order = HiveAimTable.Cell ordinal

maps = {s: np.load(os.path.join(MAPS, f"map_{s}.npz")) for s in "ABCD"}
first = maps["A"]
xs = first["xs_in"].astype(float)
elev = first["elev"].astype(float)
speeds = first["speeds"].astype(float)
dv = float(speeds[1] - speeds[0])
n = len(xs)
assert np.allclose(np.diff(xs), xs[1] - xs[0])
angle_ok = (elev >= LAUNCH_MIN - 1e-6) & (elev <= LAUNCH_MAX + 1e-6)

recs = {}
summary = []
for cell in CELLS:
    found = []
    for s, z in maps.items():
        key = f"{s}_{cell}_up"
        if f"{key}__lo" in z:
            found.append((z[f"{key}__lo"].astype(int), z[f"{key}__hi"].astype(int), z[f"{key}__tfl"].astype(int)))
    assert len(found) == 2, (cell, len(found))
    (lo1, hi1, t1), (lo2, hi2, t2) = found
    v1 = (lo1 < 255) & angle_ok[None, None, :]
    v2 = (lo2 < 255) & angle_ok[None, None, :]
    blo, bhi = np.maximum(lo1, lo2), np.minimum(hi1, hi2)
    both = v1 & v2 & (blo <= bhi)

    launch = np.zeros((n, n)); speed = np.zeros((n, n)); tof = np.zeros((n, n)); half = np.zeros((n, n))
    flags = np.zeros((n, n), int)
    for iy in range(n):
        for ix in range(n):
            if both[iy, ix].any():
                lo, hi, ok = blo[iy, ix], bhi[iy, ix], both[iy, ix]
                tf = np.maximum(t1[iy, ix], t2[iy, ix])
                fl = 1
            elif (v1[iy, ix] | v2[iy, ix]).any():
                w1 = np.where(v1[iy, ix], hi1[iy, ix] - lo1[iy, ix], -1)
                w2 = np.where(v2[iy, ix], hi2[iy, ix] - lo2[iy, ix], -1)
                use1 = w1 >= w2
                lo = np.where(use1, lo1[iy, ix], lo2[iy, ix]); hi = np.where(use1, hi1[iy, ix], hi2[iy, ix])
                tf = np.where(use1, t1[iy, ix], t2[iy, ix])
                ok = v1[iy, ix] | v2[iy, ix]
                fl = 1 | 2
            else:
                continue
            w = np.where(ok, hi - lo, -1)
            cand = np.where(w >= KEEP_FRAC * w.max())[0]
            h = cand[np.argmin(np.abs(elev[cand] - PREF_DEG))]
            launch[iy, ix] = elev[h]
            speed[iy, ix] = (speeds[lo[h]] + speeds[hi[h]]) / 2
            half[iy, ix] = (speeds[hi[h]] - speeds[lo[h]]) / 2 + dv / 2
            tof[iy, ix] = tf[h] * 0.01
            flags[iy, ix] = fl

    valid = flags & 1 > 0
    vy, vx = np.nonzero(valid)
    for iy in range(n):          # fill no-shot spots from the nearest scoring spot
        for ix in range(n):
            if valid[iy, ix]:
                continue
            k = np.argmin((vy - iy) ** 2 + (vx - ix) ** 2)
            j = (vy[k], vx[k])
            launch[iy, ix], speed[iy, ix], tof[iy, ix], half[iy, ix] = launch[j], speed[j], tof[j], half[j]
    recs[cell] = (launch, speed, tof, half, flags)
    summary.append(f"{cell}: {valid.sum()} scoring spots ({(flags == 1).sum()} whatever the other HIVE does), "
                   f"launch {launch[valid].min():.0f}-{launch[valid].max():.0f} deg, "
                   f"speed {speed[valid].min():.2f}-{speed[valid].max():.2f} m/s, "
                   f"scoring centroid ({xs[vx].mean():.0f}, {xs[vy].mean():.0f})")

buf = bytearray()
for cell in CELLS:
    launch, speed, tof, half, flags = recs[cell]
    for iy in range(n):
        for ix in range(n):
            buf += struct.pack(">hHHHB", int(round(launch[iy, ix] * 100)), int(round(speed[iy, ix] * 1000)),
                               int(round(tof[iy, ix] * 1000)), int(round(half[iy, ix] * 1000)), int(flags[iy, ix]))
packed = base64.b64encode(zlib.compress(bytes(buf), 9)).decode()
chunks = [packed[i:i + 8000] for i in range(0, len(packed), 8000)]

java = f"""package org.firstinspires.ftc.teamcode.subsystem.control;

// GENERATED by tools/export_hive_aim_table.py from {os.path.basename(os.path.normpath(MAPS))}. Do not edit by hand:
// re-run the script after re-running the shot-map simulation.
// Preferred launch angle {PREF_DEG:g} deg, launch angles {LAUNCH_MIN:g}-{LAUNCH_MAX:g} deg, keep >= {KEEP_FRAC:g} of widest window.
// {chr(10).join('// ' + s for s in summary)[3:]}
final class HiveAimTableData {{
    private HiveAimTableData() {{}}

    static final int N = {n};
    static final double X0 = {xs[0]:g};
    static final double DX = {xs[1] - xs[0]:g};
    static final int RECORD_BYTES = 9;
    static final int RAW_BYTES = {len(buf)};

    /** zlib-compressed records, base64. Cells in order RED_AUD, RED_FAR, BLUE_AUD, BLUE_FAR; rows by y, then x. */
    static String packed() {{
        StringBuilder sb = new StringBuilder({len(packed)});
{chr(10).join(f'        sb.append("{c}");' for c in chunks)}
        return sb.toString();
    }}
}}
"""
os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
with open(OUT, "w", encoding="utf-8", newline="\n") as f:
    f.write(java)
print("\n".join(summary))
print(f"wrote {OUT}: {len(buf)} raw bytes -> {len(packed)} base64 chars")
