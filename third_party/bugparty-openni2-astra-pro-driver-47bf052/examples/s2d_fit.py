#!/usr/bin/env python3
"""
Build S2D lookup table from paired frames:
  1. Official driver depth frame (uint16 depth values in mm)
  2. Our driver raw shift frame (IDENTITY mode, uint16 raw 11-bit shifts)

For each shift value s, we compute the average depth from the official driver
at pixels where our driver measured shift s. This gives us the correct
shift -> depth mapping per-pixel.

Then we fit the formula: depth = focalBL / (fittingCoeff - shift)
to get the best focalBL and fittingCoeff.

Usage: python3 s2d_fit.py <official_frame.bin> <our_shift_frame.bin>
"""
import struct
import sys
import numpy as np

def load_frame(path):
    """Load a frame dump: header [width, height, min, max, nonzero] + 640x480 uint16"""
    with open(path, 'rb') as f:
        hdr_data = f.read(20)
        width, height, min_v, max_v, nonzero = struct.unpack('<5I', hdr_data)
        pixel_data = f.read(width * height * 2)
        pixels = np.frombuffer(pixel_data, dtype=np.uint16).reshape((height, width))
        return width, height, min_v, max_v, nonzero, pixels

if len(sys.argv) < 3:
    print("Usage: python3 s2d_fit.py <official_depth.bin> <our_shift.bin>")
    sys.exit(1)

print("Loading official driver depth frame...")
ow, oh, omin, omax, onz, official_depth = load_frame(sys.argv[1])
print(f"  {ow}x{oh}, min={omin} max={omax} nonzero={onz} ({100*onz/(ow*oh):.1f}%)")

print("Loading our driver raw shift frame...")
sw, sh, smin, smax, snz, our_shifts = load_frame(sys.argv[2])
print(f"  {sw}x{sh}, min={smin} max={smax} nonzero={snz} ({100*snz/(sw*sh):.1f}%)")

# For each shift value, compute average official depth
print("\nBuilding shift->depth histogram...")
shift_depth_map = {}  # shift -> list of official depths
for y in range(sh):
    for x in range(sw):
        s = our_shifts[y, x]
        d = official_depth[y, x]
        if s != 0 and d != 0:
            if s not in shift_depth_map:
                shift_depth_map[s] = []
            shift_depth_map[s].append(d)

print(f"  Valid pixel pairs: {sum(len(v) for v in shift_depth_map.values())}")

# Compute per-shift average depth
shift_avg = {}
shift_std = {}
shift_count = {}
for s in sorted(shift_depth_map.keys()):
    depths = shift_depth_map[s]
    shift_avg[s] = np.mean(depths)
    shift_std[s] = np.std(depths) if len(depths) > 1 else 0
    shift_count[s] = len(depths)

print(f"  Unique shift values: {len(shift_avg)}")

# Show some sample mappings
print("\n  Shift -> Avg Official Depth (stddev) [count]")
for s in sorted(shift_avg.keys())[:20]:
    print(f"    {s:5d} -> {shift_avg[s]:7.1f}mm (±{shift_std[s]:5.1f}) [{shift_count[s]:5d}]")
print("    ...")
for s in sorted(shift_avg.keys())[-5:]:
    print(f"    {s:5d} -> {shift_avg[s]:7.1f}mm (±{shift_std[s]:5.1f}) [{shift_count[s]:5d}]")

# Fit formula: depth = focalBL / (C - shift)
# Using shifts with sufficient sample counts
print("\nFitting formula: depth = focalBL / (fittingCoeff - shift)")
min_samples = 50
fit_shifts = []
fit_depths = []
for s in sorted(shift_avg.keys()):
    if shift_count[s] >= min_samples:
        fit_shifts.append(s)
        fit_depths.append(shift_avg[s])

fit_shifts = np.array(fit_shifts)
fit_depths = np.array(fit_depths)
print(f"  Using {len(fit_shifts)} shift values with >= {min_samples} samples each")

# Fit: 1/depth = (C - shift) / focalBL = C/focalBL - shift/focalBL
# Let y = 1/depth, x = shift
# y = a + b*x where a = C/focalBL, b = -1/focalBL
# So focalBL = -1/b, C = a * focalBL = -a/b

x = fit_shifts.astype(np.float64)
y = 1.0 / fit_depths.astype(np.float64)

# Linear regression
A = np.vstack([np.ones_like(x), x]).T
params, residuals, _, _ = np.linalg.lstsq(A, y, rcond=None)
a, b = params

focalBL = -1.0 / b
fittingCoeff = -a / b

print(f"  focalBL = {focalBL:.2f}")
print(f"  fittingCoeff = {fittingCoeff:.4f}")
print(f"  Residual sum: {residuals.sum() if len(residuals) > 0 else 0:.2e}")

# Verify fit quality
print("\n  Verification (shift -> official depth, fitted depth, error):")
test_shifts = [s for s in sorted(shift_avg.keys()) if shift_count[s] >= min_samples]
for s in test_shifts[::max(1, len(test_shifts)//15)]:
    official_d = shift_avg[s]
    fitted_d = focalBL / (fittingCoeff - s) if fittingCoeff != s else float('inf')
    err = fitted_d - official_d
    pct = 100.0 * err / official_d if official_d != 0 else 0
    print(f"    {s:5d} -> {official_d:7.1f}mm, fitted={fitted_d:7.1f}mm, err={err:+7.1f}mm ({pct:+.2f}%)")

# Save lookup table
outpath = "/home/bowmanhan/Code/OrbbecSDK/openni2-astra-driver/s2d_table_calibrated.txt"
with open(outpath, 'w') as f:
    f.write(f"# Calibrated S2D table from paired frames\n")
    f.write(f"# focalBL = {focalBL:.2f}\n")
    f.write(f"# fittingCoeff = {fittingCoeff:.4f}\n")
    f.write(f"# nShiftScale = 1\n")
    f.write(f"# shift\tdepth\tcount\tstddev\n")
    for s in sorted(shift_avg.keys()):
        fitted_d = focalBL / (fittingCoeff - s) if fittingCoeff != s else 0
        f.write(f"{s}\t{shift_avg[s]:.1f}\t{shift_count[s]}\t{shift_std[s]:.1f}\t{fitted_d:.1f}\n")
print(f"\nSaved S2D table to {outpath}")
print(f"\nUse these parameters in AstraDevice.cpp:")
print(f"  const double focalBL = {focalBL:.2f};")
print(f"  const double fittingCoeff = {fittingCoeff:.4f};")
print(f"  const int nShiftScale = 1;")
