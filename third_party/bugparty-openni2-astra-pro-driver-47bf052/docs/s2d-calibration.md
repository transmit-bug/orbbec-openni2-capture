# S2D (Shift-to-Depth) Calibration Guide

## Overview

The Astra Pro depth camera outputs raw **shift values** (11-bit, 0-2047) from its stereo
correlation hardware. These must be converted to metric depth (mm) using the formula:

```
depth = focalBL / (fittingCoeff - shift)
```

Where:
- `focalBL` = focal_length * baseline (in mm*px units)
- `fittingCoeff` = the shift value at zero disparity (infinite depth)
- `shift` = raw 11-bit value from firmware (Packed11 decoded)

This document describes the **paired-frame calibration method** used to determine these
parameters for a specific device.

## Why Calibration Is Needed

The original PrimeSense S2D formula (used in the openni2-astra-driver upstream) assumes
fixed parameters that are **wrong for the Astra Pro**:

| Parameter | PrimeSense default | Astra Pro (calibrated) |
|-----------|-------------------|----------------------|
| ConstShift | 800 | N/A (different formula) |
| focalBL | ~953452 | **342857** |
| fittingCoeff | ~2445 | **1066** |

Using the wrong parameters produces depth values that are off by hundreds of mm.
Some devices also have empty flash calibration data, making device-specific calibration
the only viable approach.

## Paired-Frame Calibration Method

### Principle

Capture two frames of the **same scene** simultaneously:
1. **Official driver** — outputs depth in mm (ground truth)
2. **Our driver** (IDENTITY mode) — outputs raw shift values

For each shift value `s`, compute the average official depth across all pixels where
our driver measured shift `s`. Then fit the formula via linear regression.

### Step 1: Build Tools

```bash
cd openni2-astra-driver/build
cmake .. && make -j4
```

This builds `dump_frame` (frame capture tool) and `oni_driver_astra.so` (our driver).

### Step 2: Deploy Our Driver

```bash
cp build/oni_driver_astra.so \
   /path/to/AstraSDK-v2.1.3/lib/Plugins/openni2/OpenNI2/Drivers/libAstraDriver.so
```

> **Important**: OpenNI2 loads drivers from `OpenNI2/Drivers/` relative to `libOpenNI2.so`,
> NOT from the `OPENNI2_REPO` env var.

### Step 3: Capture Official Driver Frame

Use the official Orbbec driver to capture a depth frame:

```bash
# Switch to official driver
cp /path/to/liborbbec.so.official \
   /path/to/OpenNI2/Drivers/libAstraDriver.so

# Compile and run dump_frame
cd openni2-astra-driver/examples
gcc -o dump_frame dump_frame.c \
    -I../include -L/path/to/AstraSDK-v2.1.3/lib/Plugins/openni2 \
    -lOpenNI2 -Wl,-rpath,/path/to/AstraSDK-v2.1.3/lib/Plugins/openni2

LD_LIBRARY_PATH=/path/to/AstraSDK-v2.1.3/lib/Plugins/openni2 \
    ./dump_frame /tmp/official_depth.bin
```

### Step 4: Capture Our Driver Raw Shift Frame

Switch to our driver in IDENTITY mode:

```bash
# Deploy our driver
cp build/oni_driver_astra.so \
   /path/to/OpenNI2/Drivers/libAstraDriver.so

# Run with ASTRA_S2D_IDENTITY to get raw shift values
ASTRA_S2D_IDENTITY=1 \
LD_LIBRARY_PATH=/path/to/AstraSDK-v2.1.3/lib/Plugins/openni2 \
    ./dump_frame /tmp/our_shifts.bin
```

### Step 5: Fit S2D Parameters

```bash
cd openni2-astra-driver/examples
python3 s2d_fit.py /tmp/official_depth.bin /tmp/our_shifts.bin
```

Output example:
```
focalBL = 342857.28
fittingCoeff = 1066.2583
nShiftScale = 1

Use these parameters in AstraDevice.cpp:
  const double focalBL = 342857.28;
  const double fittingCoeff = 1066.2583;
  const int nShiftScale = 1;
```

### Step 6: Update and Rebuild Driver

Edit `src/AstraDevice.cpp` in `computeShiftToDepthTable()`:

```cpp
const double focalBL = 342857.28;      // focal_length * baseline (calibrated)
const double fittingCoeff = 1066.2583;  // zero-disparity shift (calibrated)
const int    nShiftScale = 1;
```

Rebuild and redeploy:
```bash
cd build && make -j4
cp oni_driver_astra.so /path/to/OpenNI2/Drivers/libAstraDriver.so
```

### Step 7: Verify

Run `pixel_dump` (or any OpenNI2 depth viewer) and compare against the official driver:

```bash
LD_LIBRARY_PATH=/path/to/AstraSDK-v2.1.3/lib/Plugins/openni2 \
    ./pixel_dump <x> <y>
```

Expected: our depth values within 1-3mm of official driver across the full range.

## Binary Frame Format

The `dump_frame` tool writes:
- **Header**: 20 bytes — 5x uint32 LE: `[width, height, minVal, maxVal, nonzeroCount]`
- **Pixels**: width * height * 2 bytes — uint16 LE, row-major

The `s2d_fit.py` script reads this format.

## Calibration Quality Checklist

- [ ] Both frames captured in the **same scene** (don't move camera between captures)
- [ ] Official frame has >100K non-zero pixels (good coverage)
- [ ] Raw shift frame has similar pixel coverage
- [ ] Fit uses shift values with >=50 pixel samples (set `min_samples` in script)
- [ ] Max fit error < 1% across the depth range
- [ ] Live verification matches official driver within ~3mm

## Formula Behavior

| Shift range | Depth | Notes |
|-------------|-------|-------|
| 1 - fittingCoeff | depth = focalBL/(C-shift) | Valid range |
| shift >= fittingCoeff | 0 | Negative disparity, no depth |
| shift = 0 | 0 | No signal |

With fittingCoeff=1066.26:
- Near depth limit: shift=1 → depth=322mm
- Far depth limit: shift=1023 → depth=3940mm
- Practical range: ~400mm to ~4000mm (shift 87 to 1023)

## Troubleshooting

### "All zero depth frames"
- Verify driver is loaded: check stderr for `AstraDevice: S2D table` message
- Check `libAstraDriver.so` is in the correct `OpenNI2/Drivers/` path

### "Depth values too high/low compared to reality"
- S2D parameters are wrong — redo calibration
- Make sure scene hasn't changed between official and our captures

### "Only 3% non-zero pixels"
- Frame corruption (partial first frame) — check bytesWritten vs expected
- Packet parsing issue — check DIAG output for packet errors
- Try re-plugging USB and restarting

### "High shift values (>1023) map to 0"
- This is expected: shifts >= fittingCoeff produce negative disparity
- Very few real pixels have shifts near the fittingCoeff limit
- If too many pixels are zero, the scene may be beyond the depth range

## File Reference

| File | Purpose |
|------|---------|
| `examples/dump_frame.c` | Capture depth/shift frames to binary file |
| `examples/s2d_fit.py` | Fit S2D parameters from paired frames |
| `src/AstraDevice.cpp` | S2D table computation (`computeShiftToDepthTable`) |
| `src/DepthProcessor.cpp` | Packed11 decode + S2D conversion pipeline |
| `s2d_table_calibrated.txt` | Per-shift mapping table with stats |
