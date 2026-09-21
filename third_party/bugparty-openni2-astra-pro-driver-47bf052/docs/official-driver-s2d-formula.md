# Official Orbbec driver: shift-to-depth formula

Reverse-engineered from `liborbbec.so` (build `ed619a06…`, AstraSDK
v2.1.3-94bca0f52e Ubuntu18.04 x86_64) using rizin.

## Formula

```
depth_mm = (forcalllength × baseline × nShiftScale) / (fittingCoeff − shift)
```

This is the **standard rectified-stereo / structured-light triangulation
formula** (`Z = f·B / d`), where `disparity_pixels = (fittingCoeff − shift)
/ nShiftScale`. *Not* the rational PrimeSense formula (`Z = ZPD ·
EmitterD/(EmitterD − metric)`) we use in `AstraDevice::computeShiftToDepthTable`.

## Pseudocode (from `XnDepthProcessor::Init` @ 0x60570)

```cpp
double focalBL  = forcalllength * baseline;          // float × float (from flash)
int    nShift   = nShiftScale;                       // int (from flash)
double fitC     = fittingCoeff;                      // double (from flash)
int    unitScale = (depth_mode == DEPTH_100_UM) ? 10 : 1;
uint16_t* table = malloc(0x2000);                    // 8 KB = 4096 × uint16
uint16_t  minDepth = sensor.minDepth_mm;             // from flash
uint16_t  maxDepth = sensor.maxDepth_mm * unitScale; // from flash, * scale

for (uint16_t s = 0; s < 0x1000; ++s) {
    double disparity = (fitC - s) / nShift;
    if (disparity <= 0)            { table[s] = 0; continue; }
    if (disparity >= focalBL)      { table[s] = 0; continue; }
    double depth = unitScale * focalBL / disparity;
    if (depth <= minDepth)         { table[s] = 0; continue; }
    if (depth >= maxDepth)         { table[s] = 0; continue; }
    table[s] = (uint16_t)depth;
}
```

The runtime function `depthOptimization` (sym at 0x51350) is a *separate*
LUT — it's a pre-computed quadratic table used by an "optimisation" feature
(post-processing toggle), not the live S2D path.

## Comparison to our driver

| concern               | official driver                      | our driver                          |
| --------------------- | ------------------------------------ | ----------------------------------- |
| formula               | `f·B·N/(C−s)` (rectified stereo)     | `ZPD · m/(D−m)` (PrimeSense rational) |
| constants source      | flash: forcalllength, baseline, fittingCoeff, nShiftScale | hardcoded fits      |
| min depth cutoff      | `sensor.minDepth_mm` (flash)         | hardcoded 300 mm                    |
| max depth cutoff      | `sensor.maxDepth_mm × unitScale` (flash) | hardcoded 8000 mm + quantization filter |
| disparity guard       | `disparity > 0` and `disparity < f·B` | `metric ∈ [0, EmitterD)` |

The two formulas are mathematically equivalent up to a change of variables —
both have the same `1/x` blow-up behaviour as disparity → 0. The reason the
official driver's max depth in any given scene matches "physical reality"
isn't the formula choice; it's that **the per-device max-depth cutoff comes
from flash**.

## What we need to read from flash

To match exactly we should read the calibration block during init and pass:

- `forcalllength` (float) — focal length in pixels (or sensor units)
- `baseline` (float) — emitter-to-CMOS distance (mm)
- `fittingCoeff` (double) — shift offset
- `nShiftScale` (int) — disparity scale factor
- `minDepth_mm` (uint16) — per-device lower cutoff
- `maxDepth_mm` (uint16) — per-device upper cutoff

The official driver reads these via `XnSensor::GetCameraParam` (function at
0x457e0). That function takes a **chip-ID-dependent branch**:

```cpp
if (privateData->chip_id == 0x06) {
    XnHostProtocolI2CReadFlash(addr=0x70000, size=0x78, dst);
} else {
    ReadFlash(addr=0x70000, size=0x3c, dst);   // direct firmware-flash read
}
```

### Probed paths on our Astra Pro (fw 0xe752, SN 17103010495)

| source                              | result                                          |
| ----------------------------------- | ----------------------------------------------- |
| direct `ReadFlash(0x70000, 240)`    | all `0xFF` (erased / not the right region)      |
| direct `ReadFlash(0x10000, 240)`    | ARM/Thumb opcodes — this is firmware code       |
| `algorithmParams(ALG_DEPTH_INFO)`   | 2-byte response `c8 00` (just a count/flag)     |

So the calibration on this device lives behind the **I2C-flash protocol path**
(chip_id == 0x06 branch). To finish the work:

1. Read the chip-ID. The official driver stores it at `[XnSensor + 0x334d]`,
   populated during `XnHostProtocolInitFWParams` from `GetUsbCoreType` or a
   similar identification call.
2. Implement `XnHostProtocolI2CReadFlash` — see `sym.XnHostProtocolI2CReadFlash`
   at 0x5de00 and `sym.XnHostProtocolI2CReadFlashOnce` at 0x5dce0.
3. Wire the resulting 120-byte struct into a new path in
   `AstraDevice::computeShiftToDepthTable` that uses the rectified-stereo
   formula at the top of this doc.

Until then we use hardcoded PrimeSense-fit constants in
`computeShiftToDepthTable` and accept a ~1.5 % bias vs. the official driver.

## Other things noticed

* `min/max cutoff` of the inner loop is `0x64` (= 100, depth_min) to `0xfa1`
  (= 4001, max shift). Independent of the depth bounds.
* The "depth optimization" path (toggled via
  `XnSensor::SetDepthOptimizationState`) uses the depthOptTable quadratic LUT
  built in `depthOptTableInit` at 0x511f0. That formula is:

  ```
  a · depth² + (b + 1.0) · depth + (c − shift) = 0
  depth = (sqrt((b + 1.0)² − 4a(c − shift)) − (b + 1.0)) / (2a)
  ```

  The 1.0 constant is at `data.000a9de0`. Args come from the optimization
  param block via `setObDepthOptimizationParam`. We do not need this for the
  basic depth path — only if a user enables depth optimization.

## References

- `liborbbec.so.official` @ `AstraSDK-v2.1.3/lib/Plugins/openni2/liborbbec.so.official`
- Function: `method.XnDepthProcessor.Init` (0x60570)
- LUT inner loop: 0x60708 – 0x6072f
- Quadratic optim path: `sym.depthOptTableInit_double__double__double` (0x511f0)
- Log format string proving the 6 flash fields: `data.000ac9b8`
  (`Read forcalllength: %f, baseline: %f, fbcoeff: %f, fittingCoeff: %f,
  fCoefficient: %f, nShiftScale: %d`)
