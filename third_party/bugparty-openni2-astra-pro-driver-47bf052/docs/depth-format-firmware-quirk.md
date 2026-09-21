# Astra Pro firmware 0xe752 ignores depth-format requests

## Summary

On Astra Pro fw `0xe752` (SN `17103010495`), the firmware **silently ignores
the XnHostProtocol `PARAM_DEPTH_FORMAT` (paramId 18)** command. No matter
which value we set (0=Uncompressed16, 1=PSCompressed, 3=Packed11), the depth
endpoint always emits the **Packed11** byte stream. The driver must therefore
configure its depth decoder for Packed11 and ignore the format negotiation.

Code:
```cpp
// AstraDevice::configureDepthStream
m_fwCmd->setDepthFormat(3);     // 3 = XN_IO_DEPTH_FORMAT_PACKED_11_BIT
m_depthProc->setDepthFormat(1); // 1 = Packed11 (internal enum)
```

Setting any other format produces visibly garbage depth (~3 % valid pixels,
median pinned to ~322 mm = `s2dTable[1]`, the smallest non-zero LUT entry).

## How the symptom manifests

Side-by-side capture vs. the pristine official `liborbbec.so` (same scene,
640×480 @ 30 fps):

| metric                | PSCompressed (fw fmt 1) | Packed11 (fw fmt 3) | official |
| --------------------- | ----------------------- | ------------------- | -------- |
| nonzero pixels        | 3.0 %                   | 11.3 %              | 11.2 %   |
| median depth          | 322 mm                  | 1094 mm             | 1073 mm  |
| ratio vs official     | 0.30                    | **1.017**           | 1.000    |
| only-our-side pixels  | ~8 000                  | ~2 000              | —        |

When asked for PSCompressed, the PrimeSense nibble RLE/diff decoder reads
mostly small (`0x0`) nibbles from the (actually-Packed11) byte stream,
interprets them as `Δ = -6`, drifts `nLastValue` down through unsigned
underflow, and produces either `NO_DEPTH` (clamped) or `s2dTable[1]` (when it
happens to land on shift = 1). The decoder is correct for valid PSCompressed
input — it is simply being fed the wrong bytes.

## How we proved it

Two diagnostic env-var hooks were added to the driver (still in tree):

1. `ASTRA_DEPTH_FORMAT={ps,p11,u16}` — overrides the format request in
   `AstraDevice::configureDepthStream`.
2. `ASTRA_DUMP_BULK_<HEX>=<path>` — `UsbDevice::bulkReadLoop` writes the
   first ~1 MB of raw USB bulk data from the chosen endpoint to a file.

Capture procedure (with the camera pointed at a static scene):

```bash
for FMT in p11 ps u16; do
  # USB-reset the depth interface (camera gets stuck after a failed start)
  python3 -c "import fcntl,os; fd=os.open('/dev/bus/usb/003/<dev>',os.O_WRONLY); fcntl.ioctl(fd,21780,0); os.close(fd)"
  ASTRA_DEPTH_FORMAT=$FMT \
  ASTRA_DUMP_BULK_81=/tmp/depth_${FMT}.bin \
  LD_LIBRARY_PATH=AstraSDK-v2.1.3/lib/Plugins/openni2 \
    ./test_gtk_viewer_v2 --capture out_${FMT} --warmup 30 --count 1
done
```

The dumps were parsed (`analyze_dump.py`) into Orbbec packet streams
(magic `0x4252`, big-endian `bufSize`, header types `0x7100`/`0x7200`/
`0x7500`):

| metric (per ~1 MB dump) | p11    | ps     | u16    |
| ----------------------- | ------ | ------ | ------ |
| 0x7200 buffer packets   | 335    | 336    | 336    |
| payload bytes/buffer    | 3060   | 3060   | 3060   |
| zero-byte share         | 20.0 % | 20.4 % | 19.9 % |
| top non-zero bytes      | dd, 77, ee, 79, 1d, 3b, bb | (same set) | (same set) |
| **+11 byte equality**   | **37 %** | **40 %** | **56 %** |
| **+2  byte equality**   | 19 %   | 21 %   | **0 %** |

Packet counts, sizes and byte distributions are statistically identical across
all three configurations. The 11-byte periodicity (one Packed11 element =
11 bytes → 8 pixels) is dominant in every dump; the 2-byte periodicity that
Uncompressed16 would produce is **completely absent even when we explicitly
request u16**. That rules out every alternative: firmware is locked to
Packed11.

## Implications

* Do **not** depend on `setDepthFormat()` to switch formats on this firmware.
  Treat the call as a hint; verify with a USB capture before trusting it.
* The `PSCompressed` and `Uncompressed16` paths in
  `DepthProcessor::processFramePacketChunk` cannot be exercised on this device.
  Don't delete them — other Astra Pro firmware revisions may behave
  differently — but mark them clearly as untested-on-this-fw.
* The S2D LUT remains the only remaining accuracy gap (~1.7 % vs. official).
  That's a flash-calibration-read problem, not a format problem.

## References

* Comparison harness: `../switch_driver.sh`,
  `../test_gtk_viewer_v2.c --capture …`, `../compare_depth.py`,
  `../analyze_dump.py` (in the parent OrbbecSDK directory)
* Decoder: `src/DepthProcessor.cpp`
* Format command: `src/FirmwareCmd.cpp::setDepthFormat` →
  `setParam(SP_DEPTH_FORMAT=18, value)`
* Packet structure: `src/PacketParser.h` (note `bufSize` is **big-endian**
  on the wire)
