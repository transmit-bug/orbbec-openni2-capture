# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Index

- [Repository Overview](#repository-overview)
- [Build Commands](#build-commands)
- [Architecture](#architecture)
- [Key Patterns](#key-patterns)
- [Comparison Harness](#comparison-harness)

## Repository Overview

This is the **openni2-astra-driver** — a source-built OpenNI2 driver for the Orbbec Astra Pro depth camera (VID=0x2bc5, PID=0x0403). It replaces Orbbec's proprietary `liborbbec.so` with an open-source implementation that can be modified and rebuilt.

Build output: `build/oni_driver_astra.so`

## Build Commands

```bash
mkdir -p build && cd build && cmake .. && cmake --build .
```

## Architecture

### Source Layout
- `src/AstraDriver.cpp` — OpenNI2 DriverBase subclass; device enumeration via libusb
- `src/AstraDevice.cpp` — OpenNI2 DeviceBase subclass; stream lifecycle, firmware init, S2D LUT
- `src/AstraDepthStream.cpp` — Depth stream (DStreamBase); Packed11 → S2D conversion
- `src/AstraIRStream.cpp` — IR stream (DStreamBase); raw 16-bit passthrough
- `src/FirmwareCmd.cpp` — XnHostProtocol + SendCmd dual-stack firmware command interface
- `src/UsbDevice.cpp` — libusb bulk transfer I/O for depth/IR endpoints
- `src/PacketParser.cpp` — USB bulk data state machine; reassembly into frames
- `src/FrameProcessor.cpp` — Frame assembly base class
- `src/IrProcessor.cpp` — IR frame assembly
- `src/DepthProcessor.cpp` — 3 depth format decoders (Packed11/PSCompressed/Uncompressed16) + ShiftToDepth LUT
- `include/openni2/` — OpenNI2 C headers + driver API (OniDriverAPI.h)
- `examples/` — Test programs (hello_astra, ir_control, depth_viewer, ir_viewer, s2d_table, etc.)

### Astra Pro Firmware Protocol
For direct USB control, see `../ASTRA_PRO_PROTOCOL.md` — a reverse-engineered reference of the dual-protocol-stack (XnHostProtocol + SendCmd) firmware commands, cmdId lookup tables, and flash memory layout.

## Key Patterns

### Depth Format Must Be Packed11
On Astra Pro fw 0xe752, **depth format must be Packed11**: in `AstraDevice::configureDepthStream` set `m_fwCmd->setDepthFormat(3)` and `m_depthProc->setDepthFormat(1)`. PSCompressed (firmware fmt 1) yields garbage — nibble stream collapses to ~LUT[1] (~322mm) for ~3% of pixels.

### S2D LUT
Uses PrimeSense triangulation formula with gdb-captured constants (nParamCoeff=4, nConstShift=200, ShiftScale=10, ZPD=130.0, ZPPS=0.113967, DCL=7.5, pixelSizeFactor=1). ShiftScale=10 produces mm directly — do NOT divide by 10. Mean depth within 0.7% of official driver. 0.4% edge-noise pixels produce depths > 1m where official outputs 0 (edge filtering not yet implemented).

### Driver Loading
OpenNI2 scans `<libOpenNI2.so dir>/OpenNI2/Drivers/*.so` and loads ALL files exporting `oniDriverCreate`. Keep exactly one driver `.so` in that dir or you get duplicate device entries.

### Official Driver Reference
Pristine official driver: `../AstraSDK-v2.1.3/lib/Plugins/openni2/liborbbec.so.official` (copied from the unmodified AstraSDK download). The `.disabled` and `.orig` files are NOT the original — they are earlier copies of our driver.

### OpenNI2 C API
- Must include `OniCProperties.h` for `ONI_STREAM_PROPERTY_VIDEO_MODE`, `OBEXTENSION_ID_LASER_EN`, etc.
- `oniDeviceGetSensorInfo()` returns `const OniSensorInfo*` (not mutable)
- No `oniStreamSetVideoMode`/`oniStreamGetVideoMode` — use `oniStreamSetProperty`/`oniStreamGetProperty` with `ONI_STREAM_PROPERTY_VIDEO_MODE` and `OniVideoMode` struct
- `oniStreamGetProperty` takes `int* pDataSize` (not `int`)
- Laser control: `oniDeviceSetProperty(dev, OBEXTENSION_ID_LASER_EN, &uint8_val, 1)` — setProperty works, getProperty does NOT
- Compile: `g++ -std=c++11` (not gcc) because OpenNI2 C headers have C++ only types
- Depth+IR streams cannot run on separate `oniDeviceOpen` calls — must share one device handle for concurrent streams

### Astra Pro Hardware Details
- Two USB devices: depth sensor (PID 0x0403, vendor class) and RGB camera (PID 0x0501, UVC+Audio composite)
- Microphone is USB Audio Class 1.0 on PID 0x0403 (ALSA card "Pro", 16kHz 16bit stereo PCM)
- IR sensor has no independent illumination — laser MUST be on to see IR image
- IR video modes: 14 modes across QVGA/VGA/640x400/SXGA variants, GRAY16(203) and GRAY8(200)
- Depth video modes: 8 modes, DEPTH_1_MM(100) and DEPTH_100_UM(101), including 1280x1024@7fps

## Comparison Harness

These tools live in the parent OrbbecSDK directory:
- `switch_driver.sh official|ours|status` — swap which driver is active in `OpenNI2/Drivers/`
- `test_gtk_viewer_v2.c` — GTK viewer; `--capture <prefix> [--warmup N] [--count N]` runs headless and saves raw uint16 frames + 16-bit PGMs
- `compare_depth.py` — compares `official_*.raw` vs `ours_*.raw`: histograms, per-pixel diff, side-by-side `compare_*.png`, signed diff `compare_diff_*.png`
- USB-reset depth interface between runs: `python3 -c "import fcntl,os; fd=os.open('/dev/bus/usb/003/<dev>',os.O_WRONLY); fcntl.ioctl(fd,21780,0); os.close(fd)"`
