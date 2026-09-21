#include "AstraDevice.h"
#include "AstraDepthStream.h"
#include "AstraIRStream.h"
#include "UsbDevice.h"
#include "FirmwareCmd.h"
#include "PacketParser.h"
#include "DepthProcessor.h"
#include "IrProcessor.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

AstraDevice::AstraDevice(const char* uri, oni::driver::DriverServices& driverServices)
    : m_driverServices(driverServices)
    , m_depthModes{}
    , m_irModes{}
    , m_sensorInfos{}
{
    strncpy(m_uri, uri, ONI_MAX_STR - 1);
    m_uri[ONI_MAX_STR - 1] = '\0';

    // Create subsystems
    m_usbDev = new UsbDevice();
    m_fwCmd = new FirmwareCmd(m_usbDev);

    // Create frame processors
    m_depthProc = new DepthProcessor();
    m_irProc = new IrProcessor();

    // Create packet parsers (wire up to processors later during stream start)
    m_depthParser = new PacketParser();
    m_irParser = new PacketParser();

    // Build sensor info arrays
    buildSensorInfos();
}

AstraDevice::~AstraDevice()
{
    stopAllStreams();

    delete m_irParser;
    m_irParser = nullptr;

    delete m_depthParser;
    m_depthParser = nullptr;

    delete m_irProc;
    m_irProc = nullptr;

    delete m_depthProc;
    m_depthProc = nullptr;

    delete m_fwCmd;
    m_fwCmd = nullptr;

    delete m_usbDev;
    m_usbDev = nullptr;
}

// ---------------------------------------------------------------------------
// DeviceBase: sensor info
// ---------------------------------------------------------------------------

OniStatus AstraDevice::getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors)
{
    if (!m_initialized && !initUsb()) {
        *numSensors = 0;
        return ONI_STATUS_ERROR;
    }

    *pSensorInfos = m_sensorInfos;
    *numSensors = 2;
    return ONI_STATUS_OK;
}

// ---------------------------------------------------------------------------
// DeviceBase: stream creation / destruction
// ---------------------------------------------------------------------------

oni::driver::StreamBase* AstraDevice::createStream(OniSensorType sensorType)
{
    if (!m_initialized && !initUsb()) {
        return nullptr;
    }

    if (sensorType == ONI_SENSOR_DEPTH) {
        auto* stream = new AstraDepthStream(this);
        m_depthStream = stream;
        return stream;
    } else if (sensorType == ONI_SENSOR_IR) {
        auto* stream = new AstraIRStream(this);
        m_irStream = stream;
        return stream;
    }

    return nullptr;
}

void AstraDevice::destroyStream(oni::driver::StreamBase* pStream)
{
    if (pStream == m_depthStream) {
        m_depthStream = nullptr;
    } else if (pStream == m_irStream) {
        m_irStream = nullptr;
    }
    delete pStream;
}

// ---------------------------------------------------------------------------
// DeviceBase: properties
// ---------------------------------------------------------------------------

OniStatus AstraDevice::setProperty(int propertyId, const void* data, int dataSize)
{
    if (!m_fwCmd) {
        return ONI_STATUS_ERROR;
    }

    switch (propertyId) {
    case OBEXTENSION_ID_LASER_EN:
        if (dataSize < 1) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setLaser(*static_cast<const uint8_t*>(data) != 0)) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    case OBEXTENSION_ID_LDP_EN:
        if (dataSize < 1) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setLdp(*static_cast<const uint8_t*>(data) != 0)) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    case OBEXTENSION_ID_IR_GAIN:
        if (dataSize < static_cast<int>(sizeof(int))) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setIRGain(*static_cast<const int*>(data))) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    case OBEXTENSION_ID_IR_EXP:
        if (dataSize < static_cast<int>(sizeof(int))) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_fwCmd->setIRExposure(*static_cast<const int*>(data))) {
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniStatus AstraDevice::getProperty(int propertyId, void* data, int* pDataSize)
{
    if (!m_fwCmd) {
        return ONI_STATUS_ERROR;
    }

    switch (propertyId) {
    case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:
    case OBEXTENSION_ID_SERIALNUMBER: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        int serialLen = static_cast<int>(m_serialNumber.size()) + 1;
        if (*pDataSize < serialLen) {
            *pDataSize = serialLen;
            return ONI_STATUS_BAD_PARAMETER;
        }
        strncpy(static_cast<char*>(data), m_serialNumber.c_str(), serialLen);
        *pDataSize = serialLen;
        return ONI_STATUS_OK;
    }

    case ONI_DEVICE_PROPERTY_FIRMWARE_VERSION: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        *static_cast<int*>(data) = static_cast<int>(m_fwVersion);
        *pDataSize = sizeof(int);
        return ONI_STATUS_OK;
    }

    case OBEXTENSION_ID_LASER_EN: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < 1) {
            *pDataSize = 1;
            return ONI_STATUS_BAD_PARAMETER;
        }
        bool on = false;
        if (m_fwCmd->getLaserState(on)) {
            *static_cast<uint8_t*>(data) = on ? 1 : 0;
            *pDataSize = 1;
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    case OBEXTENSION_ID_IR_GAIN: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        int gain = 0;
        if (m_fwCmd->getIRGain(gain)) {
            *static_cast<int*>(data) = gain;
            *pDataSize = sizeof(int);
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    case OBEXTENSION_ID_IR_EXP: {
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        int exposure = 0;
        if (m_fwCmd->getIRExposure(exposure)) {
            *static_cast<int*>(data) = exposure;
            *pDataSize = sizeof(int);
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    case OBEXTENSION_ID_CAM_PARAMS: {
        // Read camera intrinsics from flash address 0x10000
        // The OBCameraParams struct is 92 bytes
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        const int CAM_PARAMS_SIZE = 92;
        if (*pDataSize < CAM_PARAMS_SIZE) {
            *pDataSize = CAM_PARAMS_SIZE;
            return ONI_STATUS_BAD_PARAMETER;
        }
        // Read from flash 0x10000 in chunks (max 256 per read)
        uint8_t params[CAM_PARAMS_SIZE];
        int totalRead = 0;
        bool ok = true;
        while (totalRead < CAM_PARAMS_SIZE && ok) {
            int chunkSize = std::min(256, CAM_PARAMS_SIZE - totalRead);
            int bytesRead = 0;
            if (!m_fwCmd->readFlash(0x10000 + totalRead,
                                     static_cast<uint16_t>(chunkSize),
                                     params + totalRead, &bytesRead)) {
                ok = false;
            } else {
                totalRead += bytesRead;
            }
        }
        if (ok && totalRead >= CAM_PARAMS_SIZE) {
            memcpy(data, params, CAM_PARAMS_SIZE);
            *pDataSize = CAM_PARAMS_SIZE;
            return ONI_STATUS_OK;
        }
        return ONI_STATUS_ERROR;
    }

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniBool AstraDevice::isPropertySupported(int propertyId)
{
    switch (propertyId) {
    case ONI_DEVICE_PROPERTY_SERIAL_NUMBER:
    case ONI_DEVICE_PROPERTY_FIRMWARE_VERSION:
    case OBEXTENSION_ID_LASER_EN:
    case OBEXTENSION_ID_LDP_EN:
    case OBEXTENSION_ID_IR_GAIN:
    case OBEXTENSION_ID_IR_EXP:
    case OBEXTENSION_ID_CAM_PARAMS:
    case OBEXTENSION_ID_SERIALNUMBER:
        return TRUE;
    default:
        return FALSE;
    }
}

// ---------------------------------------------------------------------------
// USB initialization
// ---------------------------------------------------------------------------

bool AstraDevice::initUsb()
{
    if (m_initialized) {
        return true;
    }

    // 1. Open USB device
    if (!m_usbDev->open(m_uri)) {
        fprintf(stderr, "AstraDevice: failed to open USB device '%s'\n", m_uri);
        return false;
    }
    m_usbOpen = true;

    // 2. Initialize firmware (read version, serial)
    if (!initFirmware()) {
        fprintf(stderr, "AstraDevice: firmware init failed\n");
    }

    // 3. PrimeSense init sequence: GetMode -> KeepAlive -> SetMode(PS)
    uint16_t fwMode = 0;
    if (m_fwCmd->getMode(fwMode)) {
        // mode read OK
    }

    // Send keep-alive to verify device is responsive
    for (int i = 0; i < 3; i++) {
        if (m_fwCmd->keepAlive()) {
            break;
        }
    }

    // Switch firmware to PS streaming mode (mode 1)
    if (!m_fwCmd->setMode(1)) {
        fprintf(stderr, "AstraDevice: setMode(PS) failed\n");
    }

    // 4. Read camera parameters and compute S2D table
    readCameraParams();
    computeShiftToDepthTable();

    // 5. Pass S2D table and firmware version to depth processor
    m_depthProc->setShiftToDepthTable(m_s2dTable.data(),
                                       static_cast<int>(m_s2dTable.size()));
    // fwVersion is encoded as (major << 8) | minor (e.g. 0x0508 for 5.8)
    m_depthProc->setFirmwareVersion(static_cast<uint16_t>(m_fwVersion & 0xFFFF));

    // 6. Enable laser by default
    m_fwCmd->setLaser(true);

    // 7. Query device frequency (AlgorithmParams type 0x80)
    uint8_t freqBuf[16];
    int freqSize = sizeof(freqBuf);
    if (m_fwCmd->algorithmParams(FirmwareCmd::ALG_FREQUENCY, 0, 0, freqBuf, &freqSize)) {
        float deviceFreq = 0.0f;
        if (freqSize >= 4) {
            memcpy(&deviceFreq, freqBuf, 4);
        }
    }

    m_initialized = true;
    return true;
}

bool AstraDevice::initFirmware()
{
    if (!m_fwCmd->init()) {
        return false;
    }

    m_fwVersion = m_fwCmd->firmwareVersion();
    m_serialNumber = m_fwCmd->serialNumber();

    // Apply InitFWParams-style defaults from official driver.
    // These are the SetParam calls that InitFWParams sends after GetUsbCoreType
    // in XnDeviceSensorConfigureVersion. They configure the firmware's depth
    // processing pipeline.
    //
    // Without these, the firmware may use different default parameters that
    // produce incorrect depth data (shifts in wrong range, etc.).
    //
    // Captured via gdb on official liborbbec.so:
    //   PARAM_DEPTH_FORMAT=3 (Packed11), PARAM_DEPTH_RESOLUTION=1 (VGA),
    //   PARAM_DEPTH_FPS=30, PARAM_DEPTH_GAIN=1,
    //   PARAM_DEPTH_CONST_SHIFT=42, PARAM_DEPTH_SHIFT_SCALE=0,
    //   PARAM_DEPTH_SHIFT_SKIP=1, PARAM_DEPTH_REGISTRATION=1
    m_fwCmd->setParam(21, 42);   // PARAM_DEPTH_CONST_SHIFT
    m_fwCmd->setParam(24, 0);    // PARAM_DEPTH_SHIFT_SCALE
    m_fwCmd->setParam(23, 1);    // PARAM_DEPTH_SHIFT_SKIP

    return true;
}

// ---------------------------------------------------------------------------
// Sensor info
// ---------------------------------------------------------------------------

void AstraDevice::buildSensorInfos()
{
    // Depth modes: 160x120@30, 320x240@30, 640x480@30, 1280x1024@7
    m_depthModes[0] = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 160, 120, 30};
    m_depthModes[1] = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 320, 240, 30};
    m_depthModes[2] = {ONI_PIXEL_FORMAT_DEPTH_1_MM, 640, 480, 30};
    m_depthModes[3] = {ONI_PIXEL_FORMAT_DEPTH_100_UM, 1280, 1024, 7};

    m_sensorInfos[0].sensorType = ONI_SENSOR_DEPTH;
    m_sensorInfos[0].numSupportedVideoModes = NUM_DEPTH_MODES;
    m_sensorInfos[0].pSupportedVideoModes = m_depthModes;

    // IR modes: 320x240@30, 320x240@60, 640x480@30, 1280x1024@30
    m_irModes[0] = {ONI_PIXEL_FORMAT_GRAY16, 320, 240, 30};
    m_irModes[1] = {ONI_PIXEL_FORMAT_GRAY16, 320, 240, 60};
    m_irModes[2] = {ONI_PIXEL_FORMAT_GRAY8, 1280, 1024, 30};

    m_sensorInfos[1].sensorType = ONI_SENSOR_IR;
    m_sensorInfos[1].numSupportedVideoModes = NUM_IR_MODES;
    m_sensorInfos[1].pSupportedVideoModes = m_irModes;
}

// ---------------------------------------------------------------------------
// Shift-to-depth LUT
// ---------------------------------------------------------------------------

void AstraDevice::computeShiftToDepthTable()
{
    // PrimeSense ShiftToDepth formula.
    // Constants come from readCameraParams() which reads them from firmware
    // (GetFixedParams + AlgorithmParams). Falls back to hardcoded defaults
    // if firmware reads fail.
    // With ShiftScale=10, the formula output is already in mm (DEPTH_1_MM).
    // Reference: openni-sensor-primesense/Source/XnDDK/XnShiftToDepth.cpp:48-103

    const int maxShift = 2048;
    m_s2dTable.resize(maxShift, 0);

    const int    ParamCoeff      = m_s2dConfig.ParamCoeff;
    const int    ConstShift      = m_s2dConfig.ConstShift;
    const int    ShiftScale      = m_s2dConfig.ShiftScale;
    const double ZPD             = m_s2dConfig.ZPD;
    const double ZPPS            = m_s2dConfig.ZPPS;
    const double DCL             = m_s2dConfig.DCL;
    const int    pixelSizeFactor = m_s2dConfig.pixelSizeFactor;

    // nDeviceMaxDepthValue from official = 10000. Use as upper cutoff so
    // high-shift outliers (shift > ~700 → depth > ~1m) that produce
    // unrealistic depths don't pollute the output.
    const uint16_t MIN_DEPTH = 0;         // mm — official sets MinCutOff=0
    const uint16_t MAX_DEPTH = 10000;     // mm — nDeviceMaxDepthValue

    // PrimeSense formula transforms
    const double nConstShiftEff = (double)(ParamCoeff * ConstShift) / pixelSizeFactor;
    const double planePixelSize = ZPPS * pixelSizeFactor;

    int validCount = 0, minShiftValid = maxShift, maxShiftValid = 0;
    for (int s = 1; s < maxShift; s++) {
        double fixedRefX = (double)(s - nConstShiftEff) / ParamCoeff - 0.375;
        double metric    = fixedRefX * planePixelSize;
        if (DCL - metric <= 0) { m_s2dTable[s] = 0; continue; }   // disparity → ∞
        // ShiftScale=10 produces values already in mm for DEPTH_1_MM semantics.
        // Previously we incorrectly divided by 10, making all depths 10x too small.
        double depth_mm = ShiftScale * (metric * ZPD / (DCL - metric) + ZPD);
        if (depth_mm <= MIN_DEPTH || depth_mm >= MAX_DEPTH) { m_s2dTable[s] = 0; continue; }
        m_s2dTable[s] = (uint16_t)depth_mm;
        validCount++;
        if (s < minShiftValid) minShiftValid = s;
        if (s > maxShiftValid) maxShiftValid = s;
    }

    fprintf(stderr,
        "AstraDevice: S2D table (PrimeSense, ZPD=%.1f ZPPS=%.6f DCL=%.1f "
        "CS_eff=%.1f PPS_eff=%.6f): %d valid entries, shift [%d, %d]\n",
        ZPD, ZPPS, DCL, nConstShiftEff, planePixelSize,
        validCount, minShiftValid, maxShiftValid);
}

bool AstraDevice::readCameraParams()
{
    // Read S2D calibration from firmware via GetFixedParams (cmdId 0x0004)
    // and AlgorithmParams (cmdId 0x0016, type=DEPTH_INFO).
    // On failure, keep hardcoded defaults from m_s2dConfig.

    // 1. GetFixedParams — XnFixedParams struct layout (from PrimeSense source
    //    XnDeviceSensor.h:341, verified by gdb capture on Astra Pro FW 5.8.22):
    //      offset 92:  float fDCmosEmitterDistance  (DCL)
    //      offset 96:  float fDCmosRCmosDistance     (DCRCDIS)
    //      offset 100: float fReferenceDistance       (ZPD)
    //      offset 104: float fReferencePixelSize     (ZPPS)
    {
        uint8_t buf[256];
        int size = sizeof(buf);
        if (m_fwCmd->getFixedParams(buf, &size) && size >= 108) {
            float dcl, dcrcdis, zpd, zpps;
            memcpy(&dcl,     buf + 92,  4);
            memcpy(&dcrcdis, buf + 96,  4);
            memcpy(&zpd,     buf + 100, 4);
            memcpy(&zpps,    buf + 104, 4);

            if (std::isfinite(dcl) && dcl > 0)  m_s2dConfig.DCL     = dcl;
            if (std::isfinite(dcrcdis) && dcrcdis > 0) m_s2dConfig.DCRCDIS = dcrcdis;
            if (std::isfinite(zpd) && zpd > 0)  m_s2dConfig.ZPD     = zpd;
            if (std::isfinite(zpps) && zpps > 0) m_s2dConfig.ZPPS    = zpps;

            fprintf(stderr,
                "readCameraParams: GetFixedParams OK (%d bytes) → "
                "DCL=%.6f DCRCDIS=%.6f ZPD=%.6f ZPPS=%.6f\n",
                size, dcl, dcrcdis, zpd, zpps);
        } else {
            fprintf(stderr,
                "readCameraParams: GetFixedParams failed, using defaults "
                "(DCL=%.1f ZPD=%.1f ZPPS=%.6f)\n",
                m_s2dConfig.DCL, m_s2dConfig.ZPD, m_s2dConfig.ZPPS);
        }
    }

    // 2. AlgorithmParams DEPTH_INFO → nConstShift (uint16)
    {
        uint8_t buf[16];
        int size = sizeof(buf);
        if (m_fwCmd->algorithmParams(FirmwareCmd::ALG_DEPTH_INFO, 1, 30, buf, &size)
            && size >= 2) {
            uint16_t constShift;
            memcpy(&constShift, buf, 2);
            if (constShift > 0 && constShift < 4096) {
                m_s2dConfig.ConstShift = constShift;
            }
            fprintf(stderr,
                "readCameraParams: AlgorithmParams DEPTH_INFO → ConstShift=%d\n",
                constShift);
        } else {
            fprintf(stderr,
                "readCameraParams: AlgorithmParams DEPTH_INFO failed, "
                "using default ConstShift=%d\n", m_s2dConfig.ConstShift);
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Resolution mapping
// ---------------------------------------------------------------------------

uint16_t AstraDevice::pixelToFwResolution(int width)
{
    // PrimeSense resolution enum: 0=QQVGA, 1=QVGA, 2=VGA, 3=SXGA
    // Astra Pro uses: 0=QVGA(160x120), 1=VGA(640x480), 2=SXGA(1280x1024)
    switch (width) {
    case 160:
    case 320:  return 0;  // QVGA family
    case 640:  return 1;  // VGA
    case 1280: return 2;  // SXGA
    default:   return 1;  // default VGA
    }
}

// ---------------------------------------------------------------------------
// Stream configuration
// ---------------------------------------------------------------------------

bool AstraDevice::configureDepthStream(int width, int height, int fps)
{
    if (!m_fwCmd) return false;

    uint16_t fwRes = pixelToFwResolution(width);
    m_depthProc->setResolution(width, height);

    // PrimeSense firmware depth format enum (XnIODepthFormats):
    //   0 = Uncompressed16, 1 = PSCompressed, 2 = Uncompressed10, 3 = Packed11
    // Our DepthProcessor uses a different internal enum:
    //   0 = PSCompressed, 1 = Packed11, 2 = Uncompressed16
    // Firmware format and decoder format must match.
    // Default Packed11 (fw fmt 3) — PSCompressed (fw fmt 1) on this fw 0xe752
    // produces nibble streams dominated by 0x0, decoding to garbage. Override
    // for diagnostics with ASTRA_DEPTH_FORMAT={ps,p11,u16}.
    int fwFmt = 3, decFmt = 1;  // Packed11
    const char* fmtEnv = getenv("ASTRA_DEPTH_FORMAT");
    if (fmtEnv) {
        if (strcmp(fmtEnv, "ps") == 0)        { fwFmt = 1; decFmt = 0; }
        else if (strcmp(fmtEnv, "u16") == 0)  { fwFmt = 0; decFmt = 2; }
        else if (strcmp(fmtEnv, "p11") == 0)  { fwFmt = 3; decFmt = 1; }
        fprintf(stderr, "AstraDevice: ASTRA_DEPTH_FORMAT=%s -> fw=%d dec=%d\n",
                fmtEnv, fwFmt, decFmt);
    }
    bool fmtOk = m_fwCmd->setDepthFormat(static_cast<uint16_t>(fwFmt));
    fprintf(stderr, "AstraDevice: setDepthFormat(fw=%d) -> %s, decoder=%d\n",
            fwFmt, fmtOk ? "OK" : "REJECTED", decFmt);
    m_depthProc->setDepthFormat(decFmt);

    if (!m_fwCmd->setDepthResolution(fwRes)) {
        fprintf(stderr, "AstraDevice: setDepthResolution(%d) failed\n", fwRes);
        return false;
    }

    if (!m_fwCmd->setDepthFPS(static_cast<uint16_t>(fps))) {
        fprintf(stderr, "AstraDevice: setDepthFPS(%d) failed\n", fps);
        return false;
    }

    // Query blanking info (PrimeSense caches coefficients for later use)
    uint8_t blankingBuf[64];
    int blankingSize = sizeof(blankingBuf);
    if (!m_fwCmd->algorithmParams(FirmwareCmd::ALG_BLANKING, fwRes,
                                   static_cast<uint16_t>(fps), blankingBuf, &blankingSize)) {
        fprintf(stderr, "AstraDevice: AlgorithmParams(BLANKING, depth) failed\n");
    }

    return true;
}

bool AstraDevice::configureIRStream(int width, int height, int fps)
{
    if (!m_fwCmd) return false;

    uint16_t fwRes = pixelToFwResolution(width);
    m_irProc->setResolution(width, height);

    // PrimeSense IR stream start order (from XnSensorIRStream):
    // 1. SetParam: Stream0Mode = IR (done by caller before bulk read)
    // 2. Start USB read thread (done by caller before this function)
    // 3. ConfigureFirmware: IRResolution, IRFPS
    // 4. SetCmosConfig (AlgorithmParams BLANKING)

    if (!m_fwCmd->setIRResolution(fwRes)) {
        fprintf(stderr, "AstraDevice: setIRResolution(%d) failed\n", fwRes);
        return false;
    }

    if (!m_fwCmd->setIRFPS(static_cast<uint16_t>(fps))) {
        fprintf(stderr, "AstraDevice: setIRFPS(%d) failed\n", fps);
        return false;
    }

    // Query blanking info (PrimeSense caches coefficients for later use)
    uint8_t blankingBuf[64];
    int blankingSize = sizeof(blankingBuf);
    if (!m_fwCmd->algorithmParams(FirmwareCmd::ALG_BLANKING, fwRes,
                                   static_cast<uint16_t>(fps), blankingBuf, &blankingSize)) {
        fprintf(stderr, "AstraDevice: AlgorithmParams(BLANKING, IR) failed\n");
    }

    return true;
}

// ---------------------------------------------------------------------------
// USB bulk read management
// ---------------------------------------------------------------------------

bool AstraDevice::startDepthBulkRead()
{
    if (!m_usbDev || !m_usbDev->isOpen()) {
        return false;
    }

    // Wire up: bulk data -> depth parser -> depth processor
    m_depthParser->setCallback([this](const PacketHeader& header,
                                      const uint8_t* data,
                                      uint32_t dataOffset,
                                      uint32_t dataSize) {
        m_depthProc->processPacket(header, data, dataOffset, dataSize);
    });

    // Notify the depth stream when a new frame is ready
    m_depthProc->setFrameReadyCallback([this]() {
        if (m_depthStream) {
            m_depthStream->onNewFrameReady();
        }
    });

    return m_usbDev->startBulkRead(DEPTH_BULK_EP, [this](uint8_t* data, int size) {
        m_depthParser->feed(data, static_cast<size_t>(size));
    });
}

void AstraDevice::stopDepthBulkRead()
{
    if (m_usbDev && m_usbDev->isOpen()) {
        m_usbDev->stopBulkRead(DEPTH_BULK_EP);
    }
    m_depthParser->reset();

    // Turn off depth stream in firmware
    if (m_fwCmd) {
        m_fwCmd->setStream1Mode(0);  // OFF
    }
}

bool AstraDevice::startIRBulkRead()
{
    if (!m_usbDev || !m_usbDev->isOpen()) {
        fprintf(stderr, "AstraDevice: startIRBulkRead: USB device not open\n");
        return false;
    }

    // Wire up: bulk data -> IR parser -> IR processor
    m_irParser->setCallback([this](const PacketHeader& header,
                                   const uint8_t* data,
                                   uint32_t dataOffset,
                                   uint32_t dataSize) {
        m_irProc->processPacket(header, data, dataOffset, dataSize);
    });

    // Notify the IR stream when a new frame is ready
    m_irProc->setFrameReadyCallback([this]() {
        if (m_irStream) {
            m_irStream->onNewFrameReady();
        }
    });

    bool ok = m_usbDev->startBulkRead(IR_BULK_EP, [this](uint8_t* data, int size) {
        m_irParser->feed(data, static_cast<size_t>(size));
    });
    return ok;
}

void AstraDevice::stopIRBulkRead()
{
    if (m_usbDev && m_usbDev->isOpen()) {
        m_usbDev->stopBulkRead(IR_BULK_EP);
    }
    m_irParser->reset();

    // Turn off IR stream in firmware
    if (m_fwCmd) {
        m_fwCmd->setStream0Mode(0);  // OFF
    }
}

void AstraDevice::stopAllStreams()
{
    if (m_depthStreaming.load()) {
        stopDepthBulkRead();
        m_depthStreaming.store(false);
    }
    if (m_irStreaming.load()) {
        stopIRBulkRead();
        m_irStreaming.store(false);
    }
}
