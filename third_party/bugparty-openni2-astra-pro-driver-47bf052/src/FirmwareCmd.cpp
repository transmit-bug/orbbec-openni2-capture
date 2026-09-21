#include "FirmwareCmd.h"
#include "UsbDevice.h"

#include <cstdio>
#include <cstring>
#include <algorithm>

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

FirmwareCmd::FirmwareCmd(UsbDevice* usbDev)
    : m_usbDev(usbDev)
{
    // Initialize cmdId table with "newer firmware" values (r13b > 2).
    // This matches what real Astra Pro devices run.
    // 0xFFFF = unsupported.
    memset(cmdTable_, 0xFF, sizeof(cmdTable_));

    // Base v1.1 table
    cmdTable_[0]  = XN_GET_VERSION;       // 0x0000
    cmdTable_[1]  = XN_KEEP_ALIVE;        // 0x0001
    cmdTable_[2]  = XN_GET_PARAM;         // 0x0002
    cmdTable_[3]  = XN_SET_PARAM;         // 0x0003
    cmdTable_[4]  = XN_GET_FIXED_PARAMS;  // 0x0004
    cmdTable_[5]  = XN_GET_MODE;          // 0x0005
    cmdTable_[6]  = XN_SET_MODE;          // 0x0006
    cmdTable_[7]  = XN_ALGORITHM_PARAMS;  // 0x0016
    // 8: Reset (UNSUP)
    // 9: SetCmosBlanking (opcode 0x0022 for FW >= 5.0)
    cmdTable_[9] = 0x0022;
    // 10-14: UNSUP (GetCmosBlanking, Presets, SN, CfgPN, Tec)
    cmdTable_[15] = 0x0008;  // GetCMOSRegister
    cmdTable_[16] = 0x0009;  // SetCMOSRegister
    // 17-18: UNSUP in base, overridden below for newer firmware
    cmdTable_[19] = XN_READ_AHB;          // 0x0014
    cmdTable_[20] = XN_WRITE_AHB;         // 0x0015
    // 21: UNSUP (GetPlatformString)
    cmdTable_[22] = XN_GET_USB_CORE_TYPE; // 0x0028
    // 23-63: UNSUP (LED, Emitter, IR, LDP, AE, etc.)
    cmdTable_[64] = 0x0007;  // GetLog
    cmdTable_[65] = 0x000C;  // TakeSnapshot
    cmdTable_[66] = 0x000D;  // InitUpload (DANGEROUS - overlaps SendCmd EraseFlash)
    cmdTable_[67] = 0x000E;  // WriteUpload (DANGEROUS - overlaps SendCmd WriteFlash)
    cmdTable_[68] = 0x000F;  // FinishUpload
    cmdTable_[69] = 0x0010;  // FileDownloadChunk
    cmdTable_[70] = 0x0011;  // DeleteFile
    cmdTable_[71] = 0x0012;  // GetFlashMap
    cmdTable_[72] = 0x0013;  // GetFileList
    cmdTable_[73] = 0x0017;  // SetFileAttributes
    cmdTable_[74] = 0x0018;  // Unknown

    // Newer firmware overrides (r13b > 2)
    cmdTable_[17] = XN_WRITE_I2C;   // 0x000A
    cmdTable_[18] = XN_READ_I2C;    // 0x000B
    cmdTable_[75] = 0x0019;  // FileDownload (overlaps SendCmd ReadFlash)
    cmdTable_[76] = XN_READ_FLASH;  // 0x001C
    cmdTable_[77] = 0x001A;  // Unknown
    cmdTable_[78] = 0x001B;  // GetCPUStats
}

FirmwareCmd::~FirmwareCmd()
{
}

// ---------------------------------------------------------------------------
// Low-level send/receive
// ---------------------------------------------------------------------------

int FirmwareCmd::sendCommand(uint16_t cmdId, const uint8_t* data, int dataSize)
{
    if (!m_usbDev || !m_usbDev->isOpen()) {
        fprintf(stderr, "FirmwareCmd::sendCommand: USB device not open\n");
        return -1;
    }

    // Build packet: [magic(2)][sizeInHalfWords(2)][cmdId(2)][seq(2)][data...]
    int totalSize = SEND_HEADER_SIZE + dataSize;
    uint16_t sizeInHalfWords = static_cast<uint16_t>(totalSize / 2);

    std::vector<uint8_t> packet(totalSize);
    // Little-endian encoding
    packet[0] = static_cast<uint8_t>(SEND_MAGIC & 0xFF);       // magic lo
    packet[1] = static_cast<uint8_t>((SEND_MAGIC >> 8) & 0xFF); // magic hi
    packet[2] = static_cast<uint8_t>(sizeInHalfWords & 0xFF);
    packet[3] = static_cast<uint8_t>((sizeInHalfWords >> 8) & 0xFF);
    packet[4] = static_cast<uint8_t>(cmdId & 0xFF);
    packet[5] = static_cast<uint8_t>((cmdId >> 8) & 0xFF);
    packet[6] = static_cast<uint8_t>(seq_ & 0xFF);
    packet[7] = static_cast<uint8_t>((seq_ >> 8) & 0xFF);
    seq_++;  // increment sequence for next command

    if (data && dataSize > 0) {
        memcpy(packet.data() + SEND_HEADER_SIZE, data, dataSize);
    }

    // Send via USB control transfer: bmRequestType=0x40, bRequest=0
    int rc = m_usbDev->controlWrite(0x00, 0x0000, 0x0000,
                                     packet.data(), static_cast<uint16_t>(totalSize));
    if (rc < 0) {
        fprintf(stderr, "FirmwareCmd::sendCommand: controlWrite failed (rc=%d)\n", rc);
        return -1;
    }

    return rc;
}

int FirmwareCmd::receiveResponse(uint8_t* buf, int bufSize, int timeoutMs)
{
    if (!m_usbDev || !m_usbDev->isOpen()) {
        fprintf(stderr, "FirmwareCmd::receiveResponse: USB device not open\n");
        return -1;
    }

    // Read via USB control transfer: bmRequestType=0xC0, bRequest=0
    int rc = m_usbDev->controlRead(0x00, 0x0000, 0x0000,
                                    buf, static_cast<uint16_t>(bufSize));
    if (rc < RECV_HEADER_SIZE) {
        fprintf(stderr, "FirmwareCmd::receiveResponse: short read (%d bytes, need %d)\n",
                rc, RECV_HEADER_SIZE);
        return -1;
    }

    // Parse response header: [magic(2)][size(2)][opcode(2)][reqId(2)][error(2)]
    uint16_t magic = static_cast<uint16_t>(buf[0] | (buf[1] << 8));
    uint16_t error = static_cast<uint16_t>(buf[8] | (buf[9] << 8));

    if (magic != RECV_MAGIC) {
        fprintf(stderr, "FirmwareCmd::receiveResponse: bad magic 0x%04x (expected 0x%04x)\n",
                magic, RECV_MAGIC);
        return -1;
    }

    // Check error code
    if (error != 0) {
        fprintf(stderr, "FirmwareCmd::receiveResponse: device error 0x%04x\n", error);
        // Return negative error code but still allow caller to read response data
        return -static_cast<int>(error);
    }

    // Return number of data bytes (after header)
    int dataLen = rc - RECV_HEADER_SIZE;
    return dataLen;
}

bool FirmwareCmd::sendRecv(uint16_t cmdId,
                            const uint8_t* sendData, int sendSize,
                            uint8_t* recvData, int* recvDataSize,
                            int timeoutMs)
{
    int sendRc = sendCommand(cmdId, sendData, sendSize);
    if (sendRc < 0) {
        if (recvDataSize) *recvDataSize = 0;
        return false;
    }

    uint8_t respBuf[512];
    int respRc = receiveResponse(respBuf, sizeof(respBuf), timeoutMs);

    if (respRc < 0) {
        // Error code is negative of the firmware error value.
        fprintf(stderr, "FirmwareCmd: sendRecv cmdId=0x%04x FAILED, firmware error 0x%04x\n",
                cmdId, static_cast<uint16_t>(-respRc));
        // If we got data despite error, copy it out for debugging.
        if (recvData && recvDataSize) {
            int headerSize = RECV_HEADER_SIZE;
            // respRc is negative; the actual bytes read may include data
            // We need to re-read to get data. But controlRead already returned.
            // The data is in respBuf starting at offset 10.
            int actualDataLen = 0;
            // Check if we actually received enough bytes
            int totalRead = 0;
            // We need the raw byte count. Let's re-approach:
            // receiveResponse returned < 0, meaning error.
            // But the data might still be in respBuf if enough was read.
            // Actually, receiveResponse returns -error_code on firmware error.
            // The data length from controlRead could have been >= 10.
            // Let me just re-do the raw read here for the error path too.
            *recvDataSize = 0;
        }
        return false;
    }

    // Success: copy response data
    if (recvData && recvDataSize) {
        int dataLen = respRc;  // data bytes after header
        int copyLen = std::min(dataLen, 512);  // safety clamp
        memcpy(recvData, respBuf + RECV_HEADER_SIZE, copyLen);
        *recvDataSize = copyLen;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool FirmwareCmd::init()
{
    if (!m_usbDev || !m_usbDev->isOpen()) {
        fprintf(stderr, "FirmwareCmd::init: USB device not open\n");
        return false;
    }

    // 1. Read firmware version via PrimeSense GetVersion (cmd 0x0000).
    // This returns XnVersions struct: nMajor(u8), nMinor(u8), nBuild(u16), ...
    // The first 2 bytes need byte-swapping (PrimeSense convention).
    uint8_t respData[64];
    int respSize = 0;
    uint16_t cmdGetVer = cmdTable_[0];  // index 0 = GetVersion

    if (cmdGetVer != 0xFFFF && sendRecv(cmdGetVer, nullptr, 0, respData, &respSize)) {
        if (respSize >= 2) {
            // Byte-swap first 2 bytes as PrimeSense does (xnOSEndianSwapUINT16)
            uint8_t swapped[2] = {respData[1], respData[0]};
            uint8_t nMajor = swapped[0];
            uint8_t nMinor = swapped[1];
            // Encode as (major << 8) | minor for version comparison.
            // e.g. FW 5.8 → 0x0508, FW 5.1 → 0x0501
            fwVersion_ = static_cast<uint32_t>((nMajor << 8) | nMinor);
            uint16_t nBuild = 0;
            if (respSize >= 4) {
                nBuild = static_cast<uint16_t>(respData[2] | (respData[3] << 8));
            }
            fprintf(stderr, "FirmwareCmd: PrimeSense GetVersion: %d.%d (build %d, encoded 0x%04x)\n",
                    nMajor, nMinor, nBuild, fwVersion_);
        }
    } else {
        fprintf(stderr, "FirmwareCmd: PrimeSense GetVersion failed, trying Orbbec 0x0026\n");
        // Fallback: try Orbbec vendor cmd 0x0026 (different format)
        if (sendRecv(VC_GET_FW_VERSION, nullptr, 0, respData, &respSize)) {
            if (respSize >= 4) {
                // Orbbec format: raw 32-bit value, not PrimeSense major.minor
                uint32_t raw = static_cast<uint32_t>(
                    respData[0] | (respData[1] << 8) |
                    (respData[2] << 16) | (respData[3] << 24));
                // Can't reliably extract major.minor from Orbbec format,
                // assume >= 5.1 for Astra Pro (all known devices are >= 5.1)
                fwVersion_ = 0x0508;  // assume 5.8
                fprintf(stderr, "FirmwareCmd: Orbbec FW version 0x%08x, assuming 5.8\n", raw);
            }
        }
    }

    // 1b. Get USB core type (cmdId 0x0028) — official driver sends this
    // right after GetVersion, before GetSerial. Response is a uint16
    // core type value (e.g., 2 for this device).
    uint16_t usbCoreType = 0;
    {
        uint16_t cmdUsbCore = cmdTable_[22];  // index 22 = GetUsbCoreType
        if (cmdUsbCore != 0xFFFF) {
            uint8_t ucResp[16];
            int ucRespSize = 0;
            if (sendRecv(cmdUsbCore, nullptr, 0, ucResp, &ucRespSize)) {
                if (ucRespSize >= 2) {
                    usbCoreType = static_cast<uint16_t>(ucResp[0] | (ucResp[1] << 8));
                }
                fprintf(stderr, "FirmwareCmd: GetUsbCoreType OK, coreType=%u\n", usbCoreType);
            } else {
                fprintf(stderr, "FirmwareCmd: GetUsbCoreType failed (non-fatal)\n");
            }
        }
    }

    // 2. Read serial number via vendor extension cmd 0x0025
    respSize = 0;
    if (sendRecv(VC_GET_SERIAL, nullptr, 0, respData, &respSize)) {
        if (respSize > 0) {
            // Serial is ASCII, null-terminated, up to 32 bytes
            int len = std::min(respSize, 32);
            // Strip trailing nulls
            while (len > 0 && respData[len - 1] == 0) len--;
            serial_.assign(reinterpret_cast<char*>(respData), len);
        }
    } else {
        fprintf(stderr, "FirmwareCmd: GetSerialNumber (0x0025) failed\n");
        // Fallback: try XnHostProtocol GetSerialNumber (index 12)
        uint16_t cmdGetSN = cmdTable_[12];
        if (cmdGetSN != 0xFFFF) {
            if (sendRecv(cmdGetSN, nullptr, 0, respData, &respSize)) {
                if (respSize > 0) {
                    int len = std::min(respSize, 32);
                    while (len > 0 && respData[len - 1] == 0) len--;
                    serial_.assign(reinterpret_cast<char*>(respData), len);
                }
            }
        }
    }

    // Consider init successful if we got a firmware version
    if (fwVersion_ == 0) {
        fprintf(stderr, "FirmwareCmd::init: could not read firmware version\n");
        // Don't fail completely — some operations may still work
    }

    return true;
}

// ---------------------------------------------------------------------------
// XnHostProtocol: getUsbCoreType
// ---------------------------------------------------------------------------

bool FirmwareCmd::getUsbCoreType(uint16_t& coreType)
{
    // XnHostProtocol index 22 = GetUsbCoreType (cmdId 0x0028)
    uint16_t cmdId = cmdTable_[22];
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd::getUsbCoreType: not supported by firmware\n");
        return false;
    }

    // Empty-payload query: send header only, get back a 2-byte uint16
    uint8_t respData[16];
    int respSize = 0;
    if (!sendRecv(cmdId, nullptr, 0, respData, &respSize)) {
        fprintf(stderr, "FirmwareCmd::getUsbCoreType: cmdId 0x%04x failed\n", cmdId);
        return false;
    }

    if (respSize < 2) {
        fprintf(stderr, "FirmwareCmd::getUsbCoreType: response too short (%d bytes)\n", respSize);
        return false;
    }

    coreType = static_cast<uint16_t>(respData[0] | (respData[1] << 8));
    fprintf(stderr, "FirmwareCmd::getUsbCoreType: coreType=%u\n", coreType);
    return true;
}

// ---------------------------------------------------------------------------
// XnHostProtocol: setParam / getParam
// ---------------------------------------------------------------------------

bool FirmwareCmd::setParam(uint16_t paramId, uint32_t value)
{
    // SetParam is XnHostProtocol index 3
    uint16_t cmdId = cmdTable_[3];
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd::setParam: SetParam not supported by firmware\n");
        return false;
    }

    // PrimeSense XnHostProtocolSetParam payload: [paramId(u16)][value(u16)] = 4 bytes.
    // The value is truncated to 16 bits — this is the standard protocol format.
    uint8_t payload[4];
    payload[0] = static_cast<uint8_t>(paramId & 0xFF);
    payload[1] = static_cast<uint8_t>((paramId >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>(value & 0xFF);
    payload[3] = static_cast<uint8_t>((value >> 8) & 0xFF);

    return sendRecv(cmdId, payload, sizeof(payload), nullptr, nullptr);
}

bool FirmwareCmd::getParam(uint16_t paramId, uint32_t& outValue)
{
    // GetParam is XnHostProtocol index 2
    uint16_t cmdId = cmdTable_[2];
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd::getParam: GetParam not supported by firmware\n");
        return false;
    }

    // Build payload: [paramId(2)] = 2 bytes
    uint8_t payload[2];
    payload[0] = static_cast<uint8_t>(paramId & 0xFF);
    payload[1] = static_cast<uint8_t>((paramId >> 8) & 0xFF);

    uint8_t respData[64];
    int respSize = 0;

    if (!sendRecv(cmdId, payload, sizeof(payload), respData, &respSize)) {
        return false;
    }

    if (respSize < 2) {
        fprintf(stderr, "FirmwareCmd::getParam: response too short (%d bytes)\n", respSize);
        return false;
    }

    // GetParam may return 2 or 4 bytes depending on the parameter.
    // Protocol reference says 2 bytes for most params.
    if (respSize >= 4) {
        outValue = static_cast<uint32_t>(
            respData[0] | (respData[1] << 8) |
            (respData[2] << 16) | (respData[3] << 24));
    } else {
        // 2-byte response: zero-extend to uint32_t
        outValue = static_cast<uint32_t>(respData[0] | (respData[1] << 8));
    }
    return true;
}

// ---------------------------------------------------------------------------
// SendCmd: generic Orbbec extension
// ---------------------------------------------------------------------------

bool FirmwareCmd::sendCmd(uint16_t cmdId, const uint8_t* sendData, int sendSize,
                           uint8_t* recvData, int* recvSize, int timeoutMs)
{
    // Block dangerous commands
    if (cmdId == SC_ERASE_FLASH || cmdId == SC_WRITE_FLASH) {
        fprintf(stderr, "FirmwareCmd::sendCmd: BLOCKED dangerous cmd 0x%04x\n", cmdId);
        return false;
    }

    return sendRecv(cmdId, sendData, sendSize, recvData, recvSize, timeoutMs);
}

// ---------------------------------------------------------------------------
// Laser control
// ---------------------------------------------------------------------------

bool FirmwareCmd::setLaser(bool enabled)
{
    // SendCmd 0x0055: Laser Enable
    // Data: 1 byte, 0=off, 1=on
    // Proven method: works on Astra Pro (8.6% valid pixels with laser on).
    // Official driver uses SendCmd(0x0015, 42) but that goes through a different
    // code path (XnHostProtocol SendCmd wrapper) that we don't replicate yet.
    // TODO: Implement proper SendCmd wrapper for 0x0015 method.
    uint8_t data = enabled ? 1 : 0;
    if (!sendRecv(SC_LASER_ENABLE, &data, 1, nullptr, nullptr)) {
        fprintf(stderr, "FirmwareCmd::setLaser: SendCmd 0x0055 failed\n");
        return false;
    }
    return true;
}

bool FirmwareCmd::getLaserState(bool& on)
{
    // Try SendCmd 0x0050 (GetLaserMode) first.
    // Response: 6 bytes [mode(2), PWM(2), current(2)].
    // If mode > 0, laser is enabled.
    uint8_t respData[16];
    int respSize = 0;

    if (sendRecv(VC_GET_LASER_MODE, nullptr, 0, respData, &respSize, 1000) && respSize >= 2) {
        uint16_t mode = static_cast<uint16_t>(respData[0] | (respData[1] << 8));
        on = (mode > 0);
        return true;
    }

    // Fallback: emitter set point via 0x0052
    // Response: 2 bytes. 0x00a4 = on, 0x0000 = off
    respSize = 0;
    if (sendRecv(VC_GET_EMITTER_SP, nullptr, 0, respData, &respSize, 1000) && respSize >= 2) {
        uint16_t setPoint = static_cast<uint16_t>(respData[0] | (respData[1] << 8));
        on = (setPoint > 0);
        return true;
    }

    fprintf(stderr, "FirmwareCmd::getLaserState: all queries failed\n");
    return false;
}

// ---------------------------------------------------------------------------
// LDP control
// ---------------------------------------------------------------------------

bool FirmwareCmd::setLdp(bool enabled)
{
    // SetLdpEnable is XnHostProtocol index 28
    uint16_t cmdId = cmdTable_[28];
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd::setLdp: SetLdpEnable not supported by firmware\n");
        return false;
    }

    // LDP enable: single byte param, 0=disable, 1=enable
    uint8_t data = enabled ? 1 : 0;
    return sendRecv(cmdId, &data, 1, nullptr, nullptr);
}

// ---------------------------------------------------------------------------
// IR sensor control (via extended param IDs)
// ---------------------------------------------------------------------------

bool FirmwareCmd::setIRGain(int gain)
{
    // Extended param 0xF1000000 = IR Gain
    // Use SetParam with the extended param ID.
    // The firmware SetParam dispatches extended params differently.
    // We send it as: [extendedParam(4)][value(4)]
    uint16_t cmdId = cmdTable_[3];  // SetParam
    if (cmdId == 0xFFFF) return false;

    uint8_t payload[8];
    uint32_t extParam = EXT_IR_GAIN;
    payload[0] = static_cast<uint8_t>(extParam & 0xFF);
    payload[1] = static_cast<uint8_t>((extParam >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((extParam >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((extParam >> 24) & 0xFF);
    uint32_t val = static_cast<uint32_t>(gain);
    payload[4] = static_cast<uint8_t>(val & 0xFF);
    payload[5] = static_cast<uint8_t>((val >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>((val >> 16) & 0xFF);
    payload[7] = static_cast<uint8_t>((val >> 24) & 0xFF);

    return sendRecv(cmdId, payload, sizeof(payload), nullptr, nullptr);
}

bool FirmwareCmd::getIRGain(int& gain)
{
    // XnHostProtocol index 27 = GetIrGain
    uint16_t cmdId = cmdTable_[27];
    if (cmdId == 0xFFFF) {
        // Fallback: try extended param via GetParam
        // Not directly supported; would need to know the short param ID.
        fprintf(stderr, "FirmwareCmd::getIRGain: GetIrGain not supported by firmware\n");
        return false;
    }

    uint8_t respData[8];
    int respSize = 0;
    if (!sendRecv(cmdId, nullptr, 0, respData, &respSize)) return false;
    if (respSize < 2) return false;

    gain = static_cast<int>(respData[0] | (respData[1] << 8));
    return true;
}

bool FirmwareCmd::setIRExposure(int exposure)
{
    // Extended param 0xF1000004 = IR Exposure
    uint16_t cmdId = cmdTable_[3];  // SetParam
    if (cmdId == 0xFFFF) return false;

    uint8_t payload[8];
    uint32_t extParam = EXT_IR_EXPOSURE;
    payload[0] = static_cast<uint8_t>(extParam & 0xFF);
    payload[1] = static_cast<uint8_t>((extParam >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((extParam >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((extParam >> 24) & 0xFF);
    uint32_t val = static_cast<uint32_t>(exposure);
    payload[4] = static_cast<uint8_t>(val & 0xFF);
    payload[5] = static_cast<uint8_t>((val >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>((val >> 16) & 0xFF);
    payload[7] = static_cast<uint8_t>((val >> 24) & 0xFF);

    return sendRecv(cmdId, payload, sizeof(payload), nullptr, nullptr);
}

bool FirmwareCmd::getIRExposure(int& exposure)
{
    // XnHostProtocol index 43 = GetIrExp
    uint16_t cmdId = cmdTable_[43];
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd::getIRExposure: GetIrExp not supported by firmware\n");
        return false;
    }

    uint8_t respData[8];
    int respSize = 0;
    if (!sendRecv(cmdId, nullptr, 0, respData, &respSize)) return false;
    if (respSize < 2) return false;

    exposure = static_cast<int>(respData[0] | (respData[1] << 8));
    return true;
}

// ---------------------------------------------------------------------------
// Stream configuration (SetParam wrappers)
// ---------------------------------------------------------------------------

bool FirmwareCmd::setStream0Mode(uint16_t mode)
{
    return setParam(SP_STREAM0_MODE, static_cast<uint32_t>(mode));
}

bool FirmwareCmd::setStream1Mode(uint16_t mode)
{
    return setParam(SP_STREAM1_MODE, static_cast<uint32_t>(mode));
}

bool FirmwareCmd::setDepthFormat(uint16_t format)
{
    return setParam(SP_DEPTH_FORMAT, static_cast<uint32_t>(format));
}

bool FirmwareCmd::setIRResolution(uint16_t res)
{
    return setParam(SP_IR_RESOLUTION, static_cast<uint32_t>(res));
}

bool FirmwareCmd::setDepthResolution(uint16_t res)
{
    return setParam(SP_DEPTH_RESOLUTION, static_cast<uint32_t>(res));
}

bool FirmwareCmd::setIRFPS(uint16_t fps)
{
    return setParam(SP_IR_FPS, static_cast<uint32_t>(fps));
}

bool FirmwareCmd::setDepthFPS(uint16_t fps)
{
    return setParam(SP_DEPTH_FPS, static_cast<uint32_t>(fps));
}

// ---------------------------------------------------------------------------
// Firmware mode and algorithm params
// ---------------------------------------------------------------------------

bool FirmwareCmd::setMode(uint16_t mode)
{
    uint16_t cmdId = cmdTable_[6];  // index 6 = SetMode
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd: setMode not supported by firmware\n");
        return false;
    }
    uint8_t modeBuf[2];
    modeBuf[0] = static_cast<uint8_t>(mode & 0xFF);
    modeBuf[1] = static_cast<uint8_t>((mode >> 8) & 0xFF);
    return sendRecv(cmdId, modeBuf, 2, nullptr, nullptr);
}

bool FirmwareCmd::getMode(uint16_t& mode)
{
    uint16_t cmdId = cmdTable_[5];  // index 5 = GetMode
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd: getMode not supported by firmware\n");
        return false;
    }
    uint8_t respData[16];
    int respSize = 0;
    if (!sendRecv(cmdId, nullptr, 0, respData, &respSize)) {
        return false;
    }
    if (respSize < 2) {
        fprintf(stderr, "FirmwareCmd: getMode response too short (%d bytes)\n", respSize);
        return false;
    }
    mode = static_cast<uint16_t>(respData[0] | (respData[1] << 8));
    return true;
}

bool FirmwareCmd::keepAlive()
{
    uint16_t cmdId = cmdTable_[1];  // index 1 = KeepAlive
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd: keepAlive not supported by firmware\n");
        return false;
    }
    return sendRecv(cmdId, nullptr, 0, nullptr, nullptr);
}

bool FirmwareCmd::algorithmParams(uint16_t type, uint16_t resolution, uint16_t fps,
                                   uint8_t* recvData, int* recvSize)
{
    uint16_t cmdId = cmdTable_[7];  // index 7 = AlgorithmParams
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd: algorithmParams not supported by firmware\n");
        return false;
    }

    // Request payload: 10 bytes (5 x uint16_t)
    // [paramID][format=0][resolution][fps][offset=0]
    uint8_t req[10];
    req[0] = static_cast<uint8_t>(type & 0xFF);
    req[1] = static_cast<uint8_t>((type >> 8) & 0xFF);
    req[2] = 0; req[3] = 0;  // format = 0
    req[4] = static_cast<uint8_t>(resolution & 0xFF);
    req[5] = static_cast<uint8_t>((resolution >> 8) & 0xFF);
    req[6] = static_cast<uint8_t>(fps & 0xFF);
    req[7] = static_cast<uint8_t>((fps >> 8) & 0xFF);
    req[8] = 0; req[9] = 0;  // offset = 0

    uint8_t resp[512];
    int respSize = 0;
    if (!sendRecv(cmdId, req, 10, resp, &respSize)) {
        return false;
    }

    if (recvData && recvSize) {
        int copySize = (respSize < *recvSize) ? respSize : *recvSize;
        memcpy(recvData, resp, copySize);
        *recvSize = respSize;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Fixed params (camera calibration via XnHostProtocol GetFixedParams)
// ---------------------------------------------------------------------------

bool FirmwareCmd::getFixedParams(uint8_t* recvData, int* recvSize)
{
    // GetFixedParams is XnHostProtocol index 4
    uint16_t cmdId = cmdTable_[4];
    if (cmdId == 0xFFFF) {
        fprintf(stderr, "FirmwareCmd: getFixedParams not supported by firmware\n");
        return false;
    }

    // PrimeSense reads FixedParams in chunks, passing a word-offset in the
    // request payload. We need to read enough to get fReferenceDistance and
    // fReferencePixelSize, which are at int32 indices 25-26 (byte 100-107).
    // Read 128 bytes (32 uint32s) to be safe.
    const int TOTAL_BYTES = 128;
    int totalRead = 0;

    while (totalRead < TOTAL_BYTES) {
        uint16_t wordOffset = static_cast<uint16_t>(totalRead / sizeof(uint32_t));
        uint8_t payload[2];
        payload[0] = static_cast<uint8_t>(wordOffset & 0xFF);
        payload[1] = static_cast<uint8_t>((wordOffset >> 8) & 0xFF);

        uint8_t resp[512];
        int respSize = 0;

        if (!sendRecv(cmdId, payload, sizeof(payload), resp, &respSize)) {
            fprintf(stderr, "getFixedParams: sendRecv failed at offset %d (respSize=%d)\n", totalRead, respSize);
            break;
        }

        if (respSize <= 0) break;

        // Copy response data into caller's buffer
        int copyLen = respSize;
        if (totalRead + copyLen > TOTAL_BYTES) {
            copyLen = TOTAL_BYTES - totalRead;
        }
        if (recvData && recvSize) {
            memcpy(recvData + totalRead, resp, copyLen);
        }
        totalRead += copyLen;

        if (respSize < static_cast<int>(sizeof(uint32_t))) break;
    }

    if (recvSize) *recvSize = totalRead;
    return totalRead >= 64;  // At minimum we need the geometry fields
}

// ---------------------------------------------------------------------------
// Flash read
// ---------------------------------------------------------------------------

bool FirmwareCmd::readFlash(uint32_t addr, uint16_t size, uint8_t* data, int* dataSize)
{
    // SendCmd 0x0019: ReadFlash
    // Payload: [addr(4)][size(2)] = 6 bytes
    uint8_t payload[6];
    payload[0] = static_cast<uint8_t>(addr & 0xFF);
    payload[1] = static_cast<uint8_t>((addr >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((addr >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((addr >> 24) & 0xFF);
    payload[4] = static_cast<uint8_t>(size & 0xFF);
    payload[5] = static_cast<uint8_t>((size >> 8) & 0xFF);

    return sendRecv(SC_READ_FLASH, payload, sizeof(payload), data, dataSize);
}
