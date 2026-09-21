#pragma once

#include <cstdint>
#include <string>
#include <vector>

class UsbDevice;

// Firmware command interface for Astra Pro depth camera.
// Implements both XnHostProtocol (PrimeSense) and SendCmd (Orbbec extension)
// commands over USB control transfers.
//
// Packet format (send):
//   [0x4D47 magic(2)][sizeInHalfWords(2)][cmdId(2)][seq(2)][data...]
//
// Response format:
//   [0x4252 magic(2)][size(2)][opcode(2)][reqId(2)][error(2)][data...]
//
// Reference: astra_tool.py (proven on real hardware), ASTRA_PRO_PROTOCOL.md

class FirmwareCmd {
public:
    explicit FirmwareCmd(UsbDevice* usbDev);
    ~FirmwareCmd();

    // --- Initialization ---

    // Read firmware version, serial number. Must be called after USB device is open.
    // Returns true on success (at minimum, firmware version was read).
    bool init();

    // --- Device info (populated by init()) ---

    uint32_t firmwareVersion() const { return fwVersion_; }
    const std::string& serialNumber() const { return serial_; }

    // --- XnHostProtocol: parameter get/set ---

    // Set firmware parameter (XnHostProtocol cmdId from lookup table).
    // Returns true on success.
    bool setParam(uint16_t paramId, uint32_t value);

    // Get firmware parameter. Returns true on success, value written to outValue.
    bool getParam(uint16_t paramId, uint32_t& outValue);

    // --- SendCmd: Orbbec extension commands ---

    // Generic SendCmd interface. Sends cmdId with sendData, receives response.
    // Returns true on success (error code == 0). recvSize is set to response data length.
    bool sendCmd(uint16_t cmdId, const uint8_t* sendData, int sendSize,
                 uint8_t* recvData, int* recvSize, int timeoutMs = 5000);

    // --- Laser control ---

    // Enable/disable IR laser (SendCmd 0x0055). Returns true on success.
    bool setLaser(bool enabled);

    // Query laser state via emitter set point (SendCmd 0x0052).
    // Returns true on success, sets 'on' to true if laser is enabled.
    bool getLaserState(bool& on);

    // --- LDP (Laser Driver Protection) ---

    // Enable/disable LDP (XnHostProtocol cmdId from table, index 28/29).
    bool setLdp(bool enabled);

    // --- IR sensor control ---

    // Set IR gain (XnHostProtocol extended param 0xF1000000).
    bool setIRGain(int gain);

    // Get IR gain.
    bool getIRGain(int& gain);

    // Set IR exposure (XnHostProtocol extended param 0xF1000004).
    bool setIRExposure(int exposure);

    // Get IR exposure.
    bool getIRExposure(int& exposure);

    // --- Stream configuration (SetParam wrappers) ---

    // Set Stream0 mode: 0=OFF, 1=IR
    bool setStream0Mode(uint16_t mode);
    // Set Stream1 mode: 0=OFF, 1=DEPTH
    bool setStream1Mode(uint16_t mode);
    // Set depth output format
    bool setDepthFormat(uint16_t format);
    // Set IR resolution (e.g. 640)
    bool setIRResolution(uint16_t res);
    // Set depth resolution
    bool setDepthResolution(uint16_t res);
    // Set IR FPS
    bool setIRFPS(uint16_t fps);
    // Set depth FPS
    bool setDepthFPS(uint16_t fps);

    // --- Firmware mode and algorithm params ---

    // Set firmware operating mode: 0=OFF, 1=PS (streaming)
    bool setMode(uint16_t mode);

    // Get firmware operating mode. Returns true on success, sets 'mode'.
    bool getMode(uint16_t& mode);

    // Send keep-alive command. Returns true on success.
    bool keepAlive();

    // XnHostProtocolAlgorithmParams: query firmware algorithm parameters.
    // type: algorithm ID (0x00=DepthInfo, 0x02=Registration, 0x03=Padding,
    //       0x06=Blanking, 0x07=DeviceInfo, 0x80=Frequency)
    // resolution: XnResolutions enum (0=QVGA, 1=VGA, 2=SXGA)
    // fps: frame rate
    // recvData/recvSize: response data buffer
    bool algorithmParams(uint16_t type, uint16_t resolution, uint16_t fps,
                         uint8_t* recvData, int* recvSize);

    // Resolution enum for algorithmParams
    enum Resolution : uint16_t {
        RES_QVGA = 0,
        RES_VGA  = 1,
        RES_SXGA = 2,
    };

    // Algorithm type IDs for algorithmParams
    enum AlgorithmType : uint16_t {
        ALG_DEPTH_INFO   = 0x00,
        ALG_REGISTRATION = 0x02,
        ALG_PADDING      = 0x03,
        ALG_BLANKING     = 0x06,
        ALG_DEVICE_INFO  = 0x07,
        ALG_FREQUENCY    = 0x80,
    };

    // --- USB core type ---

    // Query USB core type via XnHostProtocol cmdId 0x0028.
    // Returns true on success, coreType set to the uint16 response value.
    // Official driver calls this right after GetVersion during init.
    // The coreType value feeds into InitFWParams to select correct parameter set.
    bool getUsbCoreType(uint16_t& coreType);

    // --- Fixed params (camera calibration) ---

    // Read camera calibration params via XnHostProtocol GetFixedParams (cmd 4).
    // Returns raw FixedParams data. Caller parses the specific float fields.
    // recvData: output buffer, recvSize: set to actual bytes read.
    bool getFixedParams(uint8_t* recvData, int* recvSize);

    // --- Flash read ---

    // Read flash data (SendCmd 0x0019). Returns true on success.
    // addr: flash address (e.g. 0x10000 for camera params).
    // size: bytes to read (max 256).
    // data: output buffer, dataSize: set to actual bytes read.
    bool readFlash(uint32_t addr, uint16_t size, uint8_t* data, int* dataSize);

    // Read I2C flash data (XnHostProtocol XN_READ_I2C, cmdId 0x000B).
    // Used by official driver on chip_id=0x06 devices to read calibration
    // parameters at addr 0x70000. Reads in 32-byte chunks, reassembled internally.
    // Returns true on success. dataSize is set to actual bytes read.
    bool readI2CFlash(uint32_t addr, uint16_t size, uint8_t* data, int* dataSize);

    // Convenience: send a raw cmdId and get the response data.
    // Returns true if error code == 0. Exposed for sending ad-hoc Orbbec
    // custom commands not in our PrimeSense opcode helpers.
    bool sendRecv(uint16_t cmdId,
                  const uint8_t* sendData, int sendSize,
                  uint8_t* recvData, int* recvDataSize,
                  int timeoutMs = 5000);

private:
    // Low-level XnHostProtocol send/receive.
    // sendCommand: build header, send via controlWrite.
    // Returns bytes sent, or negative on error.
    int sendCommand(uint16_t cmdId, const uint8_t* data, int dataSize);

    // receiveResponse: read response via controlRead, validate magic and error.
    // Returns response data length (bytes after 10-byte header), or negative on error.
    int receiveResponse(uint8_t* buf, int bufSize, int timeoutMs = 5000);

    UsbDevice* m_usbDev;
    uint32_t fwVersion_ = 0;
    std::string serial_;
    uint16_t seq_ = 1;  // sequence counter, incremented per command

    // XnHostProtocol cmdId lookup table (firmware-version-dependent).
    // Index maps to function, value is the cmdId to send.
    // 0xFFFF means unsupported by this firmware.
    static constexpr int CMD_TABLE_SIZE = 88;
    uint16_t cmdTable_[CMD_TABLE_SIZE];

    // Known cmdIds for the most common XnHostProtocol operations.
    // These are the values for "newer firmware" (r13b > 2), which is
    // what Astra Pro devices typically run.
    enum XnCmd : uint16_t {
        XN_GET_VERSION      = 0x0000,
        XN_KEEP_ALIVE       = 0x0001,
        XN_GET_PARAM        = 0x0002,
        XN_SET_PARAM        = 0x0003,
        XN_GET_FIXED_PARAMS = 0x0004,
        XN_GET_MODE         = 0x0005,
        XN_SET_MODE         = 0x0006,
        XN_ALGORITHM_PARAMS = 0x0016,
        XN_WRITE_I2C        = 0x000A,
        XN_READ_I2C         = 0x000B,
        XN_READ_AHB         = 0x0014,
        XN_WRITE_AHB        = 0x0015,
        XN_GET_USB_CORE_TYPE= 0x0028,
        XN_READ_FLASH       = 0x001C,
    };

    // SendCmd extension cmdIds (hardcoded, not firmware-dependent).
    enum SendCmdId : uint16_t {
        SC_ERASE_FLASH  = 0x000D,  // DANGEROUS - never use
        SC_WRITE_FLASH  = 0x000E,  // DANGEROUS - never use
        SC_READ_FLASH   = 0x0019,
        SC_LASER_ENABLE = 0x0055,
    };

    // Orbbec vendor extension cmdIds (device responds, not in XnHostProtocol table).
    enum VendorCmdId : uint16_t {
        VC_GET_SERIAL     = 0x0025,
        VC_GET_FW_VERSION = 0x0026,
        VC_GET_MODEL      = 0x0027,
        VC_GET_LASER_MODE = 0x0050,
        VC_GET_EMITTER_SP = 0x0052,
        VC_GET_EMITTER_ST = 0x0053,
    };

    // Extended param IDs (passed via XnHostProtocol SetParam/GetParam).
    enum ExtParam : uint32_t {
        EXT_IR_GAIN       = 0xF1000000,
        EXT_IR_EXPOSURE   = 0xF1000004,
        EXT_POST_FILTER   = 0xF3000010,
        EXT_LASER_CURRENT = 0xF4000000,
        EXT_LASER_TIME    = 0xF4000004,
    };

    // Stream mode param IDs (passed via SetParam for stream configuration).
    // These come from PrimeSense XnParams.h EConfig_Params.
    enum StreamParam : uint16_t {
        SP_STREAM0_MODE     = 5,   // PARAM_GENERAL_STREAM0_MODE (IR stream)
        SP_STREAM1_MODE     = 6,   // PARAM_GENERAL_STREAM1_MODE (Depth stream)
        SP_DEPTH_FORMAT     = 18,  // PARAM_DEPTH_FORMAT
        SP_IR_RESOLUTION    = 26,  // PARAM_IR_RESOLUTION
        SP_DEPTH_RESOLUTION = 19,  // PARAM_DEPTH_RESOLUTION
        SP_IR_FPS           = 27,  // PARAM_IR_FPS
        SP_DEPTH_FPS        = 20,  // PARAM_DEPTH_FPS
        SP_LASER_ON_OFF     = 0x50, // PARAM_LASER_ONOFF (emitter enable)
    };

    // Send magic (host -> device)
    static constexpr uint16_t SEND_MAGIC = 0x4D47;  // "MG"
    // Response magic (device -> host)
    static constexpr uint16_t RECV_MAGIC = 0x4252;  // "BR"

    // Header sizes
    static constexpr int SEND_HEADER_SIZE = 8;   // 4 x uint16_t
    static constexpr int RECV_HEADER_SIZE = 10;  // 5 x uint16_t
};
