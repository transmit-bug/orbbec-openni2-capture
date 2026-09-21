#pragma once

#include <openni2/driver/OniDriverAPI.h>
#include <cstdint>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>

class UsbDevice;
class FirmwareCmd;
class PacketParser;
class DepthProcessor;
class IrProcessor;

class AstraDevice : public oni::driver::DeviceBase {
public:
    AstraDevice(const char* uri, oni::driver::DriverServices& driverServices);
    ~AstraDevice() override;

    // DeviceBase overrides
    OniStatus getSensorInfoList(OniSensorInfo** pSensorInfos, int* numSensors) override;
    oni::driver::StreamBase* createStream(OniSensorType sensorType) override;
    void destroyStream(oni::driver::StreamBase* pStream) override;

    OniStatus setProperty(int propertyId, const void* data, int dataSize) override;
    OniStatus getProperty(int propertyId, void* data, int* pDataSize) override;
    OniBool isPropertySupported(int propertyId) override;

    // Accessors for stream objects
    UsbDevice* usbDevice() { return m_usbDev; }
    FirmwareCmd* firmwareCmd() { return m_fwCmd; }
    DepthProcessor* depthProcessor() { return m_depthProc; }
    IrProcessor* irProcessor() { return m_irProc; }

    // Track active stream objects for frame-ready notification
    void setDepthStream(class AstraDepthStream* s) { m_depthStream = s; }
    void setIRStream(class AstraIRStream* s) { m_irStream = s; }
    AstraDepthStream* depthStream() const { return m_depthStream; }
    AstraIRStream* irStream() const { return m_irStream; }

    // ShiftToDepth LUT access
    const uint16_t* shiftToDepthTable() const { return m_s2dTable.data(); }
    int shiftToDepthTableSize() const { return static_cast<int>(m_s2dTable.size()); }

    // Current stream mode tracking
    bool isDepthStreaming() const { return m_depthStreaming; }
    bool isIRStreaming() const { return m_irStreaming; }
    void setDepthStreaming(bool on) { m_depthStreaming = on; }
    void setIRStreaming(bool on) { m_irStreaming = on; }

    // Start/stop USB bulk read for a stream type
    bool startDepthBulkRead();
    void stopDepthBulkRead();
    bool startIRBulkRead();
    void stopIRBulkRead();

    // Configure firmware for stream modes
    bool configureDepthStream(int width, int height, int fps);
    bool configureIRStream(int width, int height, int fps);

    // Turn off all streams
    void stopAllStreams();

private:
    char m_uri[ONI_MAX_STR];
    oni::driver::DriverServices& m_driverServices;

    // Subsystem objects (owned)
    UsbDevice* m_usbDev = nullptr;
    FirmwareCmd* m_fwCmd = nullptr;

    // Frame processors (owned)
    DepthProcessor* m_depthProc = nullptr;
    IrProcessor* m_irProc = nullptr;

    // Packet parsers (owned)
    PacketParser* m_depthParser = nullptr;
    PacketParser* m_irParser = nullptr;

    // Sensor info arrays
    static const int NUM_DEPTH_MODES = 4;
    static const int NUM_IR_MODES = 4;
    OniVideoMode m_depthModes[NUM_DEPTH_MODES];
    OniVideoMode m_irModes[NUM_IR_MODES];
    OniSensorInfo m_sensorInfos[2];

    // Shift-to-depth LUT
    std::vector<uint16_t> m_s2dTable;

    // S2D configuration (from firmware or hardcoded defaults)
    struct S2DConfig {
        int    ParamCoeff      = 4;
        int    ConstShift      = 200;
        int    ShiftScale      = 10;
        double ZPD             = 130.0;
        double ZPPS            = 0.113967;
        double DCL             = 7.5;
        double DCRCDIS         = 2.63;
        int    pixelSizeFactor = 1;
    };
    S2DConfig m_s2dConfig;

    // Device state
    std::string m_serialNumber;
    uint32_t m_fwVersion = 0;
    bool m_initialized = false;
    bool m_usbOpen = false;

    // Stream state
    std::atomic<bool> m_depthStreaming{false};
    std::atomic<bool> m_irStreaming{false};

    // Active stream objects (not owned — set/cleared by stream start/stop)
    class AstraDepthStream* m_depthStream = nullptr;
    class AstraIRStream* m_irStream = nullptr;

    // Initialization helpers
    bool initUsb();
    bool initFirmware();
    void buildSensorInfos();
    void computeShiftToDepthTable();

    // Read camera parameters from flash (reserved for future use)
    bool readCameraParams();

    // Map pixel width to firmware resolution enum (0=QVGA, 1=VGA, 2=SXGA)
    static uint16_t pixelToFwResolution(int width);

    // USB bulk endpoint addresses for Astra Pro (PrimeSense assignment)
    // EP 0x81 = depth, EP 0x82 = image/IR (from XnDeviceSensorIO.cpp)
    static constexpr int DEPTH_BULK_EP = 0x81;  // EP1 IN = depth
    static constexpr int IR_BULK_EP = 0x82;     // EP2 IN = image/IR
};
