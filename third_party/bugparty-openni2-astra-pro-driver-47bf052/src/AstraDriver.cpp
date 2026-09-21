#include "AstraDriver.h"
#include "AstraDevice.h"
#include "UsbDevice.h"

#include <cstdio>

static constexpr uint16_t ASTRA_VID = 0x2bc5;

AstraDriver::AstraDriver(OniDriverServices* pDriverServices)
    : DriverBase(pDriverServices)
{
    fprintf(stderr, "=== AstraDriver (custom) BUILD: 2026-05-17b S2D: PrimeSense(ZPD=130 ZPPS=0.114 DCL=7.5 CS=200 SC=10) maxD=10000 SF=shift-space ===\n");
}

AstraDriver::~AstraDriver()
{
    shutdown();
}

OniStatus AstraDriver::initialize(
    oni::driver::DeviceConnectedCallback connectedCallback,
    oni::driver::DeviceDisconnectedCallback disconnectedCallback,
    oni::driver::DeviceStateChangedCallback deviceStateChangedCallback,
    void* pCookie)
{
    OniStatus rc = DriverBase::initialize(connectedCallback, disconnectedCallback,
                                          deviceStateChangedCallback, pCookie);
    if (rc != ONI_STATUS_OK) {
        return rc;
    }

    discoverDevices();
    return ONI_STATUS_OK;
}

void AstraDriver::discoverDevices()
{
    auto devices = UsbDevice::enumerate();
    fprintf(stderr, "AstraDriver::discoverDevices: found %zu USB devices\n", devices.size());

    for (const auto& dev : devices) {
        if (m_knownDevices.count(dev.uri)) {
            continue;
        }

        m_knownDevices[dev.uri] = true;

        OniDeviceInfo info = {};
        strncpy(info.uri, dev.uri.c_str(), ONI_MAX_STR - 1);
        snprintf(info.vendor, sizeof(info.vendor), "Orbbec");
        snprintf(info.name, sizeof(info.name), "Astra Pro (%04x:%04x)", dev.vid, dev.pid);

        fprintf(stderr, "AstraDriver: reporting device uri='%s' name='%s'\n", info.uri, info.name);
        deviceConnected(&info);
    }
}

oni::driver::DeviceBase* AstraDriver::deviceOpen(const char* uri, const char* /*mode*/)
{
    auto* device = new AstraDevice(uri, getServices());
    return device;
}

void AstraDriver::deviceClose(oni::driver::DeviceBase* pDevice)
{
    if (pDevice) {
        delete pDevice;
    }
}

void AstraDriver::shutdown()
{
}

OniStatus AstraDriver::tryDevice(const char* uri)
{
    std::string uriStr(uri);

    // Accept any URI that looks like an Orbbec Astra Pro
    if (uriStr.find("2bc5:0403") != std::string::npos ||
        uriStr.find("2bc5:0501") != std::string::npos) {
        if (m_knownDevices.find(uri) == m_knownDevices.end()) {
            m_knownDevices[uri] = true;

            OniDeviceInfo info = {};
            strncpy(info.uri, uri, ONI_MAX_STR - 1);
            snprintf(info.vendor, sizeof(info.vendor), "Orbbec");
            snprintf(info.name, sizeof(info.name), "Astra Pro");

            deviceConnected(&info);
        }
        return ONI_STATUS_OK;
    }

    return ONI_STATUS_ERROR;
}

ONI_EXPORT_DRIVER(AstraDriver);
