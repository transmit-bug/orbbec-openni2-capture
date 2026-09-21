#pragma once

#include <openni2/driver/OniDriverAPI.h>
#include <string>
#include <map>

class AstraDevice;

class AstraDriver : public oni::driver::DriverBase {
public:
    AstraDriver(OniDriverServices* pDriverServices);
    ~AstraDriver() override;

    OniStatus initialize(
        oni::driver::DeviceConnectedCallback connectedCallback,
        oni::driver::DeviceDisconnectedCallback disconnectedCallback,
        oni::driver::DeviceStateChangedCallback deviceStateChangedCallback,
        void* pCookie) override;

    oni::driver::DeviceBase* deviceOpen(const char* uri, const char* mode) override;
    void deviceClose(oni::driver::DeviceBase* pDevice) override;
    void shutdown() override;
    OniStatus tryDevice(const char* uri) override;

private:
    // Scan for Astra Pro devices and report them to the framework
    void discoverDevices();

    // Known device URIs (populated during discovery)
    std::map<std::string, bool> m_knownDevices;
};
