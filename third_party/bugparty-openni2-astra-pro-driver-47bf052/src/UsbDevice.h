#pragma once

#include <libusb-1.0/libusb.h>
#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <map>

struct UsbDeviceInfo {
    std::string uri;   // e.g. "2bc5:0403@bus3,dev10"
    uint16_t vid;
    uint16_t pid;
    uint8_t bus;
    uint8_t address;
};

class UsbDevice {
public:
    UsbDevice();
    ~UsbDevice();

    // Disable copy
    UsbDevice(const UsbDevice&) = delete;
    UsbDevice& operator=(const UsbDevice&) = delete;

    // Device discovery: find all matching 0x2bc5:0x0403 devices
    static std::vector<UsbDeviceInfo> enumerate();

    // Lifecycle
    bool open(const char* uri);  // open by URI string, claim interface 0
    bool openByVidPid(uint16_t vid, uint16_t pid);  // open first matching device
    void close();

    // Control transfer (firmware commands)
    // controlWrite: vendor OUT (bmRequestType=0x40)
    int controlWrite(uint8_t request, uint16_t value, uint16_t index,
                     const uint8_t* data, uint16_t length);
    // controlRead: vendor IN (bmRequestType=0xC0)
    int controlRead(uint8_t request, uint16_t value, uint16_t index,
                    uint8_t* data, uint16_t length);

    // Bulk transfer (streaming data) — supports concurrent reads on different endpoints
    using BulkCallback = std::function<void(uint8_t* data, int size)>;
    bool startBulkRead(int endpoint, BulkCallback callback);
    void stopBulkRead(int endpoint);

    // Query
    bool isOpen() const;
    libusb_device_handle* handle() const;

private:
    bool claimInterface0();
    void bulkReadLoop(int endpoint);

    // Per-endpoint bulk read state
    struct BulkReadState {
        std::thread thread;
        std::atomic<bool> running{false};
        BulkCallback callback;
    };

    libusb_context* ctx_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    bool interfaceClaimed_ = false;
    bool kernelDriverDetached_ = false;

    std::map<int, BulkReadState> bulkReads_;
    std::mutex bulkMutex_;
};
