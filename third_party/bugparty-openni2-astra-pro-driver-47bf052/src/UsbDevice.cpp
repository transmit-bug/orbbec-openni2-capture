#include "UsbDevice.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>

static constexpr uint16_t ASTRA_VID = 0x2bc5;
static constexpr uint16_t ASTRA_PID = 0x0403;  // PrimeSense depth interface (Astra Pro has dual PID: 0501=UVC, 0403=depth)
static constexpr int CTRL_TIMEOUT_MS = 5000;
static constexpr int BULK_TIMEOUT_MS = 1000;
static constexpr int BULK_BUF_SIZE = 64 * 1024;  // 64KB bulk read buffer

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

UsbDevice::UsbDevice() = default;

UsbDevice::~UsbDevice()
{
    close();
}

// ---------------------------------------------------------------------------
// Device enumeration
// ---------------------------------------------------------------------------

std::vector<UsbDeviceInfo> UsbDevice::enumerate()
{
    std::vector<UsbDeviceInfo> result;

    libusb_context* enumCtx = nullptr;
    int rc = libusb_init(&enumCtx);
    if (rc != 0) {
        fprintf(stderr, "UsbDevice::enumerate: libusb_init failed (%d)\n", rc);
        return result;
    }

    libusb_device** list = nullptr;
    ssize_t count = libusb_get_device_list(enumCtx, &list);
    if (count < 0) {
        fprintf(stderr, "UsbDevice::enumerate: libusb_get_device_list failed (%zd)\n", count);
        libusb_exit(enumCtx);
        return result;
    }

    for (ssize_t i = 0; i < count; i++) {
        libusb_device* dev = list[i];
        libusb_device_descriptor desc;
        rc = libusb_get_device_descriptor(dev, &desc);
        if (rc != 0)
            continue;

        if (desc.idVendor == ASTRA_VID && desc.idProduct == ASTRA_PID) {
            UsbDeviceInfo info;
            info.vid = desc.idVendor;
            info.pid = desc.idProduct;
            info.bus = libusb_get_bus_number(dev);
            info.address = libusb_get_device_address(dev);

            // Build URI: "2bc5:0403@bus3,dev10"
            char uriBuf[64];
            snprintf(uriBuf, sizeof(uriBuf), "%04x:%04x@bus%d,dev%d",
                     info.vid, info.pid,
                     static_cast<int>(info.bus),
                     static_cast<int>(info.address));
            info.uri = uriBuf;

            result.push_back(info);
        }
    }

    libusb_free_device_list(list, 1);
    libusb_exit(enumCtx);
    return result;
}

// ---------------------------------------------------------------------------
// Open by URI
// ---------------------------------------------------------------------------

bool UsbDevice::open(const char* uri)
{
    if (!uri || !uri[0])
        return false;

    // Parse URI: expected format "2bc5:0403@busN,devM"
    // We parse out bus and address, then scan for a matching device.
    int busNum = -1, devNum = -1;
    const char* atSign = strchr(uri, '@');
    if (atSign) {
        if (sscanf(atSign + 1, "bus%d,dev%d", &busNum, &devNum) != 2) {
            fprintf(stderr, "UsbDevice::open: cannot parse URI '%s'\n", uri);
            return false;
        }
    }

    int rc = libusb_init(&ctx_);
    if (rc != 0) {
        fprintf(stderr, "UsbDevice::open: libusb_init failed (%d)\n", rc);
        return false;
    }

    // If bus/address specified, enumerate and find exact match
    if (busNum >= 0 && devNum >= 0) {
        libusb_device** list = nullptr;
        ssize_t count = libusb_get_device_list(ctx_, &list);
        bool found = false;
        for (ssize_t i = 0; i < count; i++) {
            libusb_device* dev = list[i];
            libusb_device_descriptor desc;
            rc = libusb_get_device_descriptor(dev, &desc);
            if (rc != 0) continue;
            if (desc.idVendor == ASTRA_VID && desc.idProduct == ASTRA_PID &&
                libusb_get_bus_number(dev) == static_cast<uint8_t>(busNum) &&
                libusb_get_device_address(dev) == static_cast<uint8_t>(devNum)) {
                rc = libusb_open(dev, &handle_);
                if (rc != 0) {
                    fprintf(stderr, "UsbDevice::open: libusb_open failed (%d)\n", rc);
                } else {
                    found = true;
                }
                break;
            }
        }
        libusb_free_device_list(list, 1);
        if (!found) {
            fprintf(stderr, "UsbDevice::open: device at bus%d,dev%d not found\n", busNum, devNum);
            libusb_exit(ctx_);
            ctx_ = nullptr;
            return false;
        }
    } else {
        // Fallback: open first matching VID/PID
        handle_ = libusb_open_device_with_vid_pid(ctx_, ASTRA_VID, ASTRA_PID);
        if (!handle_) {
            fprintf(stderr, "UsbDevice::open: Astra Pro (0x2bc5:0x0403) not found\n");
            libusb_exit(ctx_);
            ctx_ = nullptr;
            return false;
        }
    }

    if (!claimInterface0()) {
        libusb_close(handle_);
        handle_ = nullptr;
        libusb_exit(ctx_);
        ctx_ = nullptr;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Open by VID/PID (first match)
// ---------------------------------------------------------------------------

bool UsbDevice::openByVidPid(uint16_t vid, uint16_t pid)
{
    int rc = libusb_init(&ctx_);
    if (rc != 0) {
        fprintf(stderr, "UsbDevice::openByVidPid: libusb_init failed (%d)\n", rc);
        return false;
    }

    handle_ = libusb_open_device_with_vid_pid(ctx_, vid, pid);
    if (!handle_) {
        fprintf(stderr, "UsbDevice::openByVidPid: device 0x%04x:0x%04x not found\n", vid, pid);
        libusb_exit(ctx_);
        ctx_ = nullptr;
        return false;
    }

    if (!claimInterface0()) {
        libusb_close(handle_);
        handle_ = nullptr;
        libusb_exit(ctx_);
        ctx_ = nullptr;
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Claim interface 0 (detach kernel driver if needed)
// ---------------------------------------------------------------------------

bool UsbDevice::claimInterface0()
{
    if (!handle_)
        return false;

    // Detach kernel driver if active
    int rc = libusb_kernel_driver_active(handle_, 0);
    if (rc == 1) {
        rc = libusb_detach_kernel_driver(handle_, 0);
        if (rc != 0) {
            fprintf(stderr, "UsbDevice: failed to detach kernel driver (%d)\n", rc);
            return false;
        }
        kernelDriverDetached_ = true;
    } else if (rc < 0) {
        fprintf(stderr, "UsbDevice: kernel driver check failed (%d)\n", rc);
        // Not fatal on some platforms; try to claim anyway
    }

    rc = libusb_claim_interface(handle_, 0);
    if (rc != 0) {
        fprintf(stderr, "UsbDevice: failed to claim interface 0 (%d)\n", rc);
        // Reattach kernel driver if we detached it
        if (kernelDriverDetached_) {
            libusb_attach_kernel_driver(handle_, 0);
            kernelDriverDetached_ = false;
        }
        return false;
    }

    interfaceClaimed_ = true;
    return true;
}

// ---------------------------------------------------------------------------
// Close
// ---------------------------------------------------------------------------

void UsbDevice::close()
{
    // Stop all bulk reads first
    {
        std::lock_guard<std::mutex> lock(bulkMutex_);
        for (auto& pair : bulkReads_) {
            pair.second.running.store(false);
        }
    }
    // Join all threads outside the lock
    for (auto& pair : bulkReads_) {
        if (pair.second.thread.joinable()) {
            pair.second.thread.join();
        }
    }
    bulkReads_.clear();

    if (interfaceClaimed_ && handle_) {
        libusb_release_interface(handle_, 0);
        interfaceClaimed_ = false;

        // Reattach kernel driver if we detached it
        if (kernelDriverDetached_) {
            libusb_attach_kernel_driver(handle_, 0);
            kernelDriverDetached_ = false;
        }
    }

    if (handle_) {
        libusb_close(handle_);
        handle_ = nullptr;
    }

    if (ctx_) {
        libusb_exit(ctx_);
        ctx_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// Control transfers
// ---------------------------------------------------------------------------

int UsbDevice::controlWrite(uint8_t request, uint16_t value, uint16_t index,
                            const uint8_t* data, uint16_t length)
{
    if (!handle_)
        return -1;

    // bmRequestType: vendor, host-to-device
    constexpr uint8_t bmRequestType = 0x40;

    int rc = libusb_control_transfer(handle_, bmRequestType, request,
                                      value, index,
                                      const_cast<uint8_t*>(data), length,
                                      CTRL_TIMEOUT_MS);
    if (rc < 0) {
        fprintf(stderr, "UsbDevice::controlWrite: transfer failed (%d)\n", rc);
    }
    return rc;
}

int UsbDevice::controlRead(uint8_t request, uint16_t value, uint16_t index,
                           uint8_t* data, uint16_t length)
{
    if (!handle_)
        return -1;

    // bmRequestType: vendor, device-to-host
    constexpr uint8_t bmRequestType = 0xC0;

    int rc = libusb_control_transfer(handle_, bmRequestType, request,
                                      value, index,
                                      data, length,
                                      CTRL_TIMEOUT_MS);
    if (rc < 0) {
        fprintf(stderr, "UsbDevice::controlRead: transfer failed (%d)\n", rc);
    }
    return rc;
}

// ---------------------------------------------------------------------------
// Bulk read (streaming) — supports concurrent reads on different endpoints
// ---------------------------------------------------------------------------

bool UsbDevice::startBulkRead(int endpoint, BulkCallback callback)
{
    std::lock_guard<std::mutex> lock(bulkMutex_);

    if (bulkReads_.count(endpoint)) {
        if (bulkReads_[endpoint].running.load()) {
            fprintf(stderr, "UsbDevice::startBulkRead: already running on ep=0x%02x\n", endpoint);
            return false;
        }
        // Thread finished but not joined — join it before replacing
        if (bulkReads_[endpoint].thread.joinable()) {
            bulkReads_[endpoint].thread.join();
        }
    }

    if (!handle_ || !callback) {
        return false;
    }

    auto& state = bulkReads_[endpoint];
    state.callback = std::move(callback);
    state.running.store(true);

    state.thread = std::thread(&UsbDevice::bulkReadLoop, this, endpoint);
    return true;
}

void UsbDevice::stopBulkRead(int endpoint)
{
    BulkReadState* state = nullptr;
    {
        std::lock_guard<std::mutex> lock(bulkMutex_);
        auto it = bulkReads_.find(endpoint);
        if (it == bulkReads_.end()) return;
        state = &it->second;
        state->running.store(false);
    }
    // Join outside the lock
    if (state->thread.joinable()) {
        state->thread.join();
    }
}

void UsbDevice::bulkReadLoop(int endpoint)
{
    uint8_t* buf = new uint8_t[BULK_BUF_SIZE];
    int totalReads = 0;
    int attempts = 0;
    int errorCount = 0;
    size_t totalBytes = 0;
    int bulkDiagCount = 0;

    // Diagnostic: dump raw bulk bytes from this endpoint to file
    // Set ASTRA_DUMP_BULK_<HEX>=<path> e.g. ASTRA_DUMP_BULK_81=/tmp/depth.bin
    char envName[32];
    snprintf(envName, sizeof(envName), "ASTRA_DUMP_BULK_%02X", endpoint & 0xFF);
    const char* dumpPath = getenv(envName);
    FILE* dumpFp = nullptr;
    size_t dumpLimit = 1 * 1024 * 1024;  // 1 MB cap
    size_t dumpWritten = 0;
    if (dumpPath) {
        dumpFp = fopen(dumpPath, "wb");
        if (dumpFp) {
            fprintf(stderr, "DIAG bulk ep=0x%02x dumping up to %zu bytes to %s\n",
                    endpoint, dumpLimit, dumpPath);
        }
    }

    auto it = bulkReads_.find(endpoint);
    if (it == bulkReads_.end()) {
        delete[] buf;
        return;
    }
    auto& state = it->second;

    while (state.running.load()) {
        int transferred = 0;
        int rc = libusb_bulk_transfer(handle_, static_cast<unsigned char>(endpoint),
                                       buf, BULK_BUF_SIZE, &transferred,
                                       BULK_TIMEOUT_MS);
        attempts++;

        if (rc == 0 && transferred > 0) {
            if (dumpFp && dumpWritten < dumpLimit) {
                size_t toWrite = std::min((size_t)transferred, dumpLimit - dumpWritten);
                fwrite(buf, 1, toWrite, dumpFp);
                dumpWritten += toWrite;
                if (dumpWritten >= dumpLimit) {
                    fflush(dumpFp);
                    fclose(dumpFp);
                    dumpFp = nullptr;
                    fprintf(stderr, "DIAG bulk ep=0x%02x dump complete (%zu bytes)\n",
                            endpoint, dumpWritten);
                }
            }
            state.callback(buf, transferred);
            totalReads++;
            totalBytes += transferred;
        } else if (rc == LIBUSB_ERROR_TIMEOUT) {
            if (transferred > 0) {
                state.callback(buf, transferred);
                totalReads++;
                totalBytes += transferred;
            }
            continue;
        } else if (rc < 0) {
            errorCount++;
            if (errorCount <= 3 || rc == LIBUSB_ERROR_NO_DEVICE) {
                fprintf(stderr, "UsbDevice::bulkReadLoop: error %d on ep=0x%02x\n",
                        rc, endpoint);
            }
            if (rc == LIBUSB_ERROR_NO_DEVICE || rc == LIBUSB_ERROR_IO) {
                break;
            }
        }
    }
    // Bulk loop done (quiet — remove DIAG spam for production)
    delete[] buf;
}

// ---------------------------------------------------------------------------
// Query
// ---------------------------------------------------------------------------

bool UsbDevice::isOpen() const
{
    return handle_ != nullptr;
}

libusb_device_handle* UsbDevice::handle() const
{
    return handle_;
}
