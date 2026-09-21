#pragma once

#include <openni2/driver/OniDriverAPI.h>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>

class AstraDevice;

class AstraIRStream : public oni::driver::StreamBase {
public:
    AstraIRStream(AstraDevice* device);
    ~AstraIRStream() override;

    OniStatus start() override;
    void stop() override;

    OniStatus setProperty(int propertyId, const void* data, int dataSize) override;
    OniStatus getProperty(int propertyId, void* data, int* pDataSize) override;
    OniBool isPropertySupported(int propertyId) override;

    int getRequiredFrameSize() override;

    // Called by AstraDevice when a new frame is ready from IrProcessor
    void onNewFrameReady();

private:
    void frameDeliveryLoop();

    AstraDevice* m_device;
    OniVideoMode m_videoMode;
    std::atomic<bool> m_running{false};
    uint32_t m_frameIndex{0};

    // Frame delivery thread
    std::thread m_frameThread;

    // Frame notification
    std::mutex m_frameMutex;
    std::condition_variable m_frameCv;
    bool m_frameReady{false};
};
