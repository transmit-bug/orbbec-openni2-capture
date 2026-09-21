#include "AstraDepthStream.h"
#include "AstraDevice.h"
#include "FirmwareCmd.h"
#include "DepthProcessor.h"

#include <cstdio>
#include <cstring>
#include <chrono>

AstraDepthStream::AstraDepthStream(AstraDevice* device)
    : m_device(device)
{
    // Default video mode: 640x480 @ 30fps, DEPTH_1_MM
    m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
    m_videoMode.resolutionX = 640;
    m_videoMode.resolutionY = 480;
    m_videoMode.fps = 30;
}

AstraDepthStream::~AstraDepthStream()
{
    stop();
}

OniStatus AstraDepthStream::start()
{
    if (m_running.load()) {
        return ONI_STATUS_OK;
    }

    // PrimeSense stream start order:
    // 1. ConfigureFirmware (format, resolution, fps, CMOS blanking)
    // 2. SetStream1Mode = DEPTH (firmware param) — triggers data flow
    // 3. Start USB read thread
    //
    // The firmware must receive format/resolution/FPS BEFORE the stream
    // mode is activated, otherwise it may use default settings.

    if (!m_device->configureDepthStream(m_videoMode.resolutionX,
                                         m_videoMode.resolutionY,
                                         m_videoMode.fps)) {
        fprintf(stderr, "AstraDepthStream: configureDepthStream failed\n");
        return ONI_STATUS_ERROR;
    }

    // Enable depth stream mode in firmware (XN_VIDEO_STREAM_DEPTH = 2)
    if (!m_device->firmwareCmd()->setStream1Mode(2)) {
        fprintf(stderr, "AstraDepthStream: setStream1Mode(DEPTH) failed\n");
        return ONI_STATUS_ERROR;
    }

    // Enable laser for depth sensing
    if (!m_device->firmwareCmd()->setLaser(true)) {
        fprintf(stderr, "AstraDepthStream: setLaser(true) failed\n");
        // Not fatal - firmware may auto-enable laser with depth stream
    }

    // Start USB bulk read for depth endpoint
    if (!m_device->startDepthBulkRead()) {
        fprintf(stderr, "AstraDepthStream: startDepthBulkRead failed\n");
        m_device->firmwareCmd()->setStream1Mode(0);
        return ONI_STATUS_ERROR;
    }

    m_device->setDepthStreaming(true);
    m_running.store(true);

    // Start the frame delivery thread
    m_frameThread = std::thread(&AstraDepthStream::frameDeliveryLoop, this);

    return ONI_STATUS_OK;
}

void AstraDepthStream::stop()
{
    if (!m_running.load()) {
        return;
    }

    m_running.store(false);

    // Wake up the frame thread so it can exit
    {
        std::lock_guard<std::mutex> lk(m_frameMutex);
        m_frameReady = true;
    }
    m_frameCv.notify_one();

    if (m_frameThread.joinable()) {
        m_frameThread.join();
    }

    m_device->stopDepthBulkRead();
    m_device->setDepthStreaming(false);

    // Turn off laser when depth stream stops
    m_device->firmwareCmd()->setLaser(false);
}

void AstraDepthStream::frameDeliveryLoop()
{
    DepthProcessor* proc = m_device->depthProcessor();

    while (m_running.load()) {
        // Poll the processor for a new complete frame
        uint8_t* frameData = nullptr;
        int frameSize = 0;

        if (proc->getLatestFrame(&frameData, &frameSize)) {
            // New frame available — acquire an OniFrame and deliver it
            OniFrame* pFrame = getServices().acquireFrame();
            if (pFrame) {
                int requiredSize = getRequiredFrameSize();
                if (frameSize > 0 && frameData) {
                    int copySize = (frameSize < pFrame->dataSize) ? frameSize : pFrame->dataSize;
                    if (copySize > requiredSize) {
                        copySize = requiredSize;
                    }
                    memcpy(pFrame->data, frameData, copySize);
                }

                pFrame->frameIndex = m_frameIndex++;
                pFrame->timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                pFrame->videoMode = m_videoMode;
                pFrame->width = m_videoMode.resolutionX;
                pFrame->height = m_videoMode.resolutionY;
                pFrame->stride = m_videoMode.resolutionX * sizeof(uint16_t);
                pFrame->sensorType = ONI_SENSOR_DEPTH;
                pFrame->croppingEnabled = FALSE;

                raiseNewFrame(pFrame);
            }
        } else {
            // No new frame yet — sleep briefly to avoid busy-spin
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void AstraDepthStream::onNewFrameReady()
{
    std::lock_guard<std::mutex> lk(m_frameMutex);
    m_frameReady = true;
    m_frameCv.notify_one();
}

OniStatus AstraDepthStream::setProperty(int propertyId, const void* data, int dataSize)
{
    switch (propertyId) {
    case ONI_STREAM_PROPERTY_VIDEO_MODE:
        if (dataSize != sizeof(OniVideoMode)) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_running.load()) {
            // Can't change video mode while streaming
            return ONI_STATUS_OUT_OF_FLOW;
        }
        m_videoMode = *static_cast<const OniVideoMode*>(data);
        return ONI_STATUS_OK;

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniStatus AstraDepthStream::getProperty(int propertyId, void* data, int* pDataSize)
{
    switch (propertyId) {
    case ONI_STREAM_PROPERTY_VIDEO_MODE:
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(OniVideoMode))) {
            *pDataSize = sizeof(OniVideoMode);
            return ONI_STATUS_BAD_PARAMETER;
        }
        *static_cast<OniVideoMode*>(data) = m_videoMode;
        *pDataSize = sizeof(OniVideoMode);
        return ONI_STATUS_OK;

    case ONI_STREAM_PROPERTY_MAX_VALUE:
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        *static_cast<int*>(data) = 10000;  // 10 meters max
        *pDataSize = sizeof(int);
        return ONI_STATUS_OK;

    case ONI_STREAM_PROPERTY_MIN_VALUE:
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(int))) {
            *pDataSize = sizeof(int);
            return ONI_STATUS_BAD_PARAMETER;
        }
        *static_cast<int*>(data) = 0;
        *pDataSize = sizeof(int);
        return ONI_STATUS_OK;

    case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(float))) {
            *pDataSize = sizeof(float);
            return ONI_STATUS_BAD_PARAMETER;
        }
        *static_cast<float*>(data) = 58.0f;  // Astra Pro horizontal FOV ~58 degrees
        *pDataSize = sizeof(float);
        return ONI_STATUS_OK;

    case ONI_STREAM_PROPERTY_VERTICAL_FOV:
        if (!pDataSize || !data) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (*pDataSize < static_cast<int>(sizeof(float))) {
            *pDataSize = sizeof(float);
            return ONI_STATUS_BAD_PARAMETER;
        }
        *static_cast<float*>(data) = 45.0f;  // Astra Pro vertical FOV ~45 degrees
        *pDataSize = sizeof(float);
        return ONI_STATUS_OK;

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniBool AstraDepthStream::isPropertySupported(int propertyId)
{
    switch (propertyId) {
    case ONI_STREAM_PROPERTY_VIDEO_MODE:
    case ONI_STREAM_PROPERTY_MAX_VALUE:
    case ONI_STREAM_PROPERTY_MIN_VALUE:
    case ONI_STREAM_PROPERTY_HORIZONTAL_FOV:
    case ONI_STREAM_PROPERTY_VERTICAL_FOV:
        return TRUE;
    default:
        return FALSE;
    }
}

int AstraDepthStream::getRequiredFrameSize()
{
    return m_videoMode.resolutionX * m_videoMode.resolutionY * sizeof(uint16_t);
}
