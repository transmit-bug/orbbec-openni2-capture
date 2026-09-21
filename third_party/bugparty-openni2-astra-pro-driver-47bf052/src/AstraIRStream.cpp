#include "AstraIRStream.h"
#include "AstraDevice.h"
#include "FirmwareCmd.h"
#include "IrProcessor.h"

#include <cstdio>
#include <cstring>
#include <chrono>

AstraIRStream::AstraIRStream(AstraDevice* device)
    : m_device(device)
{
    // Default video mode: 640x480 @ 30fps, GRAY16
    m_videoMode.pixelFormat = ONI_PIXEL_FORMAT_GRAY16;
    m_videoMode.resolutionX = 640;
    m_videoMode.resolutionY = 480;
    m_videoMode.fps = 30;
}

AstraIRStream::~AstraIRStream()
{
    stop();
}

OniStatus AstraIRStream::start()
{
    if (m_running.load()) {
        return ONI_STATUS_OK;
    }

    // PrimeSense stream start order:
    // 1. SetStream0Mode = IR (firmware param)
    // 2. Start USB read thread
    // 3. ConfigureFirmware (resolution, fps, CMOS blanking)

    // Enable IR stream mode in firmware (XN_VIDEO_STREAM_IR = 3)
    if (!m_device->firmwareCmd()->setStream0Mode(3)) {
        fprintf(stderr, "AstraIRStream: setStream0Mode(IR) failed\n");
        return ONI_STATUS_ERROR;
    }

    // Start USB bulk read for IR endpoint
    if (!m_device->startIRBulkRead()) {
        fprintf(stderr, "AstraIRStream: startIRBulkRead failed\n");
        m_device->firmwareCmd()->setStream0Mode(0);
        return ONI_STATUS_ERROR;
    }

    // Configure resolution, fps, and query blanking
    if (!m_device->configureIRStream(m_videoMode.resolutionX,
                                      m_videoMode.resolutionY,
                                      m_videoMode.fps)) {
        fprintf(stderr, "AstraIRStream: configureIRStream failed\n");
        m_device->stopIRBulkRead();
        return ONI_STATUS_ERROR;
    }

    m_device->setIRStreaming(true);
    m_running.store(true);

    // Start the frame delivery thread
    m_frameThread = std::thread(&AstraIRStream::frameDeliveryLoop, this);

    return ONI_STATUS_OK;
}

void AstraIRStream::stop()
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

    m_device->stopIRBulkRead();
    m_device->setIRStreaming(false);
}

void AstraIRStream::frameDeliveryLoop()
{
    IrProcessor* proc = m_device->irProcessor();

    while (m_running.load()) {
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
                pFrame->sensorType = ONI_SENSOR_IR;
                pFrame->croppingEnabled = FALSE;

                // Stride depends on pixel format
                if (m_videoMode.pixelFormat == ONI_PIXEL_FORMAT_GRAY16) {
                    pFrame->stride = m_videoMode.resolutionX * sizeof(uint16_t);
                } else {
                    pFrame->stride = m_videoMode.resolutionX * sizeof(uint8_t);
                }

                raiseNewFrame(pFrame);
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void AstraIRStream::onNewFrameReady()
{
    std::lock_guard<std::mutex> lk(m_frameMutex);
    m_frameReady = true;
    m_frameCv.notify_one();
}

OniStatus AstraIRStream::setProperty(int propertyId, const void* data, int dataSize)
{
    switch (propertyId) {
    case ONI_STREAM_PROPERTY_VIDEO_MODE:
        if (dataSize != sizeof(OniVideoMode)) {
            return ONI_STATUS_BAD_PARAMETER;
        }
        if (m_running.load()) {
            return ONI_STATUS_OUT_OF_FLOW;
        }
        m_videoMode = *static_cast<const OniVideoMode*>(data);
        return ONI_STATUS_OK;

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniStatus AstraIRStream::getProperty(int propertyId, void* data, int* pDataSize)
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
        *static_cast<int*>(data) = 1023;  // 10-bit IR max
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
        *static_cast<float*>(data) = 58.0f;
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
        *static_cast<float*>(data) = 45.0f;
        *pDataSize = sizeof(float);
        return ONI_STATUS_OK;

    default:
        return ONI_STATUS_NOT_IMPLEMENTED;
    }
}

OniBool AstraIRStream::isPropertySupported(int propertyId)
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

int AstraIRStream::getRequiredFrameSize()
{
    int bytesPerPixel = (m_videoMode.pixelFormat == ONI_PIXEL_FORMAT_GRAY16) ? 2 : 1;
    return m_videoMode.resolutionX * m_videoMode.resolutionY * bytesPerPixel;
}
