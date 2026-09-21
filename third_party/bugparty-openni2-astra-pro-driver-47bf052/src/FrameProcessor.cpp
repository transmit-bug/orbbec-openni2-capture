#include "FrameProcessor.h"
#include <cstring>
#include <cstdio>

FrameProcessor::FrameProcessor(uint16_t sofType, uint16_t eofType)
    : sofType_(sofType)
    , eofType_(eofType)
    , lastSofPacketId_(0)
{
}

FrameProcessor::~FrameProcessor()
{
}

void FrameProcessor::setFrameReadyCallback(FrameReadyCallback cb)
{
    frameReadyCb_ = std::move(cb);
}

void FrameProcessor::processPacket(const PacketHeader& header,
                                    const uint8_t* data,
                                    uint32_t dataOffset,
                                    uint32_t dataSize)
{
    // Per-frame packet accounting
    framePacketCount_++;
    frameDataBytes_ += dataSize;

    // 1. SOF handling
    if (header.type == sofType_ && dataOffset == 0) {
        // Double-SOF detection: if packetID is sequential, it's a repeat SOF
        if (header.packetID == lastSofPacketId_ + 1) {
            return;  // Ignore duplicate SOF
        }

        // New frame starting. If we have data from a previous incomplete frame,
        // deliver it first.
        if (bytesWritten_ > 0) {
            onEndOfFrame(header);
        }

        lastSofPacketId_ = header.packetID;
        onStartOfFrame(header);
    }

    // 2. Process data regardless of corruption status
    processFramePacketChunk(header, data, dataOffset, dataSize);

    // 3. Explicit EOF: end frame
    if (header.type == eofType_ && (dataOffset + dataSize) >= header.bufSize - sizeof(PacketHeader)) {
        onEndOfFrame(header);
    }
}

bool FrameProcessor::getLatestFrame(uint8_t** data, int* size)
{
    if (!newFrameReady_.exchange(false)) {
        // No new frame since last call
        return false;
    }

    uint8_t* outData;
    int outSize;

    {
        std::lock_guard<std::mutex> lock(bufferMutex_);

        // Swap: stable gets what was ready, ready gets what was write
        int oldStable = stableIdx_;
        stableIdx_ = readyIdx_;
        readyIdx_ = writeIdx_;
        writeIdx_ = oldStable;

        outData = buffers_[stableIdx_].data.data();
        outSize = buffers_[stableIdx_].size;
    }
    // Mutex released — postProcess runs on stable buffer without blocking USB

    *data = outData;
    *size = outSize;

    // Post-process outside lock: safe because the triple-buffer needs two full USB
    // frames (~66ms at 30fps) before this slot cycles back to writeIdx_. SoftFilter
    // takes ~5-10ms so the race window is effectively unreachable under normal load.
    if (outSize > 0) {
        postProcessFrame(outData, outSize);
    }

    return outSize > 0;
}

void FrameProcessor::setResolution(int width, int height)
{
    width_ = width;
    height_ = height;
}

void FrameProcessor::onStartOfFrame(const PacketHeader& /*header*/)
{
    frameCorrupted_ = false;
    bytesWritten_ = 0;
    framePacketCount_ = 0;
    frameDataBytes_ = 0;
}

void FrameProcessor::onEndOfFrame(const PacketHeader& header)
{
    // Deliver all frames — even "corrupted" ones contain mostly valid data.
    // Missing pixels are zeroed by memset; SoftFilter is skipped for short frames.
    if (bytesWritten_ > 0) {
        std::lock_guard<std::mutex> lock(bufferMutex_);

        FrameBuffer& wb = buffers_[writeIdx_];
        if (wb.data.size() < static_cast<size_t>(bytesWritten_)) {
            wb.data.resize(bytesWritten_);
        }
        memcpy(wb.data.data(), writeBuffer_.data(), bytesWritten_);
        wb.size = bytesWritten_;
        wb.timestamp = header.timestamp;
        wb.frameId = ++frameCounter_;

        // Rotate: write becomes ready, ready becomes stable, stable becomes write
        int oldWrite = writeIdx_;
        int oldReady = readyIdx_;
        int oldStable = stableIdx_;

        readyIdx_ = oldWrite;
        stableIdx_ = oldReady;
        writeIdx_ = oldStable;

        // Signal that a new frame is ready
        newFrameReady_.store(true);
        if (frameReadyCb_) {
            frameReadyCb_();
        }
    }

    // Reset write state for next frame
    bytesWritten_ = 0;
    frameCorrupted_ = false;
}

void FrameProcessor::markCorrupted()
{
    if (!frameCorrupted_) {
        frameCorrupted_ = true;
    }
}
