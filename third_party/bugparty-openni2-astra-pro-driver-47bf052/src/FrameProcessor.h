#pragma once

#include "PacketParser.h"
#include <cstdint>
#include <cstddef>
#include <vector>
#include <mutex>
#include <atomic>
#include <functional>

class FrameProcessor {
public:
    FrameProcessor(uint16_t sofType, uint16_t eofType);
    virtual ~FrameProcessor();

    // Called by PacketParser callback for each packet
    void processPacket(const PacketHeader& header,
                       const uint8_t* data, uint32_t dataOffset, uint32_t dataSize);

    // Get the latest complete frame (triple buffer).
    // Returns true if a new frame is available since last call.
    // data/size point into internal buffer (valid until next getLatestFrame).
    bool getLatestFrame(uint8_t** data, int* size);

    // Post-process frame on consumer thread (e.g., SoftFilter).
    // Called by getLatestFrame before returning. Default is no-op.
    virtual void postProcessFrame(uint8_t* data, int size) {}

    // Set frame dimensions
    void setResolution(int width, int height);
    int width() const { return width_; }
    int height() const { return height_; }

    // Check if current frame is corrupted
    bool isFrameCorrupted() const { return frameCorrupted_; }

    // Set callback invoked when a new frame is ready (called from onEndOfFrame)
    using FrameReadyCallback = std::function<void()>;
    void setFrameReadyCallback(FrameReadyCallback cb);

protected:
    // Subclass implements this to process data chunks within a frame
    virtual void processFramePacketChunk(const PacketHeader& header,
                                          const uint8_t* data,
                                          uint32_t dataOffset,
                                          uint32_t dataSize) = 0;

    // Called on SOF - subclass can override
    virtual void onStartOfFrame(const PacketHeader& header);

    // Called on EOF - subclasses must call this at end of their override
    virtual void onEndOfFrame(const PacketHeader& header);

    // Write buffer access for subclasses
    std::vector<uint8_t> writeBuffer_;
    int bytesWritten_ = 0;

    // Mark frame as corrupted
    void markCorrupted();
    bool frameCorrupted_ = false;

private:
    uint16_t sofType_;
    uint16_t eofType_;
    int width_ = 0;
    int height_ = 0;

    // Triple buffer: write, ready, stable
    struct FrameBuffer {
        std::vector<uint8_t> data;
        int size = 0;
        uint64_t timestamp = 0;
        uint32_t frameId = 0;
    };
    FrameBuffer buffers_[3];
    int writeIdx_ = 0;
    int readyIdx_ = 1;
    int stableIdx_ = 2;
    std::mutex bufferMutex_;
    uint32_t frameCounter_ = 0;

    // Last SOF packet ID for double-SOF detection
    uint16_t lastSofPacketId_ = 0;

    // Per-frame packet counter (diagnostic)
    int framePacketCount_ = 0;
    int frameDataBytes_ = 0;

    // Frame ready callback
    FrameReadyCallback frameReadyCb_;

    // Flag: true when a new frame was written since last getLatestFrame()
    std::atomic<bool> newFrameReady_{false};
};
