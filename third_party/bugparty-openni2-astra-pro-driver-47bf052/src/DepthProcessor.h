#pragma once

#include "FrameProcessor.h"
#include "PacketParser.h"
#include <vector>

// Depth frame processor with 3 decode formats and ShiftToDepth LUT.
//
// Supported depth input formats (set via setDepthFormat):
//   0 = PSCompressed  (4-bit nibble RLE/diff, default for Astra Pro)
//   1 = Packed11      (11-bit packed, 11 bytes -> 8 pixels)
//   2 = Uncompressed16 (raw uint16 shift values)
//
// All decoded shift values pass through a ShiftToDepth LUT before
// being written to the output buffer as uint16 depth values (mm).
//
// Post-processing: SoftFilter speckle removal (optional, enabled by default)

class DepthProcessor : public FrameProcessor {
public:
    DepthProcessor();
    ~DepthProcessor() override;

    // Set the ShiftToDepth lookup table.
    // table: array of maxShiftValue_ entries mapping shift -> depth(mm).
    // size:  number of entries in the table.
    void setShiftToDepthTable(const uint16_t* table, int size);

    // Set depth input format: 0=PSCompressed, 1=Packed11, 2=Uncompressed16
    void setDepthFormat(int format);

    // Set firmware version for SOF padding logic (e.g. 0x0501 for v5.1)
    void setFirmwareVersion(uint16_t version);

    // Enable/disable SoftFilter speckle removal (default: true)
    void setSoftFilterEnabled(bool enabled);

    // Post-process: run SoftFilter on consumer thread
    void postProcessFrame(uint8_t* data, int size) override;

protected:
    void processFramePacketChunk(const PacketHeader& header,
                                  const uint8_t* data,
                                  uint32_t dataOffset,
                                  uint32_t dataSize) override;
    void onStartOfFrame(const PacketHeader& header) override;
    void onEndOfFrame(const PacketHeader& header) override;

private:
    // --- ShiftToDepth LUT ---
    std::vector<uint16_t> shiftToDepthTable_;
    int maxShiftValue_ = 2048;  // XN_DEVICE_SENSOR_MAX_SHIFT_VALUE

    // Apply ShiftToDepth conversion to a single pixel.
    // Values >= maxShiftValue_ are mapped to NO_DEPTH_VALUE (0).
    uint16_t applyShiftToDepth(uint16_t shiftValue);

    // Depth format: 0=PSCompressed, 1=Packed11, 2=Uncompressed16
    int depthFormat_ = 0;  // PSCompressed

    // Firmware version for SOF padding logic
    uint16_t fwVersion_ = 0;

    // --- SOF padding (FW >= 5.1) ---
    uint32_t paddingPixelsOnStart_ = 0;
    uint32_t paddingPixelsOnEnd_ = 0;
    void padPixels(uint32_t count);

    // --- Packed11 cross-packet buffer ---
    // Leftover bytes (< 11) from previous packet, prepended on next call.
    static const int PACKED11_INPUT_SIZE = 11;   // 11 bytes per element
    static const int PACKED11_OUTPUT_SIZE = 8;    // 8 pixels per element
    std::vector<uint8_t> packed11Buffer_;
    int packed11BufferSize_ = 0;
    bool packed11FrameSkipped_ = false;

    // PSCompressed: accumulate all frame data, decode at EOF.
    // Simpler and more robust than PrimeSense's cross-chunk self-synchronization
    // (which truncates to "possible stop" points and re-decodes on next chunk).
    // Since frames are only delivered at EOF, buffering adds no latency.
    std::vector<uint8_t> psFrameBuf_;

    // --- Format-specific decoders ---
    void processPacked11(const uint8_t* data, uint32_t size);
    void processUncompressed16(const uint8_t* data, uint32_t size);
    void processPSCompressed(const uint8_t* data, uint32_t size, bool isLastChunk);

    // Packed11: 11 bytes -> 8 pixels (each through ShiftToDepth)
    void unpack11to16(const uint8_t* input, int inputSize);

    // PSCompressed: 4-bit nibble RLE/diff decompression
    // Called once at EOF with the full frame buffer.
    void uncompressDepthPS(const uint8_t* input, uint32_t inputSize, bool isLastPart,
                           uint32_t& outWrittenOutput, uint32_t& outActualRead);

    // Calculate expected frame size in bytes (width * height * 2)
    uint32_t calculateExpectedSize();

    // Write a single uint16 pixel to the write buffer
    void writePixel(uint16_t value);

    // No-depth sentinel value
    static const uint16_t NO_DEPTH_VALUE = 0;

    // SoftFilter speckle removal
    bool softFilterEnabled_ = true;
};
