#pragma once

#include "FrameProcessor.h"
#include "PacketParser.h"

class IrProcessor : public FrameProcessor {
public:
    IrProcessor();

protected:
    void processFramePacketChunk(const PacketHeader& header,
                                  const uint8_t* data,
                                  uint32_t dataOffset,
                                  uint32_t dataSize) override;

    void onStartOfFrame(const PacketHeader& header) override;

private:
    // 10-bit to 16-bit unpacking.
    // input:  packed 10-bit data
    // inputSize: number of bytes in input
    // output:  buffer for unpacked 16-bit pixels
    // actualRead:  number of input bytes consumed (always a multiple of 5)
    // outputSize:  number of bytes written to output
    // Returns true on success, false if output buffer too small.
    bool unpack10to16(const uint8_t* input, uint32_t inputSize,
                       uint16_t* output, uint32_t* actualRead, uint32_t* outputSize);

    // Calculate expected frame size in bytes (Gray16)
    uint32_t calculateExpectedSize() const;

    // Handle padding pixels from SOF header (FW >= 5.1)
    // High 16 bits of timestamp = start padding, low 16 bits = end padding
    void applyPaddingPixels(const PacketHeader& header);

    // Continuous buffer for cross-packet data (5-byte alignment)
    std::vector<uint8_t> contBuffer_;
    int contBufferSize_ = 0;

    // Padding pixels from SOF
    int startPadding_ = 0;
    int endPadding_ = 0;

    // Expected frame size
    static const int INPUT_ELEMENT_SIZE = 5;  // 5 bytes per group
    static const int OUTPUT_ELEMENT_SIZE = 8; // 4 x uint16 = 8 bytes per group
};
