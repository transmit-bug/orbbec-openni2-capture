#include "IrProcessor.h"
#include <cstring>
#include <algorithm>

IrProcessor::IrProcessor()
    : FrameProcessor(PacketType::IMAGE_SOF, PacketType::IMAGE_EOF)
{
    // Pre-allocate continuous buffer (never more than INPUT_ELEMENT_SIZE leftover bytes)
    contBuffer_.resize(INPUT_ELEMENT_SIZE);
    contBufferSize_ = 0;

    // Default resolution: VGA 640x480
    setResolution(640, 480);

    // Pre-allocate write buffer for a full VGA Gray16 frame
    // Non-SXGA modes have 8 extra rows of calibration data
    uint32_t expectedSize = calculateExpectedSize();
    writeBuffer_.resize(expectedSize, 0);
}

bool IrProcessor::unpack10to16(const uint8_t* input, uint32_t inputSize,
                                uint16_t* output, uint32_t* actualRead,
                                uint32_t* outputSize)
{
    uint32_t nElements = inputSize / INPUT_ELEMENT_SIZE; // floored
    uint32_t nNeededOutput = nElements * OUTPUT_ELEMENT_SIZE;

    *actualRead = 0;

    if (*outputSize < nNeededOutput) {
        *outputSize = 0;
        return false;
    }

    // Convert 10-bit packed data into 16-bit shorts
    // Input: [B0][B1][B2][B3][B4] (5 bytes) -> 4 pixels
    // P0 = (B0 << 2) | (B1[7:6])
    // P1 = (B1[5:0] << 4) | (B2[7:4])
    // P2 = (B2[3:0] << 6) | (B3[7:2])
    // P3 = (B3[1:0] << 8) | B4
    for (uint32_t nElem = 0; nElem < nElements; ++nElem) {
        int32_t cInput;

        // P0
        cInput = *input;
        *output = (cInput & 0xFF) << 2;

        input++;
        cInput = *input;
        *output |= (cInput & 0xC0) >> 6;
        output++;

        // P1
        *output = (cInput & 0x3F) << 4;

        input++;
        cInput = *input;
        *output |= (cInput & 0xF0) >> 4;
        output++;

        // P2
        *output = (cInput & 0x0F) << 6;

        input++;
        cInput = *input;
        *output |= (cInput & 0xFC) >> 2;
        output++;

        // P3
        *output = (cInput & 0x03) << 8;

        input++;
        cInput = *input;
        *output |= (cInput & 0xFF);
        output++;

        input++;
    }

    *actualRead = nElements * INPUT_ELEMENT_SIZE;
    *outputSize = nNeededOutput;
    return true;
}

uint32_t IrProcessor::calculateExpectedSize() const
{
    int xRes = width();
    int yRes = height();

    // Non-SXGA modes have 8 extra rows of calibration data
    if (xRes != 1280) {
        yRes += 8;
    }

    return static_cast<uint32_t>(xRes) * yRes * 2; // 2 bytes per pixel (Gray16)
}

void IrProcessor::onStartOfFrame(const PacketHeader& header)
{
    FrameProcessor::onStartOfFrame(header);

    // Reset continuous buffer
    contBufferSize_ = 0;

    // Reset padding
    startPadding_ = 0;
    endPadding_ = 0;

    // Ensure write buffer is large enough
    uint32_t expectedSize = calculateExpectedSize();
    if (writeBuffer_.size() < expectedSize) {
        writeBuffer_.resize(expectedSize, 0);
    }

    // FW >= 5.1: timestamp field encodes padding pixels
    // High 16 bits = start padding, low 16 bits = end padding
    applyPaddingPixels(header);
}

void IrProcessor::applyPaddingPixels(const PacketHeader& header)
{
    // The timestamp field in the SOF packet may encode padding pixels
    // for firmware version >= 5.1.
    // High 16 bits = start padding pixels, low 16 bits = end padding pixels
    uint32_t ts = header.timestamp;
    startPadding_ = static_cast<int>((ts >> 16) & 0xFFFF);
    endPadding_ = static_cast<int>(ts & 0xFFFF);

    // Write start padding pixels (value 0 = no IR data)
    if (startPadding_ > 0) {
        int paddingBytes = startPadding_ * 2; // 2 bytes per Gray16 pixel
        int maxBytes = static_cast<int>(writeBuffer_.size()) - bytesWritten_;
        if (paddingBytes > maxBytes) {
            paddingBytes = maxBytes;
        }
        memset(writeBuffer_.data() + bytesWritten_, 0, paddingBytes);
        bytesWritten_ += paddingBytes;
    }
}

void IrProcessor::processFramePacketChunk(const PacketHeader& /*header*/,
                                          const uint8_t* data,
                                          uint32_t /*dataOffset*/,
                                          uint32_t dataSize)
{
    // IR data is 10-bit packed: 5 bytes -> 4 pixels (Gray16)
    // Must handle cross-packet boundaries with continuous buffer

    // If there are leftover bytes from the previous packet, fill up to a
    // complete 5-byte group before unpacking
    if (contBufferSize_ != 0) {
        // Fill continuous buffer to a whole element
        uint32_t nReadBytes = std::min(dataSize,
            static_cast<uint32_t>(INPUT_ELEMENT_SIZE - contBufferSize_));
        memcpy(contBuffer_.data() + contBufferSize_, data, nReadBytes);
        contBufferSize_ += nReadBytes;
        data += nReadBytes;
        dataSize -= nReadBytes;

        if (contBufferSize_ == INPUT_ELEMENT_SIZE) {
            // Process the complete element
            uint32_t nActualRead = 0;
            uint32_t remaining = static_cast<uint32_t>(writeBuffer_.size()) - bytesWritten_;
            uint32_t nOutputSize = remaining;
            if (!unpack10to16(contBuffer_.data(), INPUT_ELEMENT_SIZE,
                             reinterpret_cast<uint16_t*>(writeBuffer_.data() + bytesWritten_),
                             &nActualRead, &nOutputSize)) {
                markCorrupted();
            } else {
                bytesWritten_ += nOutputSize;
            }
            contBufferSize_ = 0;
        }
    }

    // Process full 5-byte groups from current data
    if (dataSize > 0 && !frameCorrupted_) {
        uint32_t nActualRead = 0;
        uint32_t remaining = static_cast<uint32_t>(writeBuffer_.size()) - bytesWritten_;
        uint32_t nOutputSize = remaining;

        if (!unpack10to16(data, dataSize,
                          reinterpret_cast<uint16_t*>(writeBuffer_.data() + bytesWritten_),
                          &nActualRead, &nOutputSize)) {
            markCorrupted();
        } else {
            bytesWritten_ += nOutputSize;

            data += nActualRead;
            dataSize -= nActualRead;

            // Store leftover bytes (less than INPUT_ELEMENT_SIZE) for next packet
            if (dataSize > 0) {
                memcpy(contBuffer_.data(), data, dataSize);
                contBufferSize_ = static_cast<int>(dataSize);
            }
        }
    }
}
