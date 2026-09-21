#include "DepthProcessor.h"
#include "SoftFilter.h"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <climits>

// ---------------------------------------------------------------------------
// Bit extraction macro (from PrimeSense XnPacked11DepthProcessor)
// TAKE_BITS(source, count, offset) extracts <count> bits starting at
// bit position <offset> from <source>.
// For example: TAKE_BITS(0xF4, 3, 2) == 0x5
// ---------------------------------------------------------------------------
#define TAKE_BITS(source, count, offset) \
    (((source) & (((1 << (count)) - 1) << (offset))) >> (offset))

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

DepthProcessor::DepthProcessor()
    : FrameProcessor(PacketType::DEPTH_SOF, PacketType::DEPTH_EOF)
{
    setResolution(640, 480);
    uint32_t frameSize = static_cast<uint32_t>(640) * 480 * 2;
    writeBuffer_.resize(frameSize, 0);

    packed11Buffer_.resize(PACKED11_INPUT_SIZE);
    packed11BufferSize_ = 0;
}

DepthProcessor::~DepthProcessor()
{
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void DepthProcessor::setShiftToDepthTable(const uint16_t* table, int size)
{
    if (table && size > 0) {
        shiftToDepthTable_.assign(table, table + size);
        maxShiftValue_ = size;
    } else {
        // Identity LUT: shift == depth
        shiftToDepthTable_.resize(maxShiftValue_);
        for (int i = 0; i < maxShiftValue_; ++i) {
            shiftToDepthTable_[i] = static_cast<uint16_t>(i);
        }
    }
}

void DepthProcessor::setDepthFormat(int format)
{
    depthFormat_ = format;
}

void DepthProcessor::setFirmwareVersion(uint16_t version)
{
    fwVersion_ = version;
}

void DepthProcessor::setSoftFilterEnabled(bool enabled)
{
    softFilterEnabled_ = enabled;
}

void DepthProcessor::postProcessFrame(uint8_t* data, int size)
{
    int pixels = size / 2;
    if (pixels <= 0) return;
    uint16_t* buf = reinterpret_cast<uint16_t*>(data);

    // SoftFilter runs in shift-space (before S2D) so maxDiff=4 means 4 shift
    // units. This matches the official driver which calls filterSpeckles on the
    // raw Unpack11to16 output before XnShiftToDepthConvert. In depth-space at
    // 2m each shift step is ~12mm, so maxDiff=4mm would strand every shift band
    // as an isolated region and produce concentric-square artifacts.
    // ASTRA_SOFTFILTER=0 disables SoftFilter at runtime for A/B testing.
    static const char* sfEnv = getenv("ASTRA_SOFTFILTER");
    bool sfOn = softFilterEnabled_ && !(sfEnv && sfEnv[0] == '0');
    if (sfOn && pixels == width() * height()) {
        SoftFilter::apply(buf, width(), height(), 0);  // 0 = no shift (no data)
    }

    // Always apply S2D, even on short frames — buffer must contain depths,
    // not shifts, when delivered to the consumer.
    for (int i = 0; i < pixels; i++) {
        buf[i] = applyShiftToDepth(buf[i]);
    }
}

// ---------------------------------------------------------------------------
// ShiftToDepth conversion
// ---------------------------------------------------------------------------

uint16_t DepthProcessor::applyShiftToDepth(uint16_t shiftValue)
{
    if (shiftValue >= static_cast<uint16_t>(maxShiftValue_)) {
        return NO_DEPTH_VALUE;
    }
    if (!shiftToDepthTable_.empty()) {
        return shiftToDepthTable_[shiftValue];
    }
    return shiftValue;
}

// ---------------------------------------------------------------------------
// Pixel write helper
// ---------------------------------------------------------------------------

void DepthProcessor::writePixel(uint16_t value)
{
    if (bytesWritten_ + 2 <= static_cast<int>(writeBuffer_.size())) {
        uint16_t* dst = reinterpret_cast<uint16_t*>(writeBuffer_.data() + bytesWritten_);
        *dst = value;
        bytesWritten_ += 2;
    }
}

// ---------------------------------------------------------------------------
// Padding (FW >= 5.1)
// ---------------------------------------------------------------------------

void DepthProcessor::padPixels(uint32_t count)
{
    uint32_t paddingBytes = count * sizeof(uint16_t);
    uint32_t remaining = static_cast<uint32_t>(writeBuffer_.size()) - bytesWritten_;
    if (paddingBytes > remaining) {
        paddingBytes = remaining;
        count = paddingBytes / sizeof(uint16_t);
    }
    // Fill with NO_DEPTH_VALUE (0)
    uint16_t* dst = reinterpret_cast<uint16_t*>(writeBuffer_.data() + bytesWritten_);
    for (uint32_t i = 0; i < count; ++i) {
        dst[i] = NO_DEPTH_VALUE;
    }
    bytesWritten_ += paddingBytes;
}

// ---------------------------------------------------------------------------
// SOF / EOF
// ---------------------------------------------------------------------------

void DepthProcessor::onStartOfFrame(const PacketHeader& header)
{
    FrameProcessor::onStartOfFrame(header);

    packed11BufferSize_ = 0;
    packed11FrameSkipped_ = false;
    psFrameBuf_.clear();

    paddingPixelsOnStart_ = 0;
    paddingPixelsOnEnd_ = 0;

    // Don't memset writeBuffer_ yet for PSCompressed — we decode at EOF.
    // For Packed11/Uncompressed16, data is written incrementally, so clear now.
    if (depthFormat_ != 0) {
        uint32_t expectedSize = calculateExpectedSize();
        if (writeBuffer_.size() < expectedSize) {
            writeBuffer_.resize(expectedSize, 0);
        }
        memset(writeBuffer_.data(), 0, writeBuffer_.size());
    }

    if (fwVersion_ >= 0x0501 && header.timestamp != 0) {
        // Ensure writeBuffer_ exists for padding
        uint32_t expectedSize = calculateExpectedSize();
        if (writeBuffer_.size() < expectedSize) {
            writeBuffer_.resize(expectedSize, 0);
        }
        paddingPixelsOnStart_ = header.timestamp >> 16;
        paddingPixelsOnEnd_ = header.timestamp & 0xFFFF;
        padPixels(paddingPixelsOnStart_);
    }
}

void DepthProcessor::onEndOfFrame(const PacketHeader& header)
{
    // For PSCompressed, decode the entire frame now (all data buffered in psFrameBuf_)
    if (depthFormat_ == 0 && !psFrameBuf_.empty()) {
        uint32_t expectedSize = calculateExpectedSize();
        if (writeBuffer_.size() < expectedSize) {
            writeBuffer_.resize(expectedSize, 0);
        }
        memset(writeBuffer_.data(), 0, writeBuffer_.size());
        bytesWritten_ = 0;

        // Apply start padding (was already computed in onStartOfFrame)
        if (paddingPixelsOnStart_ != 0) {
            padPixels(paddingPixelsOnStart_);
        }

        uint32_t nWrittenOutput = expectedSize - bytesWritten_;
        uint32_t nActualRead = 0;
        uncompressDepthPS(psFrameBuf_.data(), static_cast<uint32_t>(psFrameBuf_.size()),
                          true, nWrittenOutput, nActualRead);
        bytesWritten_ += nWrittenOutput;
    }

    if (paddingPixelsOnEnd_ != 0) {
        padPixels(paddingPixelsOnEnd_);
        paddingPixelsOnEnd_ = 0;
    }

    uint32_t expectedSize = calculateExpectedSize();
    if (static_cast<uint32_t>(bytesWritten_) != expectedSize) {
        markCorrupted();
    }

    // SoftFilter deferred to consumer thread — don't run on USB callback thread

    packed11BufferSize_ = 0;
    packed11FrameSkipped_ = false;
    psFrameBuf_.clear();
    FrameProcessor::onEndOfFrame(header);
}

// ---------------------------------------------------------------------------
// Main dispatch
// ---------------------------------------------------------------------------

void DepthProcessor::processFramePacketChunk(const PacketHeader& header,
                                              const uint8_t* data,
                                              uint32_t dataOffset,
                                              uint32_t dataSize)
{
    if (dataSize == 0) return;

    bool isLastChunk = (header.type == PacketType::DEPTH_EOF) &&
        ((dataOffset + dataSize) >= (header.bufSize - sizeof(PacketHeader)));

    switch (depthFormat_) {
    case 1:  // Packed11
        processPacked11(data, dataSize);
        break;
    case 2:  // Uncompressed16
        processUncompressed16(data, dataSize);
        break;
    case 0:  // PSCompressed (default)
    default:
        processPSCompressed(data, dataSize, isLastChunk);
        break;
    }
}

// ---------------------------------------------------------------------------
// Uncompressed 16-bit decoder (simplest format)
// ---------------------------------------------------------------------------

void DepthProcessor::processUncompressed16(const uint8_t* data, uint32_t size)
{
    // Handle odd byte alignment (skip one byte to keep uint16 alignment)
    if (size % 2 != 0) {
        size--;
        data++;
    }

    const uint16_t* raw = reinterpret_cast<const uint16_t*>(data);
    uint32_t pixelCount = size / 2;

    // Check available space
    uint32_t needed = pixelCount * sizeof(uint16_t);
    uint32_t remaining = static_cast<uint32_t>(writeBuffer_.size()) - bytesWritten_;
    if (needed > remaining) {
        pixelCount = remaining / sizeof(uint16_t);
    }

    uint16_t* dst = reinterpret_cast<uint16_t*>(writeBuffer_.data() + bytesWritten_);
    for (uint32_t i = 0; i < pixelCount; ++i) {
        uint16_t shift = std::min(raw[i], static_cast<uint16_t>(maxShiftValue_ - 1));
        dst[i] = applyShiftToDepth(shift);
    }
    bytesWritten_ += pixelCount * sizeof(uint16_t);
}

// ---------------------------------------------------------------------------
// Packed 11-bit decoder (11 bytes -> 8 pixels)
// ---------------------------------------------------------------------------

void DepthProcessor::processPacked11(const uint8_t* data, uint32_t size)
{
    // Optional frame-start byte-skip for alignment debugging. Set
    // ASTRA_PACKED11_SKIP=N to drop N bytes at the start of each new frame.
    // Default 0 = no skip.
    static const char* skipEnv = getenv("ASTRA_PACKED11_SKIP");
    int frameSkipBytes = skipEnv ? atoi(skipEnv) : 0;
    if (!packed11FrameSkipped_ && frameSkipBytes > 0) {
        uint32_t toSkip = std::min(static_cast<uint32_t>(frameSkipBytes), size);
        data += toSkip;
        size -= toSkip;
        if (toSkip >= static_cast<uint32_t>(frameSkipBytes)) {
            packed11FrameSkipped_ = true;
        }
    }

    if (packed11BufferSize_ != 0) {
        uint32_t nReadBytes = std::min(size,
            static_cast<uint32_t>(PACKED11_INPUT_SIZE - packed11BufferSize_));
        memcpy(packed11Buffer_.data() + packed11BufferSize_, data, nReadBytes);
        packed11BufferSize_ += nReadBytes;
        data += nReadBytes;
        size -= nReadBytes;

        if (packed11BufferSize_ == PACKED11_INPUT_SIZE) {
            // Decode the complete 11-byte group
            unpack11to16(packed11Buffer_.data(), PACKED11_INPUT_SIZE);
            packed11BufferSize_ = 0;
        }
    }

    // Decode full 11-byte groups from the current data
    if (size > 0 && !frameCorrupted_) {
        uint32_t nElements = size / PACKED11_INPUT_SIZE;  // floored
        if (nElements > 0) {
            unpack11to16(data, nElements * PACKED11_INPUT_SIZE);
            data += nElements * PACKED11_INPUT_SIZE;
            size -= nElements * PACKED11_INPUT_SIZE;
        }

        // Store leftover bytes (< 11) for next packet
        if (size > 0) {
            memcpy(packed11Buffer_.data(), data, size);
            packed11BufferSize_ = static_cast<int>(size);
        }
    }
}

void DepthProcessor::unpack11to16(const uint8_t* input, int inputSize)
{
    uint32_t nElements = inputSize / PACKED11_INPUT_SIZE;
    uint32_t nNeededOutput = nElements * PACKED11_OUTPUT_SIZE * sizeof(uint16_t);

    uint32_t remaining = static_cast<uint32_t>(writeBuffer_.size()) - bytesWritten_;
    if (nNeededOutput > remaining) {
        nElements = remaining / (PACKED11_OUTPUT_SIZE * sizeof(uint16_t));
        nNeededOutput = nElements * PACKED11_OUTPUT_SIZE * sizeof(uint16_t);
    }

    uint16_t* output = reinterpret_cast<uint16_t*>(writeBuffer_.data() + bytesWritten_);

    // Unpack 11-bit packed data into raw shift values (S2D applied later in
    // postProcessFrame, after SoftFilter runs in shift-space).
    // Bit layout per 11-byte group -> 8 pixels:
    //   input:  0,  1,  2,3,  4,  5,  6,7,  8,  9,10
    //   bits:  8,3,5,6,2,8,1,7,4,4,7,1,8,2,6,5,3,8
    //   output: 0,  1,    2,  3,  4,    5,  6,  7
    for (uint32_t nElem = 0; nElem < nElements; ++nElem) {
        output[0] = (TAKE_BITS(input[0], 8, 0) << 3) | TAKE_BITS(input[1], 3, 5);
        output[1] = (TAKE_BITS(input[1], 5, 0) << 6) | TAKE_BITS(input[2], 6, 2);
        output[2] = (TAKE_BITS(input[2], 2, 0) << 9) |
            (TAKE_BITS(input[3], 8, 0) << 1) | TAKE_BITS(input[4], 1, 7);
        output[3] = (TAKE_BITS(input[4], 7, 0) << 4) | TAKE_BITS(input[5], 4, 4);
        output[4] = (TAKE_BITS(input[5], 4, 0) << 7) | TAKE_BITS(input[6], 7, 1);
        output[5] = (TAKE_BITS(input[6], 1, 0) << 10) |
            (TAKE_BITS(input[7], 8, 0) << 2) | TAKE_BITS(input[8], 2, 6);
        output[6] = (TAKE_BITS(input[8], 6, 0) << 5) | TAKE_BITS(input[9], 5, 3);
        output[7] = (TAKE_BITS(input[9], 3, 0) << 8) | TAKE_BITS(input[10], 8, 0);

        input += PACKED11_INPUT_SIZE;
        output += PACKED11_OUTPUT_SIZE;
    }

    bytesWritten_ += nNeededOutput;
}

// ---------------------------------------------------------------------------
// PS Compressed decoder (4-bit nibble RLE/diff)
// ---------------------------------------------------------------------------

void DepthProcessor::processPSCompressed(const uint8_t* data, uint32_t size, bool /*isLastChunk*/)
{
    // Buffer all PSCompressed data for the frame. Actual decoding happens at EOF
    // in onEndOfFrame(), eliminating cross-chunk self-synchronization complexity.
    psFrameBuf_.insert(psFrameBuf_.end(), data, data + size);
}

void DepthProcessor::uncompressDepthPS(const uint8_t* pInput, uint32_t nInputSize, bool bLastPart,
                                        uint32_t& pnWrittenOutput, uint32_t& pnActualRead)
{
    // PrimeSense PSCompressed decoder.
    // When called with bLastPart=true (after full-frame buffering), all data
    // is consumed in one pass — no cross-chunk self-synchronization needed.

    const uint8_t* pCurrInput = pInput;
    const uint8_t* pInputEnd = pInput + nInputSize;
    const uint8_t* pInputOrig = pInput;
    bool bShouldReadByte = true;
    uint8_t nLastByte = 0;

    uint16_t nLastValue = 0;
    uint32_t nInput;
    uint32_t nLargeValue;

    // Output tracking
    uint16_t* pOutput = reinterpret_cast<uint16_t*>(writeBuffer_.data() + bytesWritten_);
    uint16_t* pOutputEnd = reinterpret_cast<uint16_t*>(writeBuffer_.data() + writeBuffer_.size());
    uint16_t* pOutputOrig = pOutput;

    // Per-pixel output: clamp nLastValue to shift range, apply S2D, write pixel.
    // This matches PrimeSense's XN_CHECK_UNC_DEPTH_OUTPUT which modifies the
    // value parameter in-place (since it's a C macro with text substitution).
    // When nLastValue >= maxShiftValue_, it is clamped to 0 so subsequent
    // diffs accumulate from 0 rather than from an out-of-range value.
    #define DEPTH_OUTPUT(val)                                          \
        do {                                                           \
            if (pOutput >= pOutputEnd) goto done;                      \
            if (val >= static_cast<uint16_t>(maxShiftValue_)) {       \
                val = NO_DEPTH_VALUE;                                  \
            }                                                          \
            *pOutput = applyShiftToDepth(val);                         \
            ++pOutput;                                                 \
        } while (0)

    for (;;) {
        // Get next 4-bit nibble
        if (bShouldReadByte) {
            if (pCurrInput == pInputEnd) break;
            nLastByte = *pCurrInput;
            bShouldReadByte = false;
            nInput = nLastByte >> 4;
            pCurrInput++;
        } else {
            nInput = nLastByte & 0x0F;
            bShouldReadByte = true;
        }

        switch (nInput) {
        case 0xD:  // Dummy
            break;

        case 0xE:  // RLE
        {
            uint32_t nCount;
            if (bShouldReadByte) {
                if (pCurrInput == pInputEnd) goto done;
                nLastByte = *pCurrInput;
                bShouldReadByte = false;
                nCount = nLastByte >> 4;
                pCurrInput++;
            } else {
                nCount = nLastByte & 0x0F;
                bShouldReadByte = true;
            }
            nCount++;
            for (uint32_t i = 0; i < nCount; ++i) {
                DEPTH_OUTPUT(nLastValue);
            }
            break;
        }

        case 0xF:  // Large value
        {
            uint32_t nextNib;
            if (bShouldReadByte) {
                if (pCurrInput == pInputEnd) goto done;
                nLastByte = *pCurrInput;
                bShouldReadByte = false;
                nextNib = nLastByte >> 4;
                pCurrInput++;
            } else {
                nextNib = nLastByte & 0x0F;
                bShouldReadByte = true;
            }

            if (nextNib & 0x8) {
                // 7-bit large diff
                nLargeValue = (nextNib - 0x8) << 4;

                uint32_t lowNib;
                if (bShouldReadByte) {
                    if (pCurrInput == pInputEnd) goto done;
                    nLastByte = *pCurrInput;
                    bShouldReadByte = false;
                    lowNib = nLastByte >> 4;
                    pCurrInput++;
                } else {
                    lowNib = nLastByte & 0x0F;
                    bShouldReadByte = true;
                }

                nLargeValue |= lowNib;
                nLastValue += static_cast<uint16_t>(static_cast<int16_t>(nLargeValue) - 64);
            } else {
                // 15-bit full value
                nLargeValue = nextNib << 12;

                for (int nibIdx = 0; nibIdx < 3; ++nibIdx) {
                    uint32_t nib;
                    if (bShouldReadByte) {
                        if (pCurrInput == pInputEnd) goto done;
                        nLastByte = *pCurrInput;
                        bShouldReadByte = false;
                        nib = nLastByte >> 4;
                        pCurrInput++;
                    } else {
                        nib = nLastByte & 0x0F;
                        bShouldReadByte = true;
                    }

                    nLargeValue |= nib << (8 - nibIdx * 4);
                }

                nLastValue = static_cast<uint16_t>(nLargeValue);
            }

            DEPTH_OUTPUT(nLastValue);
            break;
        }

        default:  // 0x0 - 0xC: small diff (-6 to +6)
            nLastValue += static_cast<uint16_t>(static_cast<int16_t>(nInput) - 6);
            DEPTH_OUTPUT(nLastValue);
            break;
        }
    }

done:

    #undef DEPTH_OUTPUT

    pnWrittenOutput = static_cast<uint32_t>(pOutput - pOutputOrig) * sizeof(uint16_t);
    pnActualRead = static_cast<uint32_t>(pCurrInput - pInputOrig);
}

// ---------------------------------------------------------------------------
// Expected size
// ---------------------------------------------------------------------------

uint32_t DepthProcessor::calculateExpectedSize()
{
    return static_cast<uint32_t>(width()) * height() * sizeof(uint16_t);
}
