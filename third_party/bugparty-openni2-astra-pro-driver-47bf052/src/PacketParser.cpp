#include "PacketParser.h"
#include "AstraDebug.h"

#include <cstring>
#include <algorithm>

PacketParser::PacketParser()
{
    buffer_.reserve(16384);
}

PacketParser::~PacketParser()
{
}

void PacketParser::feed(const uint8_t* data, size_t length)
{
    if (length == 0) return;

    // Compact buffer if there's a lot of consumed data
    if (consumePos_ > 4096) {
        compactBuffer();
    }

    // Append new data
    buffer_.insert(buffer_.end(), data, data + length);

    // Process as much as possible
    process();
}

void PacketParser::setCallback(PacketCallback cb)
{
    callback_ = std::move(cb);
}

void PacketParser::reset()
{
    if (diagPackets_ > 0) {
        ASTRA_DLOG("DIAG parser reset: dispatched=%d invalidType=%d invalidSize=%d "
                "bytesDispatched=%zu bytesFed=%zu\n",
                diagPackets_, diagInvalidType_, diagInvalidSize_,
                diagBytesDispatched_, diagBytesFed_);
    }

    state_ = LOOKING_FOR_MAGIC;
    buffer_.clear();
    consumePos_ = 0;
    currentData_.clear();
    dataReceived_ = 0;
    dataExpected_ = 0;
}

void PacketParser::setMagic(uint16_t magic)
{
    magic_ = magic;
}

void PacketParser::process()
{
    // Keep processing until a state handler makes no progress
    bool progress;
    do {
        progress = false;
        switch (state_) {
        case LOOKING_FOR_MAGIC:
            progress = handleLookingForMagic();
            break;
        case PACKET_HEADER:
            progress = handlePacketHeader();
            break;
        case PACKET_DATA:
            progress = handlePacketData();
            break;
        }
    } while (progress);
}

bool PacketParser::handleLookingForMagic()
{
    // We need at least 2 bytes to check for magic
    if (available() < 2) return false;

    const uint8_t* p = nextByte();
    size_t avail = available();

    // Magic is 0x4252, stored little-endian on wire: 0x52 0x42
    const uint8_t magicLo = magic_ & 0xFF;       // 0x52
    const uint8_t magicHi = (magic_ >> 8) & 0xFF; // 0x42

    for (size_t i = 0; i <= avail - 2; i++) {
        if (p[i] == magicLo && p[i + 1] == magicHi) {
            // Found magic. Discard everything before it.
            diagBytesFed_ += i;
            consumePos_ += i;
            state_ = PACKET_HEADER;
            return true;
        }
    }

    // No magic found. Discard all but the last byte (it could be the start of magic).
    diagBytesFed_ += avail - 1;
    consumePos_ += avail - 1;
    return false;
}

bool PacketParser::handlePacketHeader()
{
    // Need 12 bytes for the full PacketHeader
    if (available() < sizeof(PacketHeader)) return false;

    // Copy header data (handles potential misalignment)
    memcpy(&currentHeader_, nextByte(), sizeof(PacketHeader));

    // PrimeSense protocol quirk: nBufSize is stored little-endian on wire,
    // but must be byte-swapped before use (matching XnHostProtocol's
    // xnOSEndianSwapUINT16 call on nBufSize after XN_PREPARE_VAR16_IN_BUFFER).
    // Raw wire bytes 0c 00 -> LE read as 12 -> swap to 3072 -> subtract header = 3060 data bytes.
    currentHeader_.bufSize = __builtin_bswap16(currentHeader_.bufSize);

    // Validate packet type. The magic bytes 0x52 0x42 can appear in compressed
    // depth data. A random occurrence will almost certainly produce a type field
    // that isn't one of the known PrimeSense packet types. Rejecting unknown types
    // prevents the parser from consuming large chunks of real data as false packets.
    uint16_t t = currentHeader_.type;
    bool validType = (t == PacketType::DEPTH_SOF || t == PacketType::DEPTH_BUFFER || t == PacketType::DEPTH_EOF ||
                      t == PacketType::IMAGE_SOF || t == PacketType::IMAGE_BUFFER || t == PacketType::IMAGE_EOF ||
                      t == PacketType::AUDIO_BUFFER);
    if (!validType) {
        diagInvalidType_++;
        if (diagInvalidType_ <= 10) {
            ASTRA_DLOG("DIAG parse false magic: type=0x%04x bufSize=%u pktId=%u (invalid type #%d)\n",
                    currentHeader_.type, currentHeader_.bufSize, currentHeader_.packetID, diagInvalidType_);
        }
        consumePos_ += 2;  // skip past the magic bytes only
        diagBytesFed_ += 2;
        state_ = LOOKING_FOR_MAGIC;
        return true;
    }

    // Validate: bufSize must be at least the header size
    if (currentHeader_.bufSize < sizeof(PacketHeader)) {
        diagInvalidSize_++;
        if (diagInvalidSize_ <= 5) {
            ASTRA_DLOG("DIAG parse invalid bufSize=%u (< %zu) type=0x%04x, resyncing (invalid #%d)\n",
                    currentHeader_.bufSize, sizeof(PacketHeader), currentHeader_.type, diagInvalidSize_);
        }
        consumePos_ += 2;  // skip past the magic bytes
        diagBytesFed_ += 2;
        state_ = LOOKING_FOR_MAGIC;
        return true;
    }

    // Sanity check: bufSize shouldn't exceed a reasonable maximum
    constexpr uint32_t MAX_BUF_SIZE = 65536;
    if (currentHeader_.bufSize > MAX_BUF_SIZE) {
        diagInvalidSize_++;
        if (diagInvalidSize_ <= 5) {
            ASTRA_DLOG("DIAG parse oversized bufSize=%u (> %u) type=0x%04x, resyncing\n",
                    currentHeader_.bufSize, MAX_BUF_SIZE, currentHeader_.type);
        }
        consumePos_ += 2;
        diagBytesFed_ += 2;
        state_ = LOOKING_FOR_MAGIC;
        return true;
    }

    // Valid header found. Advance past it.
    consumePos_ += sizeof(PacketHeader);
    diagBytesFed_ += sizeof(PacketHeader);

    // Calculate data portion size
    dataExpected_ = currentHeader_.bufSize - sizeof(PacketHeader);
    dataReceived_ = 0;

    // Prepare data buffer
    currentData_.resize(dataExpected_);

    // If there's no data payload, dispatch immediately
    if (dataExpected_ == 0) {
        dispatchPacket();
        state_ = LOOKING_FOR_MAGIC;
        return true;
    }

    state_ = PACKET_DATA;
    return true;
}

bool PacketParser::handlePacketData()
{
    if (available() == 0) return false;

    uint32_t remaining = dataExpected_ - dataReceived_;
    size_t toCopy = std::min(remaining, static_cast<uint32_t>(available()));

    memcpy(currentData_.data() + dataReceived_, nextByte(), toCopy);
    consumePos_ += toCopy;
    diagBytesFed_ += toCopy;
    dataReceived_ += toCopy;

    if (dataReceived_ >= dataExpected_) {
        dispatchPacket();
        state_ = LOOKING_FOR_MAGIC;
        return true;
    }

    // Consumed all available data but packet is incomplete
    return false;
}

void PacketParser::dispatchPacket()
{
    diagPackets_++;
    diagBytesDispatched_ += dataReceived_;

    if (callback_) {
        uint32_t dataSize = dataReceived_;
        // For SOF/EOF packets with no data, data pointer can be nullptr
        const uint8_t* dataPtr = (dataSize > 0) ? currentData_.data() : nullptr;
        callback_(currentHeader_, dataPtr, 0, dataSize);
    }
    currentData_.clear();
    dataReceived_ = 0;
    dataExpected_ = 0;
}

void PacketParser::compactBuffer()
{
    if (consumePos_ > 0) {
        size_t remaining = buffer_.size() - consumePos_;
        if (remaining > 0) {
            memmove(buffer_.data(), buffer_.data() + consumePos_, remaining);
        }
        buffer_.resize(remaining);
        consumePos_ = 0;
    }
}
