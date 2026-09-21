#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>
#include <functional>

// Protocol packet header (XnSensorProtocolResponseHeader).
// 12 bytes, packed, little-endian on the wire.
#pragma pack(push, 1)
struct PacketHeader {
    uint16_t magic;      // 0x4252 ("BR") for v2.6 firmware
    uint16_t type;        // Packet type (SOBuffer/EOF, depth or image)
    uint16_t packetID;    // Sequence number
    uint16_t bufSize;     // Total payload size INCLUDING this 12-byte header
    uint32_t timestamp;   // Device timestamp in microseconds
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 12, "PacketHeader must be 12 bytes (2+2+2+2+4)");

// Packet type constants (from XnHostProtocol)
namespace PacketType {
    constexpr uint16_t DEPTH_SOF    = 0x7100;  // Depth Start Of Frame
    constexpr uint16_t DEPTH_BUFFER = 0x7200;  // Depth frame data
    constexpr uint16_t DEPTH_EOF    = 0x7500;  // Depth End Of Frame
    constexpr uint16_t IMAGE_SOF    = 0x8100;  // IR/Color Start Of Frame
    constexpr uint16_t IMAGE_BUFFER = 0x8200;  // IR/Color frame data
    constexpr uint16_t IMAGE_EOF    = 0x8500;  // IR/Color End Of Frame
    constexpr uint16_t AUDIO_BUFFER = 0x9200; // Audio data
}

// Callback type for when a complete packet chunk is ready.
//   header:     parsed packet header
//   data:       pointer to the data portion (after the 12-byte header)
//   dataOffset: byte offset within the full packet data (0 for SOF, cumulative for buffers)
//   dataSize:   number of valid bytes in data for this chunk
using PacketCallback = std::function<void(const PacketHeader& header,
                                          const uint8_t* data,
                                          uint32_t dataOffset,
                                          uint32_t dataSize)>;

// Parses raw USB bulk data into protocol packets.
//
// Implements the XnDeviceSensorProtocolUsbEpCb state machine:
//   LOOKING_FOR_MAGIC -> PACKET_HEADER -> PACKET_DATA -> LOOKING_FOR_MAGIC
//
// Incoming USB bulk transfers may split packet data across arbitrary boundaries.
// This class reassembles complete packets and invokes the callback for each one.
class PacketParser {
public:
    PacketParser();
    ~PacketParser();

    // Feed raw USB data into the parser. May trigger zero or more callbacks.
    void feed(const uint8_t* data, size_t length);

    // Set callback for parsed packets.
    void setCallback(PacketCallback cb);

    // Reset state (e.g., on stream restart).
    void reset();

    // Set the magic value to look for. Default 0x4252 ("BR").
    void setMagic(uint16_t magic);

private:
    enum State {
        LOOKING_FOR_MAGIC,
        PACKET_HEADER,
        PACKET_DATA
    };

    State state_ = LOOKING_FOR_MAGIC;
    uint16_t magic_ = 0x4252;
    PacketCallback callback_;

    // Accumulation buffer for partial data
    std::vector<uint8_t> buffer_;
    size_t consumePos_ = 0;  // read position within buffer_

    // Current packet being assembled
    PacketHeader currentHeader_;
    std::vector<uint8_t> currentData_;
    uint32_t dataReceived_ = 0;
    uint32_t dataExpected_ = 0;  // bufSize - sizeof(PacketHeader)

    // Process as much data from buffer_ as possible
    void process();

    // State handlers. Return true if they consumed data and should be called again.
    bool handleLookingForMagic();
    bool handlePacketHeader();
    bool handlePacketData();

    // Dispatch completed packet chunk via callback
    void dispatchPacket();

    // Compact buffer_: remove consumed bytes, reset consumePos_
    void compactBuffer();

    // Number of unconsumed bytes in buffer_
    size_t available() const { return buffer_.size() - consumePos_; }

    // Pointer to next unconsumed byte
    const uint8_t* nextByte() const { return buffer_.data() + consumePos_; }

    // Per-instance diagnostics
    int diagPackets_ = 0;
    int diagInvalidType_ = 0;
    int diagInvalidSize_ = 0;
    size_t diagBytesDispatched_ = 0;
    size_t diagBytesFed_ = 0;
};
