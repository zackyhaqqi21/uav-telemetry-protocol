// protocol.hpp
// Minimal UAV telemetry packet protocol: encoder + streaming parser.
//
// Packet layout (all single bytes unless noted):
//
//   +-------+-----+--------+-----+-----------+-----------+
//   | START | LEN | MSG_ID | SEQ | PAYLOAD   | CRC16     |
//   | 0xAA  |  n  |        |     | n bytes   | 2 bytes   |
//   +-------+-----+--------+-----+-----------+-----------+
//
// CRC16-CCITT (poly 0x1021, init 0xFFFF) is computed over
// LEN, MSG_ID, SEQ and PAYLOAD (everything except START and the CRC itself).
// CRC is sent low byte first (little-endian). Multi-byte payload fields
// are also little-endian.

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace uavproto {

constexpr uint8_t START_BYTE = 0xAA;
constexpr size_t MAX_PAYLOAD = 255;  // LEN is one byte
constexpr size_t OVERHEAD = 6;       // START + LEN + MSG_ID + SEQ + CRC(2)

enum MsgId : uint8_t {
  MSG_HEARTBEAT = 1,
  MSG_ATTITUDE = 2,   // roll, pitch, yaw as int16, centi-degrees
  MSG_ALT_SPEED = 3,  // altitude (cm, int16), ground speed (cm/s, int16)
  MSG_BATTERY = 4     // voltage (mV, uint16 stored as int16), percent (int16)
};

struct Packet {
  uint8_t msg_id = 0;
  uint8_t seq = 0;
  std::vector<uint8_t> payload;
};

struct Stats {
  uint32_t packets_ok = 0;        // packets that passed the CRC check
  uint32_t crc_errors = 0;        // candidate packets rejected by CRC
  uint32_t bytes_discarded = 0;   // bytes skipped while searching for START
  uint32_t packets_lost = 0;      // estimated from gaps in SEQ
  uint32_t packets_reordered = 0; // SEQ went backwards (late packet, not a loss)
};

// CRC-16/CCITT-FALSE. Check value: crc16_ccitt("123456789") == 0x29B1.
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF);

// Build a complete packet ready to be sent. Throws std::length_error
// if len > MAX_PAYLOAD.
std::vector<uint8_t> encode(uint8_t msg_id, uint8_t seq,
                            const uint8_t* payload, size_t len);
std::vector<uint8_t> encode(uint8_t msg_id, uint8_t seq,
                            const std::vector<uint8_t>& payload);

// Little-endian helpers for payload fields.
inline void put_i16(std::vector<uint8_t>& buf, int16_t v) {
  const uint16_t u = static_cast<uint16_t>(v);
  buf.push_back(static_cast<uint8_t>(u & 0xFF));
  buf.push_back(static_cast<uint8_t>(u >> 8));
}

inline int16_t get_i16(const uint8_t* p) {
  const uint16_t u = static_cast<uint16_t>(
      static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
  return static_cast<int16_t>(u);
}

// Streaming parser (byte-by-byte state machine).
//
// Feed it bytes as they arrive from a serial port / UDP socket, in any
// chunking. Every valid packet is delivered through the callback.
// Garbage bytes, corrupted packets and truncated packets are handled:
// after a CRC failure the parser re-scans the bytes it had buffered,
// so a real packet hiding inside a false candidate is not lost.
class Parser {
 public:
  using Callback = std::function<void(const Packet&)>;

  explicit Parser(Callback on_packet) : on_packet_(std::move(on_packet)) {}

  void feed(uint8_t byte) { process(byte); }
  void feed(const uint8_t* data, size_t len);
  void feed(const std::vector<uint8_t>& data) { feed(data.data(), data.size()); }

  const Stats& stats() const { return stats_; }

 private:
  enum class State { WAIT_START, LEN, ID, SEQ, PAYLOAD, CRC_L, CRC_H };

  void process(uint8_t byte);
  void reset();
  void fail_and_rescan();

  Callback on_packet_;
  Stats stats_;

  State state_ = State::WAIT_START;
  std::vector<uint8_t> raw_;      // bytes of the current candidate, starts at START
  std::vector<uint8_t> payload_;
  uint8_t len_ = 0;
  uint8_t msg_id_ = 0;
  uint8_t seq_ = 0;
  uint16_t crc_rx_ = 0;

  bool has_last_seq_ = false;
  uint8_t last_seq_ = 0;
};

}  // namespace uavproto
