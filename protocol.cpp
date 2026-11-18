// protocol.cpp
#include "protocol.hpp"

#include <stdexcept>
#include <utility>

namespace uavproto {

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8);
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> encode(uint8_t msg_id, uint8_t seq,
                            const uint8_t* payload, size_t len) {
  if (len > MAX_PAYLOAD) {
    throw std::length_error("payload too large (max 255 bytes)");
  }

  std::vector<uint8_t> out;
  out.reserve(len + OVERHEAD);
  out.push_back(START_BYTE);
  out.push_back(static_cast<uint8_t>(len));
  out.push_back(msg_id);
  out.push_back(seq);
  if (len > 0) {
    out.insert(out.end(), payload, payload + len);
  }

  // CRC covers everything after START: LEN, MSG_ID, SEQ, PAYLOAD.
  const uint16_t crc = crc16_ccitt(out.data() + 1, out.size() - 1);
  out.push_back(static_cast<uint8_t>(crc & 0xFF));  // low byte first
  out.push_back(static_cast<uint8_t>(crc >> 8));
  return out;
}

std::vector<uint8_t> encode(uint8_t msg_id, uint8_t seq,
                            const std::vector<uint8_t>& payload) {
  return encode(msg_id, seq, payload.data(), payload.size());
}

void Parser::feed(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    process(data[i]);
  }
}

void Parser::reset() {
  state_ = State::WAIT_START;
  raw_.clear();
  payload_.clear();
}

void Parser::process(uint8_t b) {
  switch (state_) {
    case State::WAIT_START:
      if (b == START_BYTE) {
        raw_.clear();
        raw_.push_back(b);
        state_ = State::LEN;
      } else {
        ++stats_.bytes_discarded;
      }
      break;

    case State::LEN:
      raw_.push_back(b);
      len_ = b;
      payload_.clear();
      state_ = State::ID;
      break;

    case State::ID:
      raw_.push_back(b);
      msg_id_ = b;
      state_ = State::SEQ;
      break;

    case State::SEQ:
      raw_.push_back(b);
      seq_ = b;
      state_ = (len_ == 0) ? State::CRC_L : State::PAYLOAD;
      break;

    case State::PAYLOAD:
      raw_.push_back(b);
      payload_.push_back(b);
      if (payload_.size() == len_) {
        state_ = State::CRC_L;
      }
      break;

    case State::CRC_L:
      raw_.push_back(b);
      crc_rx_ = b;
      state_ = State::CRC_H;
      break;

    case State::CRC_H: {
      raw_.push_back(b);
      crc_rx_ = static_cast<uint16_t>(crc_rx_ | (static_cast<uint16_t>(b) << 8));

      // raw_ = START, LEN, ID, SEQ, PAYLOAD..., CRC_L, CRC_H
      // CRC covers raw_[1 .. size-3]
      const uint16_t crc_calc = crc16_ccitt(raw_.data() + 1, raw_.size() - 3);

      if (crc_calc == crc_rx_) {
        // Estimate lost packets from SEQ gap (SEQ wraps at 256).
        if (has_last_seq_) {
          const uint8_t expected = static_cast<uint8_t>(last_seq_ + 1);
          const uint8_t delta = static_cast<uint8_t>(seq_ - expected);
          if (delta < 128) {
            // Forward jump: `delta` packets were skipped.
            stats_.packets_lost += delta;
            last_seq_ = seq_;
          } else {
            // Backward jump: a late (reordered) packet, not a loss. It was
            // probably counted as lost when its successor arrived, so take
            // that back. last_seq_ stays put so the next in-order packet
            // does not look like a gap.
            ++stats_.packets_reordered;
            if (stats_.packets_lost > 0) --stats_.packets_lost;
          }
        } else {
          last_seq_ = seq_;
        }
        has_last_seq_ = true;
        ++stats_.packets_ok;

        Packet pkt;
        pkt.msg_id = msg_id_;
        pkt.seq = seq_;
        pkt.payload = payload_;
        reset();
        if (on_packet_) on_packet_(pkt);
      } else {
        ++stats_.crc_errors;
        fail_and_rescan();
      }
      break;
    }
  }
}

// The START byte we locked onto was probably a false one (garbage or a byte
// inside another packet). Drop only that START byte and replay everything
// that came after it through the state machine, so a real packet that was
// swallowed by the false candidate is found again.
void Parser::fail_and_rescan() {
  const std::vector<uint8_t> replay(raw_.begin() + 1, raw_.end());
  ++stats_.bytes_discarded;  // the false START
  reset();
  for (uint8_t b : replay) {
    process(b);
  }
}

}  // namespace uavproto
