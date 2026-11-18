// test_main.cpp
// Plain-C++ tests (no framework needed). Build:
//   g++ -std=c++17 -Wall -Wextra -O2 protocol.cpp test_main.cpp -o test_protocol

#include <cstdio>
#include <random>
#include <vector>

#include "protocol.hpp"

using namespace uavproto;

static int g_failed = 0;

#define CHECK(cond)                                                  \
  do {                                                               \
    if (!(cond)) {                                                   \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);        \
      ++g_failed;                                                    \
    }                                                                \
  } while (0)

static std::vector<uint8_t> attitude_payload(int16_t roll, int16_t pitch,
                                             int16_t yaw) {
  std::vector<uint8_t> p;
  put_i16(p, roll);
  put_i16(p, pitch);
  put_i16(p, yaw);
  return p;
}

static std::vector<Packet> run(const std::vector<uint8_t>& stream, Stats* stats) {
  std::vector<Packet> got;
  Parser parser([&](const Packet& p) { got.push_back(p); });
  parser.feed(stream);
  if (stats) *stats = parser.stats();
  return got;
}

static void append(std::vector<uint8_t>& dst, const std::vector<uint8_t>& src) {
  dst.insert(dst.end(), src.begin(), src.end());
}

// Stream parsers only decide a candidate is false once enough bytes arrive,
// so after the last packet we feed some padding (a real link keeps sending).
static void flush_padding(std::vector<uint8_t>& stream) {
  stream.insert(stream.end(), 300, 0x00);
}

static void test_crc_check_value() {
  std::printf("[1] CRC16-CCITT check value\n");
  const uint8_t msg[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16_ccitt(msg, sizeof(msg)) == 0x29B1);
}

static void test_roundtrip() {
  std::printf("[2] Encode -> parse round trip\n");
  const auto pkt = encode(MSG_ATTITUDE, 7, attitude_payload(-1234, 560, 18000));
  CHECK(pkt.size() == 6 + OVERHEAD);

  Stats st;
  const auto got = run(pkt, &st);
  CHECK(got.size() == 1);
  if (got.size() == 1) {
    CHECK(got[0].msg_id == MSG_ATTITUDE);
    CHECK(got[0].seq == 7);
    CHECK(got[0].payload.size() == 6);
    CHECK(get_i16(&got[0].payload[0]) == -1234);
    CHECK(get_i16(&got[0].payload[2]) == 560);
    CHECK(get_i16(&got[0].payload[4]) == 18000);
  }
  CHECK(st.crc_errors == 0);
}

static void test_empty_payload() {
  std::printf("[3] Zero-length payload (heartbeat)\n");
  const auto pkt = encode(MSG_HEARTBEAT, 0, nullptr, 0);
  CHECK(pkt.size() == OVERHEAD);
  const auto got = run(pkt, nullptr);
  CHECK(got.size() == 1);
}

static void test_corrupted_crc() {
  std::printf("[4] Corrupted packet is rejected\n");
  auto pkt = encode(MSG_ATTITUDE, 1, attitude_payload(100, 200, 300));
  pkt[5] ^= 0x01;  // flip one bit inside the payload
  Stats st;
  const auto got = run(pkt, &st);
  CHECK(got.empty());
  CHECK(st.crc_errors == 1);
}

static void test_truncated_then_valid() {
  std::printf("[5] Truncated packet followed by a valid one\n");
  auto a = encode(MSG_ATTITUDE, 1, attitude_payload(1, 2, 3));
  a.resize(a.size() - 3);  // lose the tail of packet A
  const auto b = encode(MSG_ATTITUDE, 2, attitude_payload(10, 20, 30));

  std::vector<uint8_t> stream;
  append(stream, a);
  append(stream, b);
  flush_padding(stream);

  const auto got = run(stream, nullptr);
  CHECK(got.size() == 1);
  if (got.size() == 1) CHECK(got[0].seq == 2);
}

static void test_sequence_loss() {
  std::printf("[6] Packet loss estimated from SEQ gaps\n");
  std::vector<uint8_t> stream;
  append(stream, encode(MSG_HEARTBEAT, 0, nullptr, 0));
  append(stream, encode(MSG_HEARTBEAT, 1, nullptr, 0));
  append(stream, encode(MSG_HEARTBEAT, 5, nullptr, 0));  // 2,3,4 missing
  Stats st;
  const auto got = run(stream, &st);
  CHECK(got.size() == 3);
  CHECK(st.packets_lost == 3);
}

static void test_seq_wraparound() {
  std::printf("[7] SEQ wraps 255 -> 0 without counting loss\n");
  std::vector<uint8_t> stream;
  append(stream, encode(MSG_HEARTBEAT, 254, nullptr, 0));
  append(stream, encode(MSG_HEARTBEAT, 255, nullptr, 0));
  append(stream, encode(MSG_HEARTBEAT, 0, nullptr, 0));
  Stats st;
  const auto got = run(stream, &st);
  CHECK(got.size() == 3);
  CHECK(st.packets_lost == 0);
}

static void test_garbage_between_packets() {
  std::printf("[8] 200 packets with random garbage in between\n");
  std::mt19937 rng(12345);  // fixed seed -> reproducible
  const int N = 200;

  std::vector<uint8_t> stream;
  for (int i = 0; i < N; ++i) {
    const int garbage = static_cast<int>(rng() % 16);
    for (int g = 0; g < garbage; ++g) {
      stream.push_back(static_cast<uint8_t>(rng() & 0xFF));
    }
    append(stream, encode(MSG_ATTITUDE, static_cast<uint8_t>(i & 0xFF),
                          attitude_payload(static_cast<int16_t>(i),
                                           static_cast<int16_t>(-i),
                                           static_cast<int16_t>(i * 2))));
  }
  flush_padding(stream);

  Stats st;
  const auto got = run(stream, &st);
  CHECK(got.size() == static_cast<size_t>(N));

  bool content_ok = (got.size() == static_cast<size_t>(N));
  for (size_t i = 0; content_ok && i < got.size(); ++i) {
    content_ok = got[i].payload.size() == 6 &&
                 get_i16(&got[i].payload[0]) == static_cast<int16_t>(i) &&
                 get_i16(&got[i].payload[2]) == static_cast<int16_t>(-static_cast<int>(i));
  }
  CHECK(content_ok);
  CHECK(st.packets_lost == 0);

  std::printf("    sent=%d  received=%u  crc_errors=%u  bytes_discarded=%u  lost=%u\n",
              N, st.packets_ok, st.crc_errors, st.bytes_discarded,
              st.packets_lost);
}

static void test_chunked_input() {
  std::printf("[9] Same result when bytes arrive in tiny chunks\n");
  std::vector<uint8_t> stream;
  for (int i = 0; i < 20; ++i) {
    append(stream, encode(MSG_ALT_SPEED, static_cast<uint8_t>(i),
                          attitude_payload(static_cast<int16_t>(i * 100), 0, 0)));
  }
  std::vector<Packet> got;
  Parser parser([&](const Packet& p) { got.push_back(p); });
  for (size_t i = 0; i < stream.size(); i += 3) {  // 3 bytes at a time
    const size_t n = (stream.size() - i < 3) ? (stream.size() - i) : 3;
    parser.feed(&stream[i], n);
  }
  CHECK(got.size() == 20);
}

static void test_reordered_not_lost() {
  std::printf("[10] Reordered packets are not counted as lost\n");
  std::vector<uint8_t> stream;
  const uint8_t order[] = {0, 1, 3, 2, 4, 5};  // 3 and 2 swapped
  for (uint8_t s : order) append(stream, encode(MSG_HEARTBEAT, s, nullptr, 0));
  Stats st;
  const auto got = run(stream, &st);
  CHECK(got.size() == 6);
  CHECK(st.packets_lost == 0);
  CHECK(st.packets_reordered == 1);
}

int main() {
  test_crc_check_value();
  test_roundtrip();
  test_empty_payload();
  test_corrupted_crc();
  test_truncated_then_valid();
  test_sequence_loss();
  test_seq_wraparound();
  test_garbage_between_packets();
  test_chunked_input();
  test_reordered_not_lost();

  if (g_failed == 0) {
    std::printf("\nALL TESTS PASSED\n");
    return 0;
  }
  std::printf("\n%d CHECK(S) FAILED\n", g_failed);
  return 1;
}
