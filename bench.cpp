// bench.cpp
// Robustness benchmark: pushes packets through a noisy link and measures
// what the parser delivers. Build:
//   g++ -std=c++17 -Wall -Wextra -O2 protocol.cpp noisy_link.cpp bench.cpp -o bench
//   ./bench            -> writes sweep.csv and per_fault.csv

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "noisy_link.hpp"
#include "protocol.hpp"

using namespace uavproto;
using namespace uavsim;

namespace {

constexpr int kPackets = 1000;  // packets per trial
constexpr int kTrials = 30;     // independent random seeds per setting

// Payload carries the packet index, so the receiver side of the experiment
// can tell exactly which packet arrived and whether its content is intact.
std::vector<uint8_t> make_payload(uint32_t i) {
  std::vector<uint8_t> p;
  put_i16(p, static_cast<int16_t>(i & 0xFFFF));
  put_i16(p, static_cast<int16_t>((i >> 16) & 0xFFFF));
  put_i16(p, static_cast<int16_t>((i * 31u) & 0xFFFF));
  return p;
}

struct TrialResult {
  int delivered = 0;      // unique, intact, correct packets
  int false_accepts = 0;  // passed CRC but content/seq/id does not match any sent packet
  int duplicates = 0;
  int out_of_order = 0;
  uint32_t crc_errors = 0;
  uint32_t lost_estimate = 0;  // parser's own packets_lost
};

TrialResult run_trial(const NoiseConfig& cfg, uint32_t seed) {
  std::vector<std::vector<uint8_t>> packets;
  packets.reserve(kPackets);
  for (int i = 0; i < kPackets; ++i) {
    packets.push_back(encode(MSG_ATTITUDE, static_cast<uint8_t>(i & 0xFF),
                             make_payload(static_cast<uint32_t>(i))));
  }

  NoisyLink link(cfg, seed);
  std::vector<uint8_t> stream = link.transmit(std::move(packets));
  stream.insert(stream.end(), 300, 0x00);  // keep the link "alive" so the parser can flush

  TrialResult r;
  std::vector<bool> seen(kPackets, false);
  int last_idx = -1;

  Parser parser([&](const Packet& p) {
    bool ok = p.msg_id == MSG_ATTITUDE && p.payload.size() == 6;
    int idx = -1;
    if (ok) {
      idx = static_cast<uint16_t>(get_i16(&p.payload[0]));
      const uint16_t hi = static_cast<uint16_t>(get_i16(&p.payload[2]));
      const uint16_t chk = static_cast<uint16_t>(get_i16(&p.payload[4]));
      ok = idx < kPackets && hi == 0 &&
           chk == static_cast<uint16_t>((static_cast<uint32_t>(idx) * 31u) & 0xFFFF) &&
           p.seq == static_cast<uint8_t>(idx & 0xFF);
    }
    if (!ok) {
      ++r.false_accepts;
      return;
    }
    if (seen[idx]) {
      ++r.duplicates;
      return;
    }
    seen[idx] = true;
    ++r.delivered;
    if (idx < last_idx) ++r.out_of_order;
    last_idx = idx;
  });
  parser.feed(stream);

  r.crc_errors = parser.stats().crc_errors;
  r.lost_estimate = parser.stats().packets_lost;
  return r;
}

struct Agg {
  double delivery_mean = 0, delivery_std = 0;
  double crc_errors = 0, lost_est = 0, lost_true = 0, out_of_order = 0;
  int false_accepts = 0, duplicates = 0;
};

Agg run_setting(const NoiseConfig& cfg, uint32_t seed_base) {
  std::vector<double> deliv;
  Agg a;
  for (int t = 0; t < kTrials; ++t) {
    const TrialResult r = run_trial(cfg, seed_base + static_cast<uint32_t>(t));
    deliv.push_back(100.0 * r.delivered / kPackets);
    a.crc_errors += r.crc_errors;
    a.lost_est += r.lost_estimate;
    a.lost_true += kPackets - r.delivered;
    a.out_of_order += r.out_of_order;
    a.false_accepts += r.false_accepts;
    a.duplicates += r.duplicates;
  }
  double sum = 0;
  for (double d : deliv) sum += d;
  a.delivery_mean = sum / kTrials;
  double var = 0;
  for (double d : deliv) var += (d - a.delivery_mean) * (d - a.delivery_mean);
  a.delivery_std = std::sqrt(var / kTrials);
  a.crc_errors /= kTrials;
  a.lost_est /= kTrials;
  a.lost_true /= kTrials;
  a.out_of_order /= kTrials;
  return a;
}

}  // namespace

int main() {
  std::printf("packets/trial=%d  trials/setting=%d\n\n", kPackets, kTrials);

  // ---- Experiment 1: all fault types together, increasing noise level ----
  const double levels[] = {0.0, 0.005, 0.01, 0.02, 0.03, 0.05, 0.075, 0.10, 0.15, 0.20};
  std::FILE* f1 = std::fopen("sweep.csv", "w");
  std::fprintf(f1, "level_pct,delivery_mean,delivery_std,crc_errors,lost_est,lost_true,out_of_order,false_accepts,duplicates\n");
  std::printf("%-8s %-12s %-8s %-10s %-10s %-10s %-6s\n", "noise%", "delivered%",
              "std", "crc_err", "lost_est", "lost_true", "FALSE");
  int total_false = 0;
  for (double lv : levels) {
    const Agg a = run_setting(uniform_noise(lv), 1000);
    std::printf("%-8.1f %-12.2f %-8.2f %-10.1f %-10.1f %-10.1f %-6d\n", lv * 100,
                a.delivery_mean, a.delivery_std, a.crc_errors, a.lost_est,
                a.lost_true, a.false_accepts);
    std::fprintf(f1, "%.1f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d,%d\n", lv * 100,
                 a.delivery_mean, a.delivery_std, a.crc_errors, a.lost_est,
                 a.lost_true, a.out_of_order, a.false_accepts, a.duplicates);
    total_false += a.false_accepts;
  }
  std::fclose(f1);

  // ---- Experiment 2: one fault type at a time, same intensity (10%) ----
  struct Named { const char* name; NoiseConfig cfg; };
  const double L = 0.10;
  std::vector<Named> single = {
      {"bit_flip", {L / 10, 0, 0, 0, 0}},
      {"byte_drop", {0, L / 10, 0, 0, 0}},
      {"truncate", {0, 0, L, 0, 0}},
      {"garbage", {0, 0, 0, L, 0}},
      {"reorder", {0, 0, 0, 0, L}},
  };
  std::FILE* f2 = std::fopen("per_fault.csv", "w");
  std::fprintf(f2, "fault,delivery_mean,delivery_std,crc_errors,lost_est,lost_true,out_of_order,false_accepts\n");
  std::printf("\nOne fault type at a time (each hits ~%.0f%% of packets)\n", L * 100);
  for (const auto& n : single) {
    const Agg a = run_setting(n.cfg, 5000);
    std::printf("%-10s delivered=%.2f%%  lost_est=%.1f  lost_true=%.1f  FALSE=%d\n",
                n.name, a.delivery_mean, a.lost_est, a.lost_true, a.false_accepts);
    std::fprintf(f2, "%s,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%d\n", n.name, a.delivery_mean,
                 a.delivery_std, a.crc_errors, a.lost_est, a.lost_true,
                 a.out_of_order, a.false_accepts);
    total_false += a.false_accepts;
  }
  std::fclose(f2);

  std::printf("\nTOTAL FALSE ACCEPTS (corrupted packet delivered as valid): %d\n", total_false);
  return 0;
}
