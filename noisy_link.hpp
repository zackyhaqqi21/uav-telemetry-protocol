// noisy_link.hpp
// Simulates a bad radio / serial link. Takes whole encoded packets in and
// returns the damaged byte stream the receiver would actually see.
//
// Five fault types, each with its own rate:
//   bit_flip   per BYTE   : flip one random bit
//   byte_drop  per BYTE   : byte disappears
//   truncate   per PACKET : packet is cut short (tail lost)
//   garbage    per PACKET : 1..16 random bytes inserted before the packet
//   reorder    per PACKET : packet is swapped with the next one

#pragma once

#include <cstdint>
#include <random>
#include <vector>

namespace uavsim {

struct NoiseConfig {
  double bit_flip = 0.0;
  double byte_drop = 0.0;
  double truncate = 0.0;
  double garbage = 0.0;
  double reorder = 0.0;
};

// How many faults were actually injected (ground truth for the experiment).
struct NoiseCounts {
  uint32_t bits_flipped = 0;
  uint32_t bytes_dropped = 0;
  uint32_t packets_truncated = 0;
  uint32_t garbage_bursts = 0;
  uint32_t packets_swapped = 0;
};

// Single "noise level" knob. Packets are ~12 bytes, so per-byte rates are
// divided by 10: every fault type then hits roughly `level` (0..1) of all
// packets, which makes the five fault types comparable on one axis.
NoiseConfig uniform_noise(double level);

class NoisyLink {
 public:
  NoisyLink(const NoiseConfig& cfg, uint32_t seed) : cfg_(cfg), rng_(seed) {}

  std::vector<uint8_t> transmit(std::vector<std::vector<uint8_t>> packets);

  const NoiseCounts& counts() const { return counts_; }

 private:
  bool chance(double p);

  NoiseConfig cfg_;
  NoiseCounts counts_;
  std::mt19937 rng_;
};

}  // namespace uavsim
