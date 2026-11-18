// noisy_link.cpp
#include "noisy_link.hpp"

#include <utility>

namespace uavsim {

NoiseConfig uniform_noise(double level) {
  NoiseConfig c;
  c.bit_flip = level / 10.0;
  c.byte_drop = level / 10.0;
  c.truncate = level;
  c.garbage = level;
  c.reorder = level;
  return c;
}

bool NoisyLink::chance(double p) {
  if (p <= 0.0) return false;
  return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < p;
}

std::vector<uint8_t> NoisyLink::transmit(
    std::vector<std::vector<uint8_t>> packets) {
  // 1) Reordering: swap a packet with its neighbour (no chained swaps).
  for (size_t i = 0; i + 1 < packets.size(); ++i) {
    if (chance(cfg_.reorder)) {
      std::swap(packets[i], packets[i + 1]);
      ++counts_.packets_swapped;
      ++i;
    }
  }

  // 2) Per-packet and per-byte damage while serialising to a byte stream.
  std::vector<uint8_t> stream;
  for (auto& pkt : packets) {
    if (chance(cfg_.garbage)) {
      const int n = 1 + static_cast<int>(rng_() % 16);
      for (int g = 0; g < n; ++g) {
        stream.push_back(static_cast<uint8_t>(rng_() & 0xFF));
      }
      ++counts_.garbage_bursts;
    }

    if (pkt.size() > 1 && chance(cfg_.truncate)) {
      pkt.resize(1 + rng_() % (pkt.size() - 1));  // keep 1 .. size-1 bytes
      ++counts_.packets_truncated;
    }

    for (uint8_t b : pkt) {
      if (chance(cfg_.byte_drop)) {
        ++counts_.bytes_dropped;
        continue;
      }
      if (chance(cfg_.bit_flip)) {
        b ^= static_cast<uint8_t>(1u << (rng_() % 8));
        ++counts_.bits_flipped;
      }
      stream.push_back(b);
    }
  }
  return stream;
}

}  // namespace uavsim
