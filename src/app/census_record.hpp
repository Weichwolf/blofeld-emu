// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "census.hpp"

#include <cstdint>

namespace blofeld::app {
class WavConsumer;

class CensusRecord {
public:
  void observe(const vi::Census &census, const WavConsumer &wav) noexcept;
  void report(const vi::Census &census) const;

private:
  vi::Census::Snapshot first_, last_;
  std::uint64_t first_frame_ = 0, last_frame_ = 0, first_source_ = 0, last_source_ = 0;
};
}
