// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"

#include <array>
#include <cstdint>
#include <memory>

namespace blofeld {

struct DspState {
  std::uint32_t pc = 0;
  std::uint32_t sr = 0;
  std::uint32_t omr = 0;
  std::uint32_t sp = 0;
  std::uint32_t la = 0;
  std::uint32_t lc = 0;
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::uint64_t a = 0;
  std::uint64_t b = 0;
  std::array<std::uint32_t, 8> r{};
  std::array<std::uint32_t, 8> n{};
  std::array<std::uint32_t, 8> m{};
};

// Native interpreter reference path. Each DO/REP body instruction can yield
// independently; peripheral time remains an explicit board-model input.
class DspCore {
public:
  explicit DspCore(DspBus &bus);
  ~DspCore();
  DspCore(const DspCore &) = delete;
  DspCore &operator=(const DspCore &) = delete;
  void reset_spi_boot();
  [[nodiscard]] DspState state() const;
  void step();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
