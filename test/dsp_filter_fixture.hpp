// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "dsp_filter_model.hpp"

#include <cstdint>
#include <set>
#include <span>

namespace blofeld::test::filters {
struct Sources {
  std::span<const unsigned> program;
  std::span<const unsigned> table;
};

struct Counts {
  unsigned calls = 0;
  unsigned wide = 0;
  unsigned clipped = 0;
  unsigned halved = 0;
  unsigned capped = 0;
  std::uint64_t instructions = 0;
};

class Fixture {
public:
  explicit Fixture(Sources sources);
  void run(const Input &input);
  void intact();

  [[nodiscard]] const Counts &counts() const { return counts_; }

  [[nodiscard]] const std::set<unsigned> &visited() const { return visited_; }

private:
  static DspBus &prepare(DspBus &bus, Sources sources, unsigned &stop);
  void write(DspSpace space, unsigned address, unsigned value);
  unsigned read(DspSpace space, unsigned address);
  void seed(const Input &input, const Prediction &prediction);
  void verify_section(const Input &input, const Prediction &prediction, unsigned section);
  void verify_result(const Input &input, const Prediction &prediction);
  void verify_history(const History &history, unsigned offset, const std::string &context);
  DspBus bus_;
  unsigned stop_ = 0;
  std::span<const unsigned> program_;
  std::span<const unsigned> table_;
  DspCore cpu_;
  Counts counts_;
  std::set<unsigned> visited_;
};
}
