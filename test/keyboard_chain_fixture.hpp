// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "keyboard_board_reset_fixture.hpp"
#include "keyboard_handoff_fixture.hpp"
#include "lcd_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <streambuf>
#include <string>
#include <string_view>
#include <vector>

namespace blofeld::test::keyboard_chain {
namespace kh = keyboard_handoff;
namespace kr = keyboard_board_reset;
namespace transport = update_transport;
constexpr unsigned data_offset = 1020, page_offset = 1023, page_bytes = 512, record_bytes = 514;
constexpr unsigned frame = 0x21e7, text_entry = 0xa6ee, text_return = 0x124b, buffer_ready = 0x12fc;
constexpr unsigned byte_return = 0x1545, page_ready = 0x1281, expected_rx = 2724,
                   expected_commands = 684;
constexpr unsigned max_steps = 4000000;

// Compress only routine instruction/known-pad rows; retain every semantic event,
// unknown/conflicting pad, unqualified net interval and error. No check is disabled.
struct TraceCounts {
  std::uint64_t dsp = 0, mcu = 0, pads = 0;
};

struct SenderState {
  std::optional<CpuState> interrupted;
  unsigned loaded, sent, received, initial_reads, returned;
  unsigned irq_entries, irq_returns, irq_queries, irq_unknown;
};

struct Saved {
  gpio_pair::PairSnapshot pair;
  kr::Saved peer;
  LcdFixture display;
  SenderState sender;
  TraceCounts trace_counts;
  unsigned samples, bytes, cursor, texts, text_returns;
  bool uploaded, buffered;
};

struct Scenario {
  unsigned order, phase, target_phase;
};

class TraceFilter final : public std::streambuf {
public:
  explicit TraceFilter(std::ostream &trace) : trace_(trace) {}

  [[nodiscard]] TraceCounts counts() const { return counts_; }

  void restore_counts(TraceCounts saved);

protected:
  std::streamsize xsputn(const char *data, std::streamsize size) override;
  int_type overflow(int_type value) override;
  int sync() override;

private:
  void emit();
  TraceCounts counts_;
  std::string line_;
  std::ostream &trace_;
};

struct Chain {
  const Firmware &firmware;
  std::ostream &trace;
  const Scenario scenario;
  TraceFilter filter;
  std::ostream detail;
  std::vector<std::uint8_t> helper;
  kh::Fixture sender;
  std::unique_ptr<kr::Fixture> peer;
  std::unique_ptr<LcdFixture> display;
  std::vector<unsigned> expected;
  unsigned samples = 4, bytes = 3, cursor = 0, texts = 0, text_returns = 0;
  bool uploaded = false, buffered = false;
  Chain(const Firmware &image,
        const std::vector<transport::Block> &blocks,
        const transport::Templates &templates,
        std::ostream &output,
        Scenario inputs);
  void connect();
  void prefix();
  void step();
  void received(std::uint8_t value);
  void until_buffer();
  void until_saving();
  void until_overlap();
  [[nodiscard]] Saved checkpoint() const;
  void restore(const Saved &saved);
  void compare_endpoint(const Chain &other) const;
  void guards();
  [[nodiscard]] unsigned source_byte(unsigned offset) const;
  [[nodiscard]] GpioPadSample irq_pads(std::uint32_t mask);
};
}
