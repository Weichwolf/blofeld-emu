// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_scheduled_run.hpp"
#include "firmware/firmware.hpp"
#include "lcd_fixture.hpp"
#include "mcu/hc12.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <vector>

namespace blofeld::test::keyboard_initial {
namespace gp = gpio_pair;
namespace transport = update_transport;
constexpr unsigned target = 0x1200, helper_bytes = 119, source = 0xef00, source_words = 41;
constexpr unsigned caller = 0x2000, main_record = 44, main_patch = 0x19a, scratch = 0x660;
constexpr unsigned source_read = 0x2e21, final_ack = 0x2e38, sender_rts = 0x2e3e;
constexpr unsigned entry_sp = 0x21f4, byte_mask = 255, data_mask = 63, word_bits = 24;
constexpr unsigned high_group = 18, extension_shift = 48, irq_strobe = 0x200;
constexpr unsigned service_table = 0xf81b, helper_return = 0xbb56, instruction_limit = 200000;
constexpr unsigned clear_entry = 0x8d70, text_entry = 0xa6ee, rectangle_entry = 0x8bfd;
constexpr unsigned helper_rts = 0x1254, indicator_entry = 0x1255;
constexpr unsigned stream_return = 0xbacf, trailer_begin = 0xbb65, trailer_end = 0xbb75;
constexpr unsigned final_flags = 0xc0, lcd_writes = 1559, lcd_reads = 1070;
constexpr std::array<unsigned, 4> chunk_pcs{0x2e26, 0x2e2b, 0x2e30, 0x2e35};
constexpr std::array layouts{
    transport::Layout{.mode = 3, .address = source, .declared = 42, .stored = 42, .packets = 6},
    transport::Layout{.mode = 0, .address = 0x2e00, .declared = 63, .stored = 63, .packets = 9},
    transport::Layout{
        .mode = 3, .address = source, .declared = 1542, .stored = 1542, .packets = 221},
    transport::Layout{.mode = 0, .address = 0x2e00, .declared = 57, .stored = 58, .packets = 9}};

struct Fixture {
  gp::OriginalGpioPair pair;
  const Firmware &firmware;
  const std::vector<transport::Block> &blocks;
  const unsigned branch;
  std::vector<unsigned> own_dsp;
  std::vector<std::uint8_t> own_mcu;
  std::unique_ptr<LcdFixture> display;
  std::optional<CpuState> interrupted;
  unsigned returned = 0, loaded = 0, sent = 0, received = 0, irq_entries = 0, irq_returns = 0;
  unsigned irq_queries = 0, irq_unknown = 0, acknowledgements = 0, sender_returns = 0;
  unsigned helper_returns = 0, indicators = 0, texts = 0, rectangles = 0;
  unsigned trailer_bytes = 0;

  Fixture(const Firmware &image,
          const std::vector<transport::Block> &input,
          const transport::Templates &templates,
          unsigned expected_branch,
          std::ostream &trace,
          unsigned phase);
  void align(unsigned phase);
  void connect();
  void attach_irq();
  void observe(const DspScheduledStep &step);
  void step();
  void drawing();
  void verify_copy();
  void execute(unsigned order, unsigned phase);
  void finish();
  void replay();
  void retain();
  [[nodiscard]] unsigned chunk(unsigned index) const;
};
} // namespace blofeld::test::keyboard_initial
