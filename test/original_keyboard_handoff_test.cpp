// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "firmware/firmware.hpp"
#include "keyboard_handoff_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "original_update_forward_fixture.hpp"
#include "original_update_transport_fixture.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <string_view>

namespace {
using namespace blofeld;
using namespace blofeld::test::keyboard_handoff;

void verify(Fixture &run, const Firmware &firmware) {
  run.verify_copy();
  const auto image = run.pair.mcu.bus().power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "Native Keyboard copy/entry preserves every MCU NVM byte and erase/knowledge bit");
  for (const unsigned index : {2U, 3U}) {
    const auto plan = layouts.at(index);
    for (unsigned n = 0; n < plan.stored; ++n) {
      require(run.pair.dsp.bus().read(
                  {.space = index == 2 ? DspSpace::y : DspSpace::p, .address = plan.address + n}) ==
                  run.blocks.at(index).expected.at(n),
              "All original downloaded sender/payload/padding words remain unchanged");
    }
  }
  for (unsigned n = 0; n < fw::request_caller.size(); ++n) {
    require(run.pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) ==
                fw::request_caller[n],
            "Own relocated native caller remains outside the complete copied helper");
  }
  require(!run.pair.mcu.timing_faulted() && !run.pair.dsp.timing_faulted() &&
              run.pair.link.unresolved_queries == 0 && run.irq_unknown == 0 &&
              run.pair.mcu.bus().read_memory({.address = entry_sp - 13, .size = 2}) ==
                  fw::service_table,
          "Complete prefix retains its service frame and every consumed logical pad observation");
  bool rejected = false;
  try {
    run.pair.reject_board_qualification();
  } catch (const Fault &error) {
    rejected = std::string_view(error.what()).contains("Board qualification failed");
  }
  require(rejected,
          "Caller-qualified logical transfer must not qualify unresolved board intervals");
  run.pair.trace << "FINAL chunks=" << run.sent << " samples=" << run.received
                 << " helper-samples=" << run.initial_reads << " bytes=" << run.returned
                 << " loaded=" << run.loaded << " irq-entries=" << run.irq_entries
                 << " irq-returns=" << run.irq_returns << " queries=" << run.pair.link.pad_queries
                 << " unknown=" << run.pair.link.unresolved_queries
                 << " irq-unknown=" << run.irq_unknown
                 << " dsp-pc=" << run.pair.dsp.cpu().state().pc
                 << " mcu-pc=" << run.pair.mcu.cpu().state().pc
                 << " nvm=1 original-source=1 caller=1 board-rejected=1\n";
}
} // namespace

int main(const int argc, char **argv) {
  try {
    require(argc == 5,
            "Expected original main firmware, keyboard packets, own templates and trace");
    const auto firmware = Firmware::load(argv[1]);
    const auto blocks = transport::read_blocks(argv[2], layouts);
    const auto templates = transport::read_templates(argv[3]);
    std::ofstream trace(argv[4]);
    trace.exceptions(std::ios::badbit | std::ios::failbit);
    for (const unsigned order : {0U, 1U}) {
      for (const unsigned phase : {0U, gp::phase_offset}) {
        trace << "CASE order=" << order << " phase=" << phase << '\n';
        Fixture run(firmware, blocks, templates, trace, phase);
        run.execute(order, phase);
        verify(run, firmware);
      }
    }
    std::cout << "Four original Keyboard DSP/MCU copy and receiver handoffs verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
