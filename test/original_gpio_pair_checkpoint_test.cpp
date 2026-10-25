// SPDX-License-Identifier: GPL-3.0-only
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_gpio.hpp"
#include "original_gpio_pair_fixture.hpp"
#include "storage/flash_image.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <tuple>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_pair;
using blofeld::test::require;
constexpr unsigned signature_case = 7;
constexpr unsigned changed_canary = 0x5a;

std::string rejects(const auto &operation, const std::string_view expected) {
  try {
    operation();
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains(expected), "Wrong native pair recovery diagnostic: " + message);
    return message;
  }
  throw Fault("Missing native pair recovery rejection");
}

bool at_cut(const OriginalGpioPair &pair, const unsigned cut) {
  switch (cut) {
    case 0: return pair.link.data_writes == 2 && pair.link.source_register_checks == 0;
    case 1: return pair.link.data_writes >= 3 && pair.net().conflicts != 0;
    case 2: return pair.link.data_writes == 4 && pair.mcu.bus().gpio_outputs(McuPort::q).mask == 0;
    default: throw Fault("Invalid native pair checkpoint cut");
  }
}

void seek_cut(OriginalGpioPair &pair, const unsigned cut) {
  for (unsigned n = 0; n < step_limit; ++n) {
    pair.step();
    if (at_cut(pair, cut)) { return; }
    require(!pair.done(), "Missing original event-selected checkpoint boundary");
  }
  throw Fault("Original pair checkpoint boundary budget");
}

auto mcu_state(const CpuState cpu) {
  return std::tuple(cpu.pc, cpu.d, cpu.x, cpu.y, cpu.sp, cpu.ccr);
}

void accept_return(OriginalGpioPair &pair, const Firmware &firmware, const McuFlashImage &nvm) {
  pair.verify_retained(firmware, nvm);
  require(pair.link.source_register_checks == 1 && pair.link.unresolved_queries == 0 &&
              pair.link.pad_queries != 0 && pair.link.data_writes == 4,
          "Original snapshot continuation must retain independent source/known-read expectations");
  static_cast<void>(rejects([&] { pair.reject_board_qualification(); }, "qualification failed"));
}

void restore(OriginalGpioPair &pair, const PairSnapshot &saved) {
  pair.restore_owners(saved);
  static_cast<void>(
      rejects([&] { static_cast<void>(pair.mcu.bus().observe_pads(McuPort::q, 1)); }, "Detached"));
  static_cast<void>(rejects(
      [&] { static_cast<void>(pair.dsp.bus().gpio(DspPort::c).observe_pads(1)); }, "Detached"));
  require(pair.link.pad_queries == saved.link.pad_queries &&
              pair.mcu.bus().read(ram_end - 1) == ram_canary,
          "Detached sources must not invoke stale boards; actual snapshot RAM must be restored");
  pair.reconnect(saved);
}

void provoke_fault(OriginalGpioPair &pair,
                   const unsigned cut,
                   std::ostringstream &log,
                   std::ostream &trace) {
  pair.link.fault = cut == 0 ? PadFault::dsp : PadFault::mcu;
  log.str({});
  log.clear();
  const auto message = rejects([&] { pair.finish(); }, cut == 0 ? "Unknown DSP" : "Unknown MCU");
  require(message.contains("pc=") && pair.schedule->faulted() &&
              (pair.mcu.timing_faulted() || pair.dsp.timing_faulted()) &&
              pair.link.unresolved_queries != 0,
          "A real native unknown-pad consumer must quarantine the coupled execution");
  static_cast<void>(rejects([&] { static_cast<void>(pair.checkpoint()); }, "checkpoint"));
  trace << "FAULT_BEGIN\n" << log.str() << "DIAGNOSTIC " << message << "\nFAULT_END\n";
  pair.mcu.bus().write(ram_end - 1, changed_canary);
}

void run_case(const Firmware &firmware,
              const unsigned order,
              const unsigned phase,
              const unsigned cut,
              std::ostream &report,
              std::ostream &trace) {
  std::ostringstream log;
  OriginalGpioPair pair(firmware, cases[signature_case], log);
  const auto nvm = pair.mcu.bus().power_off_flash_image();
  pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp, phase);
  seek_cut(pair, cut);
  const auto saved = pair.checkpoint();
  trace << "CASE order=" << order << " phase=" << phase << " cut=" << cut << '\n';
  trace << "CAPTURE writes=" << saved.link.data_writes << " conflicts=" << pair.net().conflicts
        << " mcu-mask=" << static_cast<unsigned>(pair.mcu.bus().gpio_outputs(McuPort::q).mask)
        << " source-loads=" << saved.link.source_register_checks << '\n';
  log.str({});
  log.clear();
  pair.finish();
  accept_return(pair, firmware, nvm);
  const auto baseline = log.str();
  const auto final_mcu = mcu_state(pair.mcu.cpu().state());
  const auto final_dsp = pair.dsp.cpu().debug_state();
  const auto final_ticks = pair.schedule->end_ticks();
  const auto final_ties = pair.schedule->ties();
  trace << "BASELINE_BEGIN\n" << baseline << "BASELINE_END\n";
  restore(pair, saved);
  provoke_fault(pair, cut, log, trace);
  for (unsigned pass = 0; pass < 2; ++pass) {
    restore(pair, saved);
    log.str({});
    log.clear();
    pair.finish();
    accept_return(pair, firmware, nvm);
    require(log.str() == baseline && mcu_state(pair.mcu.cpu().state()) == final_mcu &&
                pair.dsp.cpu().debug_state() == final_dsp &&
                pair.schedule->end_ticks() == final_ticks && pair.schedule->ties() == final_ties,
            "Joint original continuation must replay exact events, native states and time");
    trace << "REPLAY_BEGIN pass=" << pass << '\n' << log.str() << "REPLAY_END\n";
  }
  report << "CASE order=" << order << " phase=" << phase << " cut=" << cut
         << " returns=3 restores=3 faults=1 detached=6 board-rejections=3"
            " signature=127 nvm-unchanged=1 program-unchanged=1\n";
}

int execute(const int argc, char **argv) {
  require(argc == 4, "Original pair checkpoint test needs archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Private native pair replay files must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const unsigned order : {0U, 1U}) {
    for (const unsigned phase : {0U, phase_offset}) {
      for (unsigned cut = 0; cut < 3; ++cut) {
        run_case(firmware, order, phase, cut, report, trace);
      }
    }
  }
  return 0;
}
}

int main(const int argc, char **argv) {
  try {
    return execute(argc, argv);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
