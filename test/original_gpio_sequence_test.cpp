// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "dsp_norm_fixture.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_pair;
using blofeld::test::require;
constexpr unsigned request_count = 16;
constexpr unsigned caller_record_bytes = 17;
constexpr unsigned result_base = 0x2500;
constexpr unsigned signature_case = 7;
constexpr std::array<unsigned, 7> values{0, 1, 7, 63, 64, 126, 127};

constexpr Case request(const unsigned index) {
  const auto selected = index < cases.size() ? index : request_count - index - 1;
  auto result = cases.at(selected);
  if (selected < values.size()) { result.value = values.at(selected); }
  return result;
}

std::vector<std::uint8_t> caller_program() {
  std::vector<std::uint8_t> code;
  code.reserve((request_count * caller_record_bytes) + 2);
  for (unsigned n = 0; n < request_count; ++n) {
    const auto input = request(n);
    const auto result = result_base + n;
    // Own unrolled caller. S12CPU15UG v1.2: LDAB IMM, STAB IDX,
    // LDAA IMM, STAA IDX, LDD IMM, JSR EXT, STAB EXT. rr=10 selects SP.
    const std::array<std::uint8_t, caller_record_bytes> entry{
        0xc6,
        static_cast<std::uint8_t>(input.location),
        0x6b,
        0x80,
        0x86,
        static_cast<std::uint8_t>(input.kind),
        0x6a,
        0x81,
        0xcc,
        static_cast<std::uint8_t>(input.parameter >> 8U),
        static_cast<std::uint8_t>(input.parameter & mask),
        0x16,
        0xb6,
        0xdc,
        0x7b,
        static_cast<std::uint8_t>(result >> 8U),
        static_cast<std::uint8_t>(result & mask)};
    code.insert(code.end(), entry.begin(), entry.end());
  }
  code.push_back(branch_opcode);
  code.push_back(branch_self);
  return code;
}

void prepare(OriginalGpioPair &pair, const std::vector<std::uint8_t> &code) {
  for (unsigned n = 0; n < code.size(); ++n) {
    pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), code[n]);
  }
  // Declared own initial field values. The original signature word is never
  // replaced; it came from the whole unchanged original data record at setup.
  for (unsigned n = 0; n < signature_case; ++n) {
    const auto input = request(n);
    pair.dsp.bus().write({.space = DspSpace::x, .address = input.address, .value = input.value});
  }
  pair.input = request(0); // Oracle metadata only; the native caller writes all arguments.
}

void sequence(OriginalGpioPair &pair, std::ostream &trace) {
  unsigned completed = 0;
  for (unsigned n = 0; n < step_limit && completed < request_count; ++n) {
    pair.step();
    const auto result = pair.mcu.bus().read(static_cast<std::uint16_t>(result_base + completed));
    if (result == ram_canary) { continue; }
    require(result == request(completed).value &&
                pair.mcu.cpu().state().pc == caller + ((completed + 1) * caller_record_bytes) &&
                pair.link.source_register_checks == completed + 1 &&
                pair.link.data_writes == (completed + 1) * 4,
            "Only an executed caller store after the actual original return completes a request");
    trace << "RESULT index=" << completed << " value=" << static_cast<unsigned>(result)
          << " address=" << result_base + completed
          << " mcu-cycles=" << pair.mcu.run_clock().bus_cycles()
          << " dsp-cycles=" << pair.dsp.run_clock().native_cycle_units() << '\n';
    ++completed;
    if (completed < request_count) { pair.input = request(completed); }
  }
  require(completed == request_count, "All original sequential requests must return within bound");
  pair.finish();
}

void observe_caller(OriginalGpioPair &pair, std::ostream &trace) {
  pair.mcu.set_cycle_observer([&pair, &trace](const Hc12BusCycle cycle) {
    pair.on_cycle(cycle);
    const bool write = cycle.kind == 'w' || cycle.kind == 'W';
    if (write && cycle.address && *cycle.address >= result_base &&
        *cycle.address < result_base + request_count) {
      trace << "CALLER_SLOT pc=" << hex(cycle.context.pc) << " address=" << hex(*cycle.address)
            << " size=" << cycle.size << '\n';
    }
  });
}

void verify(OriginalGpioPair &pair, const std::vector<std::uint8_t> &code) {
  require(pair.link.source_register_checks == request_count &&
              pair.link.data_writes == request_count * 4 && pair.link.unresolved_queries == 0,
          "The continuous native stream must retain all source and known-read evidence");
  for (unsigned n = 0; n < code.size(); ++n) {
    require(pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) == code[n],
            "The native own caller program must remain unchanged");
  }
  for (unsigned n = 0; n < request_count; ++n) {
    const auto input = request(n);
    require(pair.dsp.bus().read({.space = DspSpace::x, .address = input.address}) == input.value &&
                pair.mcu.bus().read(static_cast<std::uint16_t>(result_base + n)) == input.value,
            "Actual native returns and retained own/original source values must agree");
  }
  const auto idle = pair.net();
  require(idle.known == mask && idle.levels == 0 && idle.conflicts == 0 &&
              pair.mcu.bus().gpio_outputs(McuPort::q).mask == 0,
          "Real original final cleanup must release the data bus to its native pull-downs");
  bool rejected = false;
  try {
    pair.reject_board_qualification();
  } catch (const Fault &) { rejected = true; }
  require(rejected,
          "A returned continuous sequence still cannot qualify unresolved board intervals");
}

int execute(const int argc, char **argv) {
  require(argc == 4, "Original GPIO sequence needs archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  const auto code = caller_program();
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Private sequence report/trace must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const unsigned order : {0U, 1U}) {
    for (const unsigned phase : {0U, phase_offset}) {
      trace << "CASE order=" << order << " phase=" << phase << '\n';
      OriginalGpioPair pair(firmware, cases[signature_case], trace);
      const auto nvm = pair.mcu.bus().power_off_flash_image();
      prepare(pair, code);
      pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
                 phase,
                 caller + (request_count * caller_record_bytes));
      observe_caller(pair, trace);
      sequence(pair, trace);
      verify(pair, code);
      pair.verify_retained(firmware, nvm);
      report << "CASE order=" << order << " phase=" << phase << " requests=" << request_count
             << " source-loads=" << pair.link.source_register_checks
             << " data-writes=" << pair.link.data_writes << " queries=" << pair.link.pad_queries
             << " unknown-queries=" << pair.link.unresolved_queries
             << " unresolved-events=" << pair.link.unresolved_events
             << " nvm-unchanged=1 program-unchanged=1 caller-unchanged=1 sources-unchanged=1"
                " board-rejected=1\n";
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
