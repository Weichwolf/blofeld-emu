// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "dsp_norm_fixture.hpp"
#include "gpio_parameter_caller.hpp"
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
constexpr unsigned parameter = 2;
constexpr unsigned location = 0x40;
constexpr unsigned kind = 1;
constexpr unsigned target = 0xe818;
constexpr unsigned initial_value = 42;
constexpr unsigned result_base = 0x2500;
constexpr unsigned caller_bytes = 35;
constexpr unsigned parameter_store = 0x8d9;
constexpr unsigned signature_address = 0x76c;
constexpr unsigned signature_value = 127;
constexpr unsigned frame_bytes = 5;
constexpr std::array<unsigned, 8> values{0, 1, 7, 63, 64, 126, 127, 42};
constexpr Case scenario{.kind = kind,
                        .location = location,
                        .parameter = parameter,
                        .address = target,
                        .value = initial_value};

std::vector<unsigned> messages() {
  std::vector<unsigned> bytes;
  bytes.reserve(values.size() * frame_bytes);
  for (const auto value : values) {
    for (const auto byte : {0xf5U, location, location | parameter, value, 0xf7U}) {
      bytes.push_back(byte);
    }
  }
  return bytes;
}

std::vector<std::uint8_t> caller_program() {
  return blofeld::test::gpio_parameter::caller_program(scenario, values, result_base);
}

struct Run {
  OriginalGpioPair pair;
  const std::vector<unsigned> bytes = messages();
  unsigned produced = 0, consumed = 0, writes = 0;

  Run(const Firmware &firmware, std::ostream &trace)
      : pair(firmware, scenario, trace, DspSetup::secondary_parser) {
    pair.additional_completion.observe = [this](const DspScheduledStep &step) { observe(step); };
  }

  unsigned read(const unsigned address) {
    return pair.dsp.bus().read({.space = DspSpace::x, .address = address});
  }

  void observe(const DspScheduledStep &step) {
    const auto producer = read(ring_write);
    if (producer != input_ring + produced) {
      require(produced < bytes.size() && producer == input_ring + produced + 1 &&
                  read(input_ring + produced) == bytes.at(produced) << 16U,
              "Every actual native GPIO publication must retain the next original sender byte");
      pair.trace << "QUEUE_BYTE index=" << produced << " byte=" << bytes.at(produced)
                 << " cursor=" << producer << '\n';
      ++produced;
    }
    const auto consumer = read(ring_read);
    if (consumer != input_ring + consumed) {
      require(consumed < produced && consumer == input_ring + consumed + 1,
              "Original secondary parser must consume each received byte once");
      ++consumed;
      pair.trace << "CONSUMED count=" << consumed << " cursor=" << consumer << '\n';
    }
    if (step.plan.interrupt || step.plan.before.cpu.pc != parameter_store) { return; }
    const auto value = (step.plan.before.cpu.a >> word_bits) & word_mask;
    require(writes < values.size() && step.plan.before.cpu.r[2] == parameter &&
                value == values.at(writes) && read(target) == value &&
                pair.dsp.cpu().state().pc == parameter_store + 2,
            "Actual original parameter store must resolve the independent target/value");
    pair.trace << "PARAMETER_STORE index=" << writes << " pc=" << hex(parameter_store)
               << " address=" << hex(target) << " value=" << value << '\n';
    ++writes;
  }

  void install_caller(const std::vector<std::uint8_t> &code) {
    for (unsigned n = 0; n < code.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), code[n]);
    }
    pair.input.value = values.front(); // Oracle only; actual writes come from original parser.
  }

  void caller_slots() {
    pair.mcu.set_cycle_observer([this](const Hc12BusCycle cycle) {
      pair.on_cycle(cycle);
      const bool write = cycle.kind == 'w' || cycle.kind == 'W';
      if (write && cycle.address && *cycle.address >= result_base &&
          *cycle.address < result_base + values.size()) {
        pair.trace << "CALLER_SLOT pc=" << hex(cycle.context.pc)
                   << " address=" << hex(*cycle.address) << " size=" << cycle.size << '\n';
      }
    });
  }

  void execute() {
    unsigned completed = 0;
    for (unsigned n = 0; n < step_limit && completed < values.size(); ++n) {
      pair.step();
      const auto result = pair.mcu.bus().read(static_cast<std::uint16_t>(result_base + completed));
      if (result == ram_canary) { continue; }
      require(result == values.at(completed) && writes == completed + 1 &&
                  pair.link.source_register_checks == completed + 1 &&
                  pair.link.data_writes == (completed + 1) * 9 &&
                  pair.mcu.cpu().state().pc == caller + ((completed + 1) * caller_bytes),
              "A real original read must follow its actual parsed write on the continuous cores");
      pair.trace << "RESULT index=" << completed << " value=" << static_cast<unsigned>(result)
                 << " address=" << result_base + completed
                 << " mcu-cycles=" << pair.mcu.run_clock().bus_cycles()
                 << " dsp-cycles=" << pair.dsp.run_clock().native_cycle_units() << '\n';
      ++completed;
      if (completed < values.size()) { pair.input.value = values.at(completed); }
    }
    require(completed == values.size(), "All original write/read pairs must complete within bound");
    pair.finish();
  }

  void verify(const std::vector<std::uint8_t> &code) {
    require(writes == values.size() && produced == bytes.size() && consumed == produced &&
                pair.link.unresolved_queries == 0 && read(target) == initial_value &&
                read(signature_address) == signature_value,
            "Actual parser/write/read chain must finish without host source repair");
    for (unsigned n = 0; n < 128; ++n) {
      const auto expected = n < bytes.size() ? bytes.at(n) << 16U : data_canary ^ n;
      require(read(input_ring + n) == expected,
              "Original ring writes must preserve every other cell");
    }
    for (unsigned n = 0; n < code.size(); ++n) {
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(caller + n)) == code[n],
              "Own original-callee caller code must remain unchanged");
    }
    bool rejected = false;
    try {
      pair.reject_board_qualification();
    } catch (const Fault &) { rejected = true; }
    require(rejected,
            "A successful original write/read chain cannot qualify unknown board voltage");
  }
};

int execute(const int argc, char **argv) {
  require(argc == 4, "Original GPIO parameter test needs archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  const auto code = caller_program();
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Private parameter report/trace must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const unsigned order : {0U, 1U}) {
    for (const unsigned phase : {0U, phase_offset}) {
      trace << "CASE order=" << order << " phase=" << phase << '\n';
      Run run(firmware, trace);
      const auto nvm = run.pair.mcu.bus().power_off_flash_image();
      run.pair.verify_retained(firmware, nvm);
      run.install_caller(code);
      run.pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
                     phase,
                     caller + static_cast<unsigned>(values.size() * caller_bytes));
      run.caller_slots();
      run.execute();
      run.verify(code);
      run.pair.verify_retained(firmware, nvm);
      report << "CASE order=" << order << " phase=" << phase << " writes=" << run.writes
             << " readbacks=" << run.pair.link.source_register_checks
             << " produced=" << run.produced << " consumed=" << run.consumed
             << " data-writes=" << run.pair.link.data_writes
             << " queries=" << run.pair.link.pad_queries
             << " unresolved-queries=" << run.pair.link.unresolved_queries
             << " unresolved-events=" << run.pair.link.unresolved_events
             << " program-unchanged=1 nvm-unchanged=1 caller-unchanged=1 ring-retained=1"
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
