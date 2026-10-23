// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_scheduled_run.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_cycles.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_dsp_schedule.hpp"
#include "gpio_parameter_caller.hpp"
#include "original_gpio_pair_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::gpio_pair;
constexpr unsigned parameter = 34, target = 0x742, initial = 42, incoming = 7;
constexpr unsigned sound_base = 0x936, first_type = 0x9b6, second_type = 0x9c6;
constexpr unsigned first_value = 3, second_value = 4, page = 0x1a42, first_result = 0x1a43;
constexpr unsigned result = 0x2500, tick = 0x182e, tuning = 0x2a, update_flags = 0x19;
constexpr unsigned update_bit = 0x1000, period = 8192, timer_control = 0x40, timer_enable = 0x80;
constexpr unsigned timer_vector = 0xfff0, timer_handler = 0x88a5, frame_bytes = 9;
constexpr unsigned wait_entry = 0xbbe1, wait_poll = 0xbbe4, wait_return = 0xbbe9, wait_delta = 8;
constexpr unsigned held_periods = 9, held_polls = 32, signature_address = 0x76c, signature = 127;
constexpr unsigned instruction_limit = 100000, byte_bits = 8;
constexpr unsigned primary_field = 128, secondary_field = 144, wrapping_tick = 254;
constexpr unsigned expected_data_writes = 17;
constexpr Case scenario{
    .kind = 2, .location = 0, .parameter = parameter, .address = target, .value = initial};

std::array<unsigned, 6> registers(const CpuState cpu) {
  return {cpu.pc, cpu.d, cpu.x, cpu.y, cpu.sp, cpu.ccr};
}

struct Run {
  OriginalGpioPair pair;
  unsigned initial_tick;
  bool enabled;
  unsigned produced = 0, consumed = 0, stores = 0, source_checks = 0;
  unsigned irq_entries = 0, irq_returns = 0, rises = 0, polls = 0, waits = 0, wait_returns = 0;
  std::optional<CpuState> interrupted;
  unsigned interrupted_tick = 0;
  std::uint64_t armed_osc = 0, cycle_checks = 0;
  bool previous_flag = false, got_result = false;

  Run(const Firmware &firmware, const unsigned starting_tick, const bool timer, std::ostream &trace)
      : pair(firmware, scenario, trace, DspSetup::secondary_parser),
        initial_tick(starting_tick),
        enabled(timer) {
    prepare();
    pair.additional_completion.observe = [this](const auto &step) { observe(step); };
  }

  unsigned read(const unsigned address) {
    return pair.dsp.bus().read({.space = DspSpace::x, .address = address});
  }

  [[nodiscard]] static std::array<Case, 3> sources() {
    return {
        {{.kind = 0,
          .location = 0,
          .parameter = primary_field,
          .address = first_type,
          .value = first_value},
         {.kind = 0,
          .location = 0,
          .parameter = secondary_field,
          .address = second_type,
          .value = second_value},
         {.kind = 2, .location = 0, .parameter = parameter, .address = target, .value = incoming}}};
  }

  void prepare() {
    for (const auto [address, value] :
         std::array<std::array<unsigned, 2>, 5>{{{part_table, sound_base},
                                                 {first_type, first_value},
                                                 {second_type, second_value},
                                                 {update_flags, 0},
                                                 {tuning, data_canary}}}) {
      pair.dsp.bus().write({.space = DspSpace::x, .address = address, .value = value});
    }
    auto &bus = pair.mcu.bus();
    bus.write(page, 0);
    bus.write(tick, static_cast<std::uint8_t>(initial_tick));
    require((bus.read(bus_flags) & 1U) != 0,
            "Declared scan inhibition retains original timer ticks without sampling panel inputs");
    bus.write(hc12::crg_flags, timer_enable);
    bus.write(hc12::rti_control, timer_control);
    bus.write(hc12::crg_interrupts, static_cast<std::uint8_t>(enabled ? timer_enable : 0));
    armed_osc = pair.mcu.run_clock().oscillator_cycles();
  }

  void observe(const DspScheduledStep &step) {
    const std::array<unsigned, 5> bytes{245, 96, parameter, incoming, 247};
    const auto producer = read(ring_write);
    if (producer != input_ring + produced) {
      require(produced < bytes.size() && producer == input_ring + produced + 1 &&
                  read(input_ring + produced) == bytes.at(produced) << 16U,
              "Original GPIO producer must publish the actual global-write bytes");
      ++produced;
    }
    const auto consumer = read(ring_read);
    if (consumer != input_ring + consumed) {
      require(consumed < produced && consumer == input_ring + consumed + 1,
              "Original secondary parser must consume its actual published prefix");
      ++consumed;
    }
    if (step.plan.interrupt) { return; }
    const auto &cpu = step.plan.before.cpu;
    if (read(target) != initial && stores == 0) {
      require(read(target) == incoming && cpu.r[1] == target &&
                  ((cpu.b >> word_bits) & word_mask) == incoming,
              "Original global store must use its independently expected R1/B1 and real memory");
      ++stores;
      pair.trace << "GLOBAL_STORE pc=" << hex(cpu.pc) << " address=" << target
                 << " value=" << incoming << '\n';
    }
    if (cpu.pc != source_load) { return; }
    const auto expected = sources();
    require(enabled && wait_returns == 1 && source_checks < expected.size() &&
                cpu.r[6] == expected.at(source_checks).address,
            "Original page refresh/readback must follow the complete real eight-tick wait");
    ++source_checks;
    if (source_checks < expected.size()) { pair.input = expected.at(source_checks); }
  }

  void clock_check() {
    const auto &clock = pair.mcu.run_clock();
    const auto oscillator_period = clock.timebase_hz() / clock.oscillator_hz();
    require(clock.oscillator_cycles() == clock.elapsed_ticks() / oscillator_period,
            "Original explicit clock epoch must agree with independent oscillator-edge division");
    const auto elapsed = clock.oscillator_cycles() - armed_osc;
    require(pair.mcu.bus().oscillator_cycles_until_rti() == period - (elapsed % period),
            "CRG divider must retain the independent literal8192-oscillator-edge phase");
    const bool flag = (pair.mcu.bus().read(hc12::crg_flags) & timer_enable) != 0;
    if (flag && !previous_flag) {
      ++rises;
      require(elapsed == (std::uint64_t{rises} * period),
              "Actual RTIF edge must occur at its independently expected oscillator timeout");
      pair.trace << "RTIF_RISE index=" << rises << " oscillator=" << elapsed << '\n';
    }
    previous_flag = flag;
    ++cycle_checks;
  }

  void install(const unsigned order, const std::vector<std::uint8_t> &code) {
    for (unsigned n = 0; n < code.size(); ++n) {
      pair.mcu.bus().write(static_cast<std::uint16_t>(caller + n), code[n]);
    }
    pair.input = sources().front();
    pair.start(order == 0 ? McuDspTieOrder::dsp_before_mcu : McuDspTieOrder::mcu_before_dsp,
               order == 0 ? 0 : phase_offset,
               caller + blofeld::test::gpio_parameter::caller_entry_bytes);
    pair.mcu.set_cycle_observer([this](const Hc12BusCycle cycle) {
      pair.on_cycle(cycle);
      clock_check();
    });
  }

  void check_frame() {
    if (!interrupted) { throw Fault("Original timer frame must have an observed foreground"); }
    const auto before = *interrupted;
    const auto entered = pair.mcu.cpu().state();
    require(entered.pc == timer_handler && entered.sp == before.sp - frame_bytes,
            "Actual native timer vector must push its complete original interrupt frame");
    const std::array<unsigned, frame_bytes> expected{before.ccr,
                                                     before.d & mask,
                                                     static_cast<unsigned>(before.d) >> byte_bits,
                                                     static_cast<unsigned>(before.x) >> byte_bits,
                                                     before.x & mask,
                                                     static_cast<unsigned>(before.y) >> byte_bits,
                                                     before.y & mask,
                                                     static_cast<unsigned>(before.pc) >> byte_bits,
                                                     before.pc & mask};
    for (unsigned n = 0; n < expected.size(); ++n) {
      require(pair.mcu.bus().read(static_cast<std::uint16_t>(entered.sp + n)) == expected[n],
              "Every actual IRQ stack byte must match the independent NXP frame order");
    }
  }

  void before_step() {
    const auto pending = pair.mcu.cpu().pending_interrupt();
    if (pending) {
      require(enabled && pending->vector == timer_vector && !interrupted,
              "Only the actual enabled CRG source may interrupt this declared context");
      interrupted = pair.mcu.cpu().state();
      interrupted_tick = pair.mcu.bus().read(tick);
      ++irq_entries;
      return;
    }
    const auto cpu = pair.mcu.cpu().state();
    if (cpu.pc == wait_entry) {
      require(waits == 0 && (cpu.d & mask) == wait_delta &&
                  pair.mcu.bus().read(tick) == initial_tick,
              "Original global writer must supply its literal eight-tick delay");
      ++waits;
      pair.trace << "WAIT_ENTRY tick=" << initial_tick << " delta=" << wait_delta << '\n';
    }
    if (cpu.pc == wait_poll) {
      require((cpu.d & mask) == ((initial_tick + wait_delta) & mask),
              "Native wait must retain its independently wrapped deadline across all IRQs");
      ++polls;
    }
    if (cpu.pc == wait_return) {
      require(enabled && wait_returns == 0 && irq_returns == wait_delta &&
                  pair.mcu.bus().read(tick) == ((initial_tick + wait_delta) & mask),
              "Original wait may return only after all eight genuine timer IRQs");
      ++wait_returns;
      pair.trace << "WAIT_RETURN tick=" << static_cast<unsigned>(pair.mcu.bus().read(tick))
                 << " interrupts=" << irq_returns << '\n';
    }
  }

  void step() {
    const bool entering = pair.mcu.cpu().pending_interrupt().has_value();
    before_step();
    pair.step();
    if (entering) { check_frame(); }
    if (interrupted && pair.mcu.cpu().state().pc == interrupted->pc &&
        pair.mcu.cpu().state().sp == interrupted->sp) {
      require(registers(pair.mcu.cpu().state()) == registers(*interrupted) &&
                  pair.mcu.bus().read(tick) == ((interrupted_tick + 1) & mask) &&
                  (pair.mcu.bus().read(hc12::crg_flags) & timer_enable) == 0,
              "Original timer INC/W1C/RTI must restore every register and advance the tick once");
      ++irq_returns;
      pair.trace << "IRQ_RETURN index=" << irq_returns
                 << " tick=" << static_cast<unsigned>(pair.mcu.bus().read(tick)) << '\n';
      interrupted.reset();
    }
    if (!got_result && pair.mcu.bus().read(result) != ram_canary) {
      require(enabled && pair.done() && pair.mcu.bus().read(result) == incoming &&
                  source_checks == sources().size(),
              "Actual original global readback must follow both real page refresh requests");
      got_result = true;
      pair.trace << "RESULT value=" << incoming << '\n';
    }
  }

  void execute() {
    for (unsigned n = 0; n < instruction_limit; ++n) {
      step();
      if (!enabled &&
          pair.mcu.run_clock().oscillator_cycles() - armed_osc >=
              std::uint64_t{held_periods} * period &&
          polls >= held_polls) {
        return;
      }
      if (enabled && got_result && pair.done() && !interrupted &&
          !pair.mcu.cpu().pending_interrupt() && pair.dsp.cpu().state().pc == pair.foreground &&
          pair.dsp.cpu().state().sp == 0) {
        return;
      }
    }
    throw Fault("Original clocked global wait exceeded native instruction budget");
  }

  void verify(const std::vector<std::uint8_t> &code) {
    auto &bus = pair.mcu.bus();
    require(stores == 1 && produced == 5 && consumed == produced && waits == 1 &&
                polls > held_polls && !interrupted && read(target) == incoming &&
                read(tuning) == 0 && read(update_flags) == update_bit &&
                read(signature_address) == signature && pair.link.unresolved_queries == 0,
            "Native global store/tuning/parser and independent clocked wait evidence differ");
    if (enabled) {
      require(irq_entries == wait_delta && irq_returns == wait_delta && rises == wait_delta &&
                  wait_returns == 1 && got_result && source_checks == sources().size() &&
                  bus.read(first_result) == first_value &&
                  bus.read(first_result + 1) == second_value &&
                  pair.link.data_writes == expected_data_writes,
              "Original global settle must complete8 real ticks, two page queries and readback");
    } else {
      require(
          irq_entries == 0 && irq_returns == 0 && rises == 1 && wait_returns == 0 && !got_result &&
              source_checks == 0 && bus.read(tick) == initial_tick &&
              bus.read(first_result) == ram_canary && bus.read(first_result + 1) == ram_canary &&
              pair.link.data_writes == 5 && bus.read(result) == ram_canary &&
              (bus.read(hc12::crg_flags) & timer_enable) != 0 &&
              pair.mcu.run_clock().oscillator_cycles() - armed_osc >=
                  std::uint64_t{held_periods} * period,
          "Disabled timer delivery must retain a genuinely unfinished wait despite elapsed time");
    }
    for (unsigned n = 0; n < code.size(); ++n) {
      require(bus.read(static_cast<std::uint16_t>(caller + n)) == code[n], "Own caller changed");
    }
    bool rejected = false;
    try {
      if (!got_result) { throw Fault("Board qualification failed: original call incomplete"); }
      pair.reject_board_qualification();
    } catch (const Fault &fault) {
      const std::string_view reason(fault.what());
      require(reason.contains(enabled ? "unresolved GPIO voltage" : "original call incomplete"),
              "The board rejection must identify the actually observed missing qualification");
      rejected = true;
    }
    require(rejected &&
                (enabled ? pair.link.unresolved_events > 0 : pair.link.unresolved_events == 0),
            "Unresolved turnaround or unfinished call must reject for its own established reason");
  }
};

int execute(const int argc, char **argv) {
  require(argc == 4, "Clocked global wait test needs archive/report/trace");
  const auto firmware = Firmware::load(argv[1]);
  constexpr std::array<unsigned, 1> values{incoming};
  const auto code = blofeld::test::gpio_parameter::caller_program(scenario, values, result);
  std::ofstream report(argv[2]);
  std::ofstream trace(argv[3]);
  require(report.is_open() && trace.is_open(), "Clocked wait outputs must open");
  report.exceptions(std::ios::badbit | std::ios::failbit);
  trace.exceptions(std::ios::badbit | std::ios::failbit);
  for (const bool enabled : {true, false}) {
    for (const unsigned starting_tick : {0U, wrapping_tick}) {
      if (!enabled && starting_tick == 0) { continue; }
      for (const unsigned order : {0U, 1U}) {
        trace << "CASE enabled=" << enabled << " tick=" << starting_tick << " order=" << order
              << '\n';
        Run run(firmware, starting_tick, enabled, trace);
        const auto nvm = run.pair.mcu.bus().power_off_flash_image();
        run.pair.verify_retained(firmware, nvm);
        run.install(order, code);
        run.execute();
        run.verify(code);
        run.pair.verify_retained(firmware, nvm);
        report << "CASE enabled=" << enabled << " tick=" << starting_tick << " order=" << order
               << " stores=" << run.stores << " waits=" << run.waits << " polls=" << run.polls
               << " irq-entries=" << run.irq_entries << " irq-returns=" << run.irq_returns
               << " flag-rises=" << run.rises << " wait-returns=" << run.wait_returns
               << " readbacks=" << run.got_result << " sources=" << run.source_checks
               << " clock-checks=" << run.cycle_checks
               << " elapsed-osc=" << run.pair.mcu.run_clock().oscillator_cycles() - run.armed_osc
               << " final-tick=" << static_cast<unsigned>(run.pair.mcu.bus().read(tick))
               << " queries=" << run.pair.link.pad_queries
               << " unknown-queries=" << run.pair.link.unresolved_queries
               << " unresolved-events=" << run.pair.link.unresolved_events
               << " net-rejected=" << enabled << " unfinished-rejected=" << !enabled
               << " program-unchanged=1 nvm-unchanged=1 caller-unchanged=1 board-rejected=1\n";
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
