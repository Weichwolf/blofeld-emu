// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "board/gpio_pads.hpp"
#include "board/mcu_dsp_schedule.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pll.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_clock.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <ostream>
#include <string>
using blofeld::test::DspProgram;
using blofeld::test::number;
using blofeld::test::require;

namespace blofeld::test::gpio_pair {
constexpr unsigned mask = 255, stack = 0x2200, caller = 0x1400;

constexpr unsigned ram_begin = 0x1200;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned common_hz = 1080000000;
constexpr unsigned clock_record = 0x100;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned first_loopback = 0x8029;
constexpr unsigned second_loopback = 0x8036;
constexpr unsigned boot_inputs = 0xfb;
constexpr unsigned pctl = 0xfffffd;
constexpr unsigned phase_offset = 18;
constexpr unsigned setup_flags = 0x21;
constexpr unsigned ram_end = 0x2600;
constexpr unsigned ram_canary = 0xa5;
constexpr unsigned bus_flags = 0x1815;
constexpr unsigned mcu_x_seed = 0x5678;
constexpr unsigned mcu_y_seed = 0x9abc;
constexpr unsigned capture_entry = 0x2662;
constexpr unsigned program_alias = 0xf000;
constexpr unsigned input_ring = 0x2d00;
constexpr unsigned data_canary = 0x2468ac;
constexpr unsigned part_table = 0x2d80;
constexpr unsigned ring_write = 0x32;
constexpr unsigned ring_read = 0x33;
constexpr unsigned ring_read_initial = 0x2d40;
constexpr unsigned input_stamp = 0x1f;
constexpr unsigned a_seed = 0x13579b;
constexpr unsigned saved_request = 0x34;
constexpr unsigned request_seed = 0x135724;
constexpr unsigned deferred = 0x35;
constexpr unsigned x_seed = 0x123456;
constexpr unsigned saved_a = 0x36;
constexpr unsigned saved_x = 0x38;
constexpr unsigned source_value = 42;
constexpr unsigned memory_mode = 0x600085;
constexpr unsigned status_seed = 0xc00000;
constexpr unsigned r_seed = 0x321;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned absolute_jump = 0x0af080;
constexpr unsigned setup_limit = 100;
constexpr unsigned alignment_caller = 0x1500;
constexpr unsigned branch_opcode = 0x20;
constexpr unsigned branch_self = 0xfe;
constexpr unsigned alignment_ccr = 0xd0;
constexpr unsigned step_limit = 10000;
constexpr unsigned io_begin = 0x240;
constexpr unsigned io_end = 0x280;
constexpr unsigned port_c_control = 0xffffbf;
constexpr unsigned source_load = 0x269a;
constexpr unsigned reply_write = 0x26a1;
constexpr unsigned word_bits = 24;
constexpr unsigned flags_seed = 0x2200;
constexpr unsigned secondary_input_entry = 0xc5d;
constexpr unsigned original_setup_begin = 0x87f7;
constexpr unsigned original_setup_end = 0x8833;
constexpr unsigned original_setup_steps = 23;

struct Case {
  unsigned kind;
  unsigned location;
  unsigned parameter;
  unsigned address;
  unsigned value = source_value;
  bool original_data = false;
};

constexpr std::array<Case, 8> cases{
    {{.kind = 0, .location = 0, .parameter = 0, .address = ram_begin},
     {.kind = 1, .location = 0x40, .parameter = 0, .address = 0xe816},
     {.kind = 1, .location = 0, .parameter = 0, .address = 0xe836},
     {.kind = 2, .location = 0, .parameter = 0, .address = 0x720},
     {.kind = 0, .location = 15, .parameter = 511, .address = 0x1b7f},
     {.kind = 1, .location = 0x4f, .parameter = 63, .address = 0xe855},
     {.kind = 1, .location = 7, .parameter = 24, .address = 0xe8f6},
     {.kind = 2,
      .location = 0,
      .parameter = 0x4c,
      .address = 0x76c,
      .value = 127,
      .original_data = true}}};

struct ClockInputs {
  unsigned slow;
  unsigned fast;
  unsigned delay;
};

inline ClockInputs clock_inputs(const Firmware &firmware) {
  for (const auto &record : firmware.dsp) {
    if (record.space == 0 && record.address == clock_record) {
      require(record.words.size() > 10, "Original clock record must include both PLL operands");
      return {.slow = record.words.at(6), .fast = record.words.at(10), .delay = record.words.at(3)};
    }
  }
  throw Fault("Original runtime clock record absent");
}

enum class PadFault : std::uint8_t { none, mcu, dsp };
enum class DspSetup : std::uint8_t { request_only, secondary_parser };

struct LinkState {
  bool returned = false;
  unsigned msteps = 0, dsteps = 0, source_register_checks = 0;
  GpioNetDriver mcu_drive;
  GpioNetDriver dsp_drive;
  GpioPadSample last_net;
  unsigned unresolved_events = 0;
  unsigned pad_queries = 0;
  unsigned unresolved_queries = 0;
  unsigned data_writes = 0;
  PadFault fault = PadFault::none;
};

struct PairSnapshot {
  McuCheckpoint mcu;
  DspCheckpoint dsp;
  McuDspScheduleCheckpoint schedule;
  LinkState link;
};

struct OriginalGpioPair {
  McuMachine mcu;
  DspMachine dsp;
  std::unique_ptr<McuDspSchedule> schedule;
  Case input;
  DspSetup setup;
  ClockInputs clocks;
  unsigned foreground = 0;
  unsigned completion_pc = caller + 3;
  std::ostream &trace;
  LinkState link;
  DspCompletionObserver additional_completion;
  std::function<void(const BusEvent &)> additional_bus_write;

  OriginalGpioPair(const Firmware &firmware,
                   const Case scenario,
                   std::ostream &log,
                   const DspSetup configuration = DspSetup::request_only)
      : mcu(McuFlashImage::from_known(firmware.mcu),
            BLOFELD_DEFAULT_SLA,
            McuRunContract{.clock = {.oscillator_hz = oscillator_hz,
                                     .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                           .memory_timing = McuRunMemory::internal_flash_reads}),
        dsp(DspRunContract{
            .clock = {.extal_hz = oscillator_hz,
                      .timebase_hz = common_hz,
                      .switch_timing = DspClockSwitch::reference_instruction_boundary},
            .timing = DspRunTiming::native_internal_estimates,
            .interrupts = DspInterruptTiming::native_vector_estimates}),
        input(scenario),
        setup(configuration),
        clocks(clock_inputs(firmware)),
        trace(log) {
    dsp.bus().pll().observe_pinit(false);
    dsp.cpu().reset_spi_boot();
    mcu_clock_prefix();
    original_pins();
    prepare_mcu();
    install(firmware);
    prepare_dsp_data(firmware);
    prepare_dsp_prefix();
    align_prefix();
    mcu.bus().set_pin_drive(McuPort::q, {});
    connect_pads();
    sync();
  }

  void mcu_clock_prefix() {
    auto &bus = mcu.bus();
    bus.set_pin_drive(McuPort::e, {.mask = boot_inputs, .levels = 3});
    bool observed = false;
    unsigned steps = 0;
    while (bus.run_clock_route().source != McuClockSource::usb_phy) {
      require(steps++ < setup_limit, "Original MCU clock prefix bound");
      const auto pc = mcu.cpu().state().pc;
      if (pc == first_loopback || pc == second_loopback) {
        const auto sample = bus.observe_pads(McuPort::e, 4);
        require(sample.known == 4, "Declared loopback source must be known");
        bus.set_pin_drive(
            McuPort::e,
            {.mask = boot_inputs,
             .levels = static_cast<std::uint8_t>(3U | ((sample.levels & 4U) != 0 ? 8U : 0U))});
      }
      if (pc == phy_poll && !observed) {
        bus.observe_phy({.edges = 15});
        observed = true;
      }
      mcu.cpu().step();
    }
    require(observed, "Original boot must consume the declared PHY observation");
    trace << "MCU_CLOCK instructions=" << steps << " cycles=" << mcu.run_clock().bus_cycles()
          << " ticks=" << mcu.run_clock().elapsed_ticks()
          << " divider=" << bus.run_clock_route().bus_divider << '\n';
  }

  void original_pins() {
    mcu.cpu().set_state(
        {.pc = original_setup_begin, .sp = stack, .ccr = ExecutionContext::initial_ccr});
    unsigned steps = 0;
    while (mcu.cpu().state().pc != original_setup_end) {
      require(steps++ < setup_limit, "Original native GPIO setup bound");
      mcu.cpu().step();
    }
    require(steps == original_setup_steps && mcu.bus().read(hc12::rdrq) == mask &&
                mcu.bus().read(hc12::perq) == mask && mcu.bus().read(hc12::ppsq) == mask,
            "Original Q reduced output and passive input pull-down setup");
  }

  void prepare_mcu() {
    auto &bus = mcu.bus();
    bus.write(hc12::initee, setup_flags);
    bus.retire_instruction();
    for (unsigned address = ram_begin; address < ram_end; ++address) {
      bus.write(static_cast<std::uint16_t>(address), ram_canary);
    }
    bus.write(hc12::ptj, 1);
    bus.write(hc12::ddrj, 7);
    bus.write(hc12::ptt, 1);
    bus.write(hc12::ddrt, 15);
    bus.write(hc12::ptq, 0);
    bus.set_pin_drive(McuPort::e, {.mask = 3, .levels = 3});
    bus.set_pin_drive(McuPort::q, {.floating = mask});
    bus.write(bus_flags, setup_flags);
    bus.write(static_cast<std::uint16_t>(stack), static_cast<std::uint8_t>(input.location));
    bus.write(static_cast<std::uint16_t>(stack + 1), static_cast<std::uint8_t>(input.kind));
    constexpr std::array<std::uint8_t, 3> code{0x16, 0xb6, 0xdc};
    for (unsigned n = 0; n < code.size(); ++n) {
      bus.write(static_cast<std::uint16_t>(caller + n), code[n]);
    }
    mcu.cpu().set_state({.pc = caller,
                         .d = static_cast<std::uint16_t>(input.parameter),
                         .x = mcu_x_seed,
                         .y = mcu_y_seed,
                         .sp = stack,
                         .ccr = 0x40});
  }

  void install(const Firmware &firmware) {
    for (const auto &record : firmware.dsp) {
      if (setup == DspSetup::request_only &&
          (record.space != 0 || (record.address != 0x10 && record.address != capture_entry))) {
        continue;
      }
      for (unsigned n = 0; n < record.words.size(); ++n) {
        const auto address = record.address + n;
        const bool mapped = record.space == 0 && address >= 0x1000;
        dsp.bus().write({.space = mapped ? DspSpace::x : static_cast<DspSpace>(record.space),
                         .address = mapped ? address + program_alias : address,
                         .value = record.words[n]});
      }
    }
  }

  void prepare_dsp_data(const Firmware &firmware) {
    auto put = [&](const unsigned address, const unsigned value) {
      dsp.bus().write({.space = DspSpace::x, .address = address, .value = value});
    };
    for (unsigned n = 0; n < 128; ++n) { put(input_ring + n, data_canary ^ n); }
    for (unsigned n = 0; n < 16; ++n) { put(part_table + n, ram_begin + (n * 128)); }
    put(ring_write, input_ring);
    put(ring_read, setup == DspSetup::secondary_parser ? input_ring : ring_read_initial);
    put(input_stamp, a_seed);
    put(6, setup == DspSetup::secondary_parser ? 0 : flags_seed);
    put(saved_request, request_seed);
    put(deferred, x_seed);
    for (unsigned address = saved_a; address <= saved_x; ++address) { put(address, data_canary); }
    if (!input.original_data) {
      put(input.address, input.value);
      return;
    }
    unsigned loaded = 0;
    for (const auto &record : firmware.dsp) {
      if (record.space != 1 || input.address < record.address ||
          input.address - record.address >= record.words.size()) {
        continue;
      }
      require(record.words.at(input.address - record.address) == input.value,
              "Original data record must contain the expected signature");
      for (unsigned n = 0; n < record.words.size(); ++n) {
        put(record.address + n, record.words[n]);
      }
      ++loaded;
    }
    require(loaded == 1, "Load the entire unique original signature data record, not a host reply");
  }

  void prepare_dsp_prefix() {
    dsp.bus().observe_irqa(true);
    DspProgram program;
    program.immediate("omr", memory_mode);
    for (unsigned n = 0; n < 3; ++n) { program.emit("nop"); }
    const auto mapped = program.pc();
    program.immediate("x0", clocks.delay);
    program.emit("movep #>" + number(clocks.slow) + ",x:<<" + number(pctl));
    program.emit("rep x0");
    const auto delay_nop = program.pc();
    program.emit("nop");
    program.emit("movep #>" + number(clocks.fast) + ",x:<<" + number(pctl));
    program.immediate("sr", status_seed);
    program.immediate("a0", 0);
    program.immediate("a1", a_seed);
    program.immediate("a2", 0);
    program.immediate("x0", x_seed);
    program.immediate("r6", r_seed);
    for (unsigned n = 0; n < 8; ++n) { program.immediate("m" + std::to_string(n), word_mask); }
    program.emit("movep #>$f00,x:<<$ffffbe");
    program.emit("movep #>0,x:<<$ffffbd");
    program.emit("movep #>$ff,x:<<$ffffbf");
    program.emit("movep #>$4006,x:<<$ffffff");
    foreground = program.pc();
    if (setup == DspSetup::secondary_parser) {
      program.immediate("r6", secondary_input_entry);
      program.emit("jsr (r6)");
    }
    program.word(absolute_jump);
    program.word(foreground);
    program.install(dsp.bus());
    unsigned delay_iterations = 0;
    for (unsigned n = 0; dsp.cpu().state().pc != foreground; ++n) {
      require(n < clocks.delay + setup_limit,
              "Own prefix includes the complete firmware-derived delay");
      if (dsp.cpu().state().pc == delay_nop) { ++delay_iterations; }
      if (dsp.cpu().state().pc == mapped) { dsp.bus().advance_instruction_cycles(3); }
      dsp.cpu().step();
    }
    require(delay_iterations == clocks.delay && dsp.bus().pll().read() == clocks.fast,
            "Own native setup retains every firmware-derived delay iteration and final PLL state");
    const auto ratio = dsp.bus().pll().target_ratio();
    trace << "DSP_CLOCK delay=" << delay_iterations << " pctl=" << dsp.bus().pll().read()
          << " ratio-numerator=" << ratio.numerator << " ratio-denominator=" << ratio.denominator
          << " timebase=" << dsp.run_clock().timebase_hz() << '\n';
  }

  void align_prefix(const unsigned limit = step_limit) {
    auto &bus = mcu.bus();
    const auto original_context = mcu.cpu().state();
    bus.write(alignment_caller, branch_opcode);
    bus.write(alignment_caller + 1, branch_self);
    mcu.cpu().set_state({.pc = alignment_caller, .sp = stack, .ccr = alignment_ccr});
    unsigned padding = 0;
    require(common_hz % mcu.run_clock().timebase_hz() == 0, "Integral common clock conversion");
    const auto ratio = common_hz / mcu.run_clock().timebase_hz();
    while (mcu.run_clock().elapsed_ticks() * ratio < dsp.run_clock().elapsed_ticks()) {
      require(++padding < limit, "explicit own prefix clock alignment");
      mcu.cpu().step();
    }
    mcu.cpu().set_state(original_context);
    sync();
  }

  [[nodiscard]] GpioPadSample net() const {
    return resolve_gpio_net(mask, link.mcu_drive, link.dsp_drive);
  }

  GpioPadSample pads(const bool dsp_side, const unsigned bits) {
    const auto observation = net();
    ++link.pad_queries;
    const auto remote = dsp.bus().gpio(DspPort::c).outputs();
    const auto high = dsp_side ? bits & ~mask : 0U;
    auto known = (observation.known | (remote.known & high)) & bits;
    if ((link.fault == PadFault::mcu && !dsp_side) || (link.fault == PadFault::dsp && dsp_side)) {
      known = 0;
    }
    const auto levels = (observation.levels | (remote.levels & high)) & known;
    const auto conflicts = observation.conflicts & bits;
    link.unresolved_queries += static_cast<unsigned>(known != bits);
    trace << "PAD_QUERY dsp=" << dsp_side << " requested=" << hex(bits) << " known=" << hex(known)
          << " levels=" << hex(levels) << " conflicts=" << hex(conflicts) << '\n';
    return {.known = known, .levels = levels, .conflicts = conflicts};
  }

  void sync() {
    const auto local = mcu.bus().gpio_outputs(McuPort::q);
    const auto bias = mcu.bus().gpio_biases(McuPort::q);
    const auto remote = dsp.bus().gpio(DspPort::c).outputs();
    link.mcu_drive = {.active = local.mask,
                      .known = local.known,
                      .levels = local.levels,
                      .reduced = local.reduced_drive,
                      .pull_up = bias.up,
                      .pull_down = bias.down};
    link.dsp_drive = {.active = (remote.mask | remote.peripheral) & mask,
                      .known = remote.known & mask,
                      .levels = remote.levels & mask,
                      .pull_down = mask & ~(remote.mask | remote.peripheral)};
    const auto observation = net();
    if (observation.known != mask && (observation.known != link.last_net.known ||
                                      observation.conflicts != link.last_net.conflicts ||
                                      observation.levels != link.last_net.levels)) {
      ++link.unresolved_events;
      trace << "NET_UNQUALIFIED mcu-pc=" << hex(mcu.cpu().state().pc)
            << " dsp-pc=" << hex(dsp.cpu().state().pc) << " known=" << hex(observation.known)
            << " levels=" << hex(observation.levels) << " conflicts=" << hex(observation.conflicts)
            << " mcu-reduced=" << hex(local.reduced_drive) << " mcu-levels=" << hex(local.levels)
            << " dsp-peripheral=" << hex(remote.peripheral) << " dsp-known=" << hex(remote.known)
            << " dsp-levels=" << hex(remote.levels) << '\n';
    }
    link.last_net = observation;
    const auto irq = mcu.bus().gpio_outputs(McuPort::j);
    const auto irq_bias = mcu.bus().gpio_biases(McuPort::j);
    dsp.bus().observe_irqa_driver({.active = irq.mask & 1U,
                                   .known = irq.known & 1U,
                                   .levels = irq.levels & 1U,
                                   .reduced = irq.reduced_drive & 1U,
                                   .pull_up = irq_bias.up & 1U,
                                   .pull_down = irq_bias.down & 1U});
  }

  void reject_board_qualification() const {
    if (link.unresolved_events != 0) {
      throw Fault("Board qualification failed: unresolved GPIO voltage intervals=" +
                  std::to_string(link.unresolved_events));
    }
  }

  void verify_retained(const Firmware &firmware, const McuFlashImage &before) {
    const auto after = mcu.bus().power_off_flash_image();
    require(before.bytes == after.bytes && before.known == after.known &&
                before.fresh == after.fresh && !mcu.timing_faulted() && !dsp.timing_faulted(),
            "Original request must preserve NVM and healthy native execution");
    std::map<std::uint32_t, std::uint32_t> expected_program;
    for (const auto &record : firmware.dsp) {
      if (record.space != 0 || (setup == DspSetup::request_only && record.address != 0x10 &&
                                record.address != capture_entry)) {
        continue;
      }
      for (unsigned n = 0; n < record.words.size(); ++n) {
        expected_program[record.address + n] = record.words[n];
      }
    }
    // Later original records replace early vectors. Verify the final ordered
    // load image, rather than demanding incompatible historical words together.
    for (const auto &[address, word] : expected_program) {
      require(dsp.bus().read({.space = DspSpace::p, .address = address}) == word,
              "Original final program image changed at P:" + hex(address, 6));
    }
    require(dsp.bus().read({.space = DspSpace::x, .address = input.address}) == input.value,
            "Original request must preserve its source data");
  }

  [[nodiscard]] bool done() const { return mcu.cpu().state().pc == completion_pc; }

  void start(const McuDspTieOrder order, const unsigned phase, const unsigned end = caller + 3) {
    completion_pc = end;
    schedule = std::make_unique<McuDspSchedule>(
        McuDspScheduleInputs{
            .dsp = {.epoch_ticks = phase,
                    .observation = DspStepObservation::reference_completion_boundary,
                    .irq_admission = DspIrqAdmission::reference_prepare_boundary},
            .shared_write_order = order,
            .maximum_dsp_steps_per_cycle = 16},
        mcu,
        dsp);
    if (end == caller + 3) {
      mcu.bus().write(caller + 3, branch_opcode);
      mcu.bus().write(caller + 4, branch_self);
    }
    attach_observers();
  }

  void run(const McuDspTieOrder order, const unsigned phase) {
    start(order, phase);
    finish();
  }

  // Pure register observations; these callbacks never manufacture a bus response.
  void before_step(const DspScheduledStep &step) {
    if (step.plan.before.cpu.pc != reply_write) { return; }
    const auto intended = (step.plan.before.cpu.a >> word_bits) & word_mask;
    trace << "REPLY_ATTEMPT a1=" << hex(intended)
          << " mcu-mask=" << hex(mcu.bus().gpio_outputs(McuPort::q).mask)
          << " mcu-level=" << hex(mcu.bus().gpio_outputs(McuPort::q).levels) << '\n';
  }

  DspCompletionObserver completed() {
    return {.observe = [this](const DspScheduledStep &step) {
      ++link.dsteps;
      if (step.plan.before.cpu.pc == source_load) {
        const auto after = dsp.cpu().state();
        require(step.plan.before.cpu.r[6] == input.address && (after.x & word_mask) == input.value,
                "Actual native source-load register effect differs from the own fixture");
        ++link.source_register_checks;
        trace << "SOURCE_REGISTER address=" << hex(input.address)
              << " x0=" << hex(after.x & word_mask) << '\n';
      }
      trace << "D " << step.completion_ticks << ' ' << hex(step.plan.before.cpu.pc, 6) << '\n';
      sync();
      if (additional_completion.observe) { additional_completion.observe(step); }
    }};
  }

  void on_cycle(const Hc12BusCycle cycle) {
    const bool write = cycle.kind == 'w' || cycle.kind == 'W';
    const bool shared =
        write && cycle.address && *cycle.address >= io_begin && *cycle.address < io_end;
    schedule->begin_cycle(
        cycle, shared, [this](const auto &step) { before_step(step); }, completed());
  }

  void attach_observers() {
    mcu.set_cycle_observer([this](const Hc12BusCycle cycle) { on_cycle(cycle); });
    mcu.bus().set_observer([this](const BusEvent &event) {
      if (!event.write) { return; }
      if (additional_bus_write) { additional_bus_write(event); }
      if (event.address == hc12::ptq) { ++link.data_writes; }
      trace << "M-W " << mcu.run_clock().elapsed_ticks() << ' ' << hex(mcu.cpu().state().pc) << ' '
            << hex(event.address) << ' ' << hex(event.value) << '\n';
      if (event.address >= io_begin && event.address < io_end) {
        schedule->observe_write(
            event,
            [this] { sync(); },
            [this](const auto &step) { before_step(step); },
            completed());
      }
    });
  }

  void step() {
    trace << "M " << mcu.run_clock().elapsed_ticks() << ' ' << hex(mcu.cpu().state().pc) << '\n';
    mcu.cpu().step();
    ++link.msteps;
    schedule->end_instruction();
    if (done()) { link.returned = true; }
  }

  void finish() {
    for (unsigned count = 0; count < step_limit; ++count) {
      step();
      if (link.returned && dsp.cpu().state().pc == foreground && dsp.cpu().state().sp == 0) {
        require((mcu.cpu().state().d & mask) == input.value,
                "independent original MCU reply value");
        require(dsp.bus().read({.space = DspSpace::x, .address = port_c_control}) == mask,
                "native cleanup input mode");
        mcu.set_cycle_observer({});
        mcu.bus().set_observer({});
        return;
      }
    }
    throw Fault("clocked pair instruction budget");
  }

  void connect_mcu_pads() {
    mcu.bus().attach_gpio_pad_source(
        McuPort::q, [this](const std::uint32_t bits) { return pads(false, bits); });
  }

  void connect_pads() {
    connect_mcu_pads();
    dsp.bus().gpio(DspPort::c).attach_pad_source([this](const std::uint32_t bits) {
      return pads(true, bits);
    });
  }

  [[nodiscard]] PairSnapshot checkpoint() const {
    return {.mcu = mcu.checkpoint(),
            .dsp = dsp.checkpoint(),
            .schedule = schedule->checkpoint(),
            .link = link};
  }

  void restore_owners(const PairSnapshot &saved) {
    mcu.restore(saved.mcu);
    dsp.restore(saved.dsp);
    link = saved.link;
  }

  void reconnect(const PairSnapshot &saved) {
    connect_pads();
    schedule->restore(saved.schedule);
    attach_observers();
  }
};

} // namespace blofeld::test::gpio_pair
