// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "dsp56kEmu/assembler.h"
#include "nand_native_peer.hpp"
#include "nand_native_tests.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace blofeld::test {
namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000;
constexpr unsigned extal = 12000000;
constexpr unsigned period = 90;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned command_controls = 0x700;
constexpr unsigned address_controls = 0xb00;
constexpr unsigned data_controls = 0x300;
constexpr unsigned low_write = 0x200;
constexpr unsigned low_read = 0x200;
constexpr unsigned read_direction = 0x400;
constexpr unsigned input_function = 0x3ff;
constexpr unsigned output_direction = 0x4ff;
constexpr unsigned output_function = 0x300;
constexpr unsigned program_nops = 7000;
constexpr unsigned erase_nops = 40000;
constexpr unsigned read_nops = 256;
constexpr unsigned max_cycles = 100000;
constexpr std::uint64_t power_tick = 108000;
constexpr std::uint64_t inherited_tick = 98000000;
constexpr std::uint64_t program_ticks = 540000;
constexpr std::uint64_t erase_ticks = 3240000;
constexpr unsigned carry_mask = 1;
constexpr unsigned data_shift = 24;
constexpr unsigned low_test_prefix = 0x014020;
constexpr unsigned timer1_offset = 0x0b;
constexpr unsigned io_address_shift = 8;
constexpr unsigned ready_bit = 12;
constexpr unsigned high_test_prefix = 0x0b8020;
constexpr unsigned port_f_offset = 0x35;
constexpr unsigned y_space_bit = 0x40;

void ensure(const bool value, const std::string &why) {
  if (!value) { throw Fault(why); }
}

struct NativeCase {
  const char *name;
  bool erase;
  bool inhibited;
  bool input_one;
  bool restore;
};

constexpr std::array native_cases{NativeCase{.name = "program-pass",
                                             .erase = false,
                                             .inhibited = false,
                                             .input_one = false,
                                             .restore = true},
                                  NativeCase{.name = "program-fail",
                                             .erase = false,
                                             .inhibited = true,
                                             .input_one = false,
                                             .restore = false},
                                  NativeCase{.name = "program-inhibited-one",
                                             .erase = false,
                                             .inhibited = true,
                                             .input_one = true,
                                             .restore = false},
                                  NativeCase{.name = "erase-pass",
                                             .erase = true,
                                             .inhibited = false,
                                             .input_one = false,
                                             .restore = true},
                                  NativeCase{.name = "erase-fail",
                                             .erase = true,
                                             .inhibited = true,
                                             .input_one = false,
                                             .restore = false}};

Nand native_device(const NativeCase &test) {
  NandImage image;
  NandArray::Page page;
  page.cells.fill(
      {.value = static_cast<std::uint8_t>(test.erase ? 0U : byte_mask), .known = byte_mask});
  page.programs = test.erase ? NandProgramCount{.main = 1, .spare = 1} : NandProgramCount{};
  for (unsigned row = 0; row <= Nand::block_pages; ++row) { image.pages.emplace(row, page); }
  image.blocks.emplace(
      0, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  Nand nand(image, NandReadContinuation::k9f1208_p);
  nand.observe_wp(true);
  return nand;
}

NandCellProfile native_physics(const NativeCase &test) {
  NandCellProfile profile{.contract = NandCellContract::reference_pulse_inhibit, .cells = {}};
  if (test.inhibited) {
    profile.cells.emplace(
        0,
        NandCellInhibit{.program = static_cast<std::uint8_t>(test.erase ? 0U : 1U),
                        .erase = static_cast<std::uint8_t>(test.erase ? 1U : 0U)});
  }
  return profile;
}

class NativeProgram {
public:
  struct Markers {
    unsigned busy = 0, ready = 0, status = 0, stop = 0;
  };

  NativeProgram(DspBus &bus, const NativeCase &test) { install(bus, test); }

  [[nodiscard]] const Markers &markers() const { return markers_; }

  void emit(const std::string &line) {
    const auto result = assembler_.assemble(line.c_str());
    ensure(result.success(), "Own native medium assembly failed: " + line);
    for (unsigned i = 0; i < result.wordCount; ++i) { words_.push_back(result.word[i]); }
  }

  void nops(const unsigned count) { words_.insert(words_.end(), count, 0); }

  [[nodiscard]] unsigned pc() const { return boot + static_cast<unsigned>(words_.size()); }

  void x(const unsigned value) { emit("movep #$" + hex(value, 6).substr(2) + ",x:<<$ffffbd"); }

  void f(const unsigned value) { emit("movep #$" + hex(value, 6).substr(2) + ",y:<<$fffff5"); }

  void latch(const NandWriteKind kind, const unsigned value) {
    unsigned controls = data_controls;
    if (kind == NandWriteKind::command) { controls = command_controls; }
    if (kind == NandWriteKind::address) { controls = address_controls; }
    x(controls);
    f(value);
    x(controls & ~low_write);
    nops(1);
    x(controls);
  }

  void input() {
    emit("movep #$" + hex(read_direction, 6).substr(2) + ",y:<<$fffff6");
    emit("movep #$" + hex(input_function, 6).substr(2) + ",y:<<$fffff7");
  }

  void output() {
    emit("movep #$" + hex(output_function, 6).substr(2) + ",y:<<$fffff7");
    emit("movep #$" + hex(output_direction, 6).substr(2) + ",y:<<$fffff6");
  }

  void install(DspBus &bus, const NativeCase &test) {
    for (const auto *line : {"movep #$01ffff,x:<<$fffffb",
                             "movep #$000f00,x:<<$ffffbe",
                             "movep #$0000ff,x:<<$ffffbf",
                             "movep #$000700,x:<<$ffffbd"}) {
      emit(line);
    }
    output();
    if (test.erase) {
      constexpr unsigned setup = 0x60;
      constexpr unsigned confirm = 0xd0;
      latch(NandWriteKind::command, setup);
      for (unsigned i = 0; i < 3; ++i) { latch(NandWriteKind::address, 0); }
      latch(NandWriteKind::command, confirm);
    } else {
      constexpr unsigned setup = 0x80;
      constexpr unsigned confirm = 0x10;
      constexpr std::array<unsigned, 3> bytes{0xa2, 0xc8, 0xed};
      latch(NandWriteKind::command, setup);
      for (unsigned i = 0; i < 4; ++i) { latch(NandWriteKind::address, 0); }
      for (unsigned i = 0; i < bytes.size(); ++i) {
        latch(NandWriteKind::data, bytes[i] | (i == 0 && test.input_one ? 1U : 0U));
      }
      latch(NandWriteKind::command, confirm);
    }
    words_.push_back(low_test_prefix | (timer1_offset << io_address_shift) | ready_bit);
    nops(1);
    markers_.busy = pc();
    nops(test.erase ? erase_nops : program_nops);
    words_.push_back(low_test_prefix | (timer1_offset << io_address_shift) | ready_bit);
    nops(1);
    markers_.ready = pc();
    constexpr unsigned status = 0x70;
    latch(NandWriteKind::command, status);
    x(data_controls);
    input();
    x(low_read);
    nops(2);
    words_.push_back(high_test_prefix | (port_f_offset << io_address_shift) | y_space_bit);
    nops(1);
    markers_.status = pc();
    x(data_controls);
    nops(1);
    output();
    latch(NandWriteKind::command, 0);
    for (unsigned i = 0; i < 4; ++i) { latch(NandWriteKind::address, 0); }
    x(data_controls);
    nops(read_nops);
    input();
    for (const auto *reg : {"x0", "y0", "a"}) {
      x(low_read);
      nops(2);
      emit("movep y:<<$fffff5," + std::string(reg));
      x(data_controls);
    }
    markers_.stop = pc();
    nops(read_nops);
    bus.install_rom({.start = boot, .words = words_});
  }

private:
  Markers markers_;
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
};

DspFetchScheduleInputs native_settings() {
  return {.dsp = {.extal_hz = extal,
                  .timebase_hz = nand_timebase_hz,
                  .timing = DspFetchTiming::reference_cycle_order,
                  .execution = DspReferenceExecution::staged_register_boundaries,
                  .pipeline = DspFetchPipeline::automatic_memory,
                  .resets = DspFetchResetTiming::reference_cycle_boundary}};
}

struct NativeFixture {
  explicit NativeFixture(const NativeCase &test)
      : program(machine.bus(), test),
        peer(machine, native_device(test), native_physics(test)),
        schedule(native_settings(),
                 machine,
                 DspHardwareResetInputs{
                     .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}),
        inherited{.identity = std::make_shared<const DspFetchEventIdentity>(),
                  .next_ticks = []() -> std::optional<std::uint64_t> { return inherited_tick; },
                  .observe = [](std::uint64_t) { throw Fault("Inherited source remains pending"); },
                  .order = DspExternalOrder::external_before_dsp,
                  .maximum_events = 4} {
    static_cast<void>(schedule.advance_to(
        {.horizon = {.ticks = power_tick}, .maximum_cycles = 0},
        [] { return DspFetchCycleInputs{}; },
        {},
        inherited,
        peer.events()));
    schedule.release_hardware_reset(peer.events());
  }

  void tick() {
    const auto elapsed = schedule.run().elapsed_ticks();
    const auto next = power_tick + (((elapsed - power_tick) / period + 1) * period);
    static_cast<void>(schedule.advance_to(
        {.horizon = {.ticks = next}, .maximum_cycles = 1},
        [&] {
          peer.before_native(schedule.run().elapsed_ticks());
          return DspFetchCycleInputs{.known_requests = 15, .grant = DspDmaGrant::available};
        },
        [&](const DspFetchCycleResult &cycle) { peer.after_native(cycle.completion_ticks); },
        inherited,
        peer.events()));
    const auto state = machine.cpu().state();
    if (state.pc == program.markers().busy) {
      busy_seen = true;
      ensure((state.sr & carry_mask) == 0, "Native BTST sees busy");
    }
    if (state.pc == program.markers().ready) {
      ready_seen = true;
      ensure((state.sr & carry_mask) != 0, "Native BTST sees derived Ready");
    }
    if (state.pc == program.markers().status) { status_seen = true; }
  }

  void finish() {
    unsigned count = 0;
    while (machine.cpu().state().pc != program.markers().stop) {
      ensure(++count < max_cycles, "Native write/poll/status/readback suffix bounded");
      tick();
    }
  }

  DspMachine machine;
  NativeProgram program;
  NandPeer peer;
  DspFetchScheduledRun schedule;
  DspFetchExternalEvents inherited;
  bool busy_seen = false, ready_seen = false, status_seen = false;
};

void native_endpoint(NativeFixture &f, const NativeCase &test) {
  constexpr unsigned second = 0xc8;
  constexpr unsigned third = 0xed;
  constexpr unsigned first = 0xa2;
  const auto cpu = f.machine.cpu().debug_state();
  const bool fails = test.inhibited && (test.erase || !test.input_one);
  unsigned expected_first = first | (test.inhibited || test.input_one ? 1U : 0U);
  if (test.erase) { expected_first = fails ? byte_mask - 1 : byte_mask; }
  const auto x = static_cast<unsigned>(cpu.cpu.x) & word_mask;
  const auto y = static_cast<unsigned>(cpu.cpu.y) & word_mask;
  const auto a = static_cast<std::uint64_t>(test.erase ? byte_mask : third) << data_shift;
  ensure(x == expected_first && y == (test.erase ? byte_mask : second) && cpu.cpu.a == a,
         "Actual native readback matches independently computed physical cells");
  ensure(((cpu.cpu.sr & carry_mask) != 0) == fails,
         "Native GPIO BTST sees computed verification status");
  ensure(f.busy_seen && f.ready_seen && f.status_seen,
         "Genuine native Busy,Ready and status instructions retired");
  ensure(f.peer.detailed().base.ready == true &&
             f.peer.detailed().base.pending == NandPending::none,
         "No pending write remains at the native endpoint");
}

void execute_native(const NativeCase &test) {
  std::cout << "CASE NATIVE_MEDIUM name=" << test.name << '\n';
  NativeFixture f(test);
  const auto kind = test.erase ? NandPending::erase : NandPending::program;
  unsigned count = 0;
  while (f.peer.detailed().base.pending != kind) {
    ensure(++count < max_cycles, "Native confirmation bounded");
    f.tick();
  }
  const auto at = f.peer.detailed().base.ticks;
  const auto due = at + (test.erase ? erase_ticks : program_ticks);
  ensure(f.peer.detailed().base.next == due && f.peer.detailed().base.ready == false,
         "Actual native confirmation creates its own cell-verification deadline");
  const auto payload = f.peer.checkpoint();
  const auto saved = f.schedule.checkpoint(f.peer.events());
  const auto cpu_cut = f.machine.cpu().debug_state();
  const auto nand_cut = f.peer.detailed();
  f.finish();
  native_endpoint(f, test);
  const auto endpoint = f.machine.cpu().debug_state();
  const auto nand_endpoint = f.peer.detailed();
  const auto calendar_endpoint = f.schedule.peripheral_state();
  if (test.restore) {
    f.peer.restore(payload);
    f.schedule.restore(saved, f.peer.events());
    ensure(f.machine.cpu().debug_state() == cpu_cut && f.peer.detailed() == nand_cut,
           "Trusted native plus full pending cell job restores");
    f.busy_seen = false;
    f.ready_seen = false;
    f.status_seen = false;
    f.finish();
    native_endpoint(f, test);
    ensure(f.machine.cpu().debug_state() == endpoint && f.peer.detailed() == nand_endpoint &&
               f.schedule.peripheral_state() == calendar_endpoint,
           "Restored native write/poll/status/readback suffix repeats completely");
  }
  ensure(f.schedule.state().next_external == inherited_tick &&
             f.schedule.state().external_events == 0,
         "Inherited input source is retained exactly");
  std::cout << "PASS NATIVE_MEDIUM name=" << test.name << " confirm=" << at << " due=" << due
            << " restore=" << test.restore << '\n';
}
}

void native_medium_tests() {
  for (const auto &test : native_cases) { execute_native(test); }
}
}
