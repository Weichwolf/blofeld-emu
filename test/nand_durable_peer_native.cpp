// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_bus_decode.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "nand_durable_native_tests.hpp"
#include "nand_native_peer.hpp"
#include "nand_native_run.hpp"
#include "nand_persistence_fixture.hpp"
#include "storage/nand.hpp"
#include "storage/nand_array.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_file.hpp"
#include "storage/nand_image.hpp"
#include "storage/nand_write_store.hpp"
#include "storage/nand_write_timing.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace blofeld::test {
namespace {
using namespace blofeld;
constexpr unsigned boot = 0xff0000;
// FM MOVEP: low-X qq, high-X/high-Y pp; register reads use X0/Y0/full A fields.
constexpr unsigned write_port_c = 0x07f43d, write_port_f = 0x09f4b5;
constexpr unsigned write_direction_f = 0x09f4b6, write_function_f = 0x09f4b7;
constexpr unsigned write_bus_control = 0x08f4bb, write_direction_c = 0x07f43e;
constexpr unsigned write_function_c = 0x07f43f;
constexpr unsigned read_port_f_x0 = 0x094435, read_port_f_y0 = 0x094635, read_port_f_a = 0x094e35;
constexpr unsigned port_f_bus_control = 0x01ffff;
constexpr unsigned control_directions = 0x000f00, control_functions = 0x0000ff;
constexpr unsigned extal = 12000000;
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
};

constexpr std::array native_cases{
    NativeCase{.name = "program-pass", .erase = false, .inhibited = false, .input_one = false},
    NativeCase{.name = "program-fail", .erase = false, .inhibited = true, .input_one = false},
    NativeCase{
        .name = "program-inhibited-one", .erase = false, .inhibited = true, .input_one = true},
    NativeCase{.name = "erase-pass", .erase = true, .inhibited = false, .input_one = false},
    NativeCase{.name = "erase-fail", .erase = true, .inhibited = true, .input_one = false}};

NandImage native_image(const NativeCase &test) {
  NandImage image;
  NandArray::Page page;
  page.cells.fill(
      {.value = static_cast<std::uint8_t>(test.erase ? 0U : byte_mask), .known = byte_mask});
  page.programs = test.erase ? NandProgramCount{.main = 1, .spare = 1} : NandProgramCount{};
  for (unsigned row = 0; row <= Nand::block_pages; ++row) { image.pages.emplace(row, page); }
  image.blocks.emplace(
      0, NandBlock{.valid = true, .program_protected = false, .erase_protected = false});
  return image;
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

std::unique_ptr<NandWriteStore>
native_store(const std::filesystem::path &path, const NativeCase &test, NandFileObserver observer) {
  auto store = std::make_unique<NandWriteStore>(path,
                                                native_image(test),
                                                NandReadContinuation::k9f1208_p,
                                                nand_timebase_hz,
                                                native_physics(test));
  store->observe_wp(true); // Explicit virtual fixture input, not a reconstructed board route.
  store->set_observer(std::move(observer));
  return store;
}

class NativeProgram {
public:
  struct Markers {
    unsigned busy = 0, ready = 0, status = 0, stop = 0;
  };

  NativeProgram(DspBus &bus, const NativeCase &test, const NandBusContract contract) {
    install(bus, test, contract);
  }

  [[nodiscard]] const Markers &markers() const { return markers_; }

  // DSP56300FM MOVEP immediate opcodes; each carries one literal extension.
  void emit(const std::uint32_t opcode, const unsigned value) {
    words_.push_back(opcode);
    words_.push_back(value);
  }

  void nops(const unsigned count) { words_.insert(words_.end(), count, 0); }

  [[nodiscard]] unsigned pc() const { return boot + static_cast<unsigned>(words_.size()); }

  void x(const unsigned value) { emit(write_port_c, value); }

  void f(const unsigned value) { emit(write_port_f, value); }

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
    emit(write_direction_f, read_direction);
    emit(write_function_f, input_function);
  }

  void output() {
    emit(write_function_f, output_function);
    emit(write_direction_f, output_direction);
  }

  void install(DspBus &bus, const NativeCase &test, const NandBusContract contract) {
    emit(write_bus_control, port_f_bus_control);
    emit(write_direction_c, control_directions);
    emit(write_function_c, control_functions);
    x(command_controls);
    output();
    if (contract == NandBusContract::functional_transactions) {
      x(data_controls);
      x(0);
      constexpr unsigned neutral_qualifiers = 0xc00;
      x(neutral_qualifiers);
      x(command_controls);
      latch(NandWriteKind::command, byte_mask);
      nops(read_nops);
      if (!test.erase) { latch(NandWriteKind::command, 0); }
    }
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
    for (const auto opcode : {read_port_f_x0, read_port_f_y0, read_port_f_a}) {
      x(low_read);
      nops(2);
      words_.push_back(opcode);
      x(data_controls);
    }
    markers_.stop = pc();
    nops(read_nops);
    bus.install_rom({.start = boot, .words = words_});
  }

private:
  Markers markers_;
  std::vector<std::uint32_t> words_;
};

struct NativeFixture {
  NativeFixture(const NativeCase &test,
                const NandBusContract contract,
                NandFileObserver observer = {})
      : program(machine.bus(), test, contract),
        peer(machine, native_store(directory.file(), test, std::move(observer)), contract),
        schedule(
            machine,
            peer,
            {.identity = std::make_shared<const PeripheralEventIdentity>(),
             .next_ticks = []() -> std::optional<std::uint64_t> { return inherited_tick; },
             .observe = [](std::uint64_t) { throw Fault("Inherited source remains pending"); }}) {}

  void tick() {
    const auto cycle = schedule.tick();
    if (!cycle.native || !cycle.native->retired) { return; }
    const auto state = machine.cpu().registers();
    if (state.pc == program.markers().busy) {
      busy_seen = true;
      ensure((state.sr & carry_mask) == 0, "Native BTST sees busy");
    }
    if (state.pc == program.markers().ready) {
      ready_seen = true;
      ensure((state.sr & carry_mask) != 0, "Native BTST sees derived Ready");
    }
    if (state.pc == program.markers().status) { status_seen = true; }
    if (state.pc == program.markers().stop) { stopped = true; }
  }

  void finish() {
    unsigned count = 0;
    while (!stopped) {
      ensure(++count < max_cycles, "Native write/poll/status/readback suffix bounded");
      tick();
    }
  }

  const PersistenceDirectory directory;
  OwnDspMachine machine{{.extal_hz = extal,
                         .timebase_hz = nand_timebase_hz,
                         .switch_timing = DspClockSwitch::reference_cycle_boundary}};
  NativeProgram program;
  NandPeer peer;
  NandNativeRun schedule;
  bool busy_seen = false, ready_seen = false, status_seen = false, stopped = false;
};

void native_endpoint(NativeFixture &f, const NativeCase &test) {
  constexpr unsigned second = 0xc8;
  constexpr unsigned third = 0xed;
  constexpr unsigned first = 0xa2;
  const auto cpu = native_cpu(f.machine);
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
  ensure(f.busy_seen && f.ready_seen && f.status_seen && f.stopped,
         "Genuine native Busy,Ready and status instructions retired");
  ensure(f.peer.detailed().base.ready == true &&
             f.peer.detailed().base.pending == NandPending::none,
         "No pending write remains at the native endpoint");
}

void execute_native(const NativeCase &test, const NandBusContract contract) {
  std::cout << "CASE DURABLE_NATIVE_MEDIUM name=" << test.name
            << " contract=" << static_cast<unsigned>(contract) << '\n';
  NativeFixture f(test, contract);
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
  ensure(f.peer.durable(), "The actual peer owns the persistent backend");
  persistence_reject([&] { static_cast<void>(f.peer.checkpoint()); }, "RAM-only checkpoint");
  const auto inflight = load_nand_image(f.directory.file());
  ensure(inflight.pages.at(0).cells.at(0) == NandByte{} &&
             inflight.pages.at(Nand::block_pages).cells.at(0) ==
                 native_image(test).pages.at(Nand::block_pages).cells.at(0),
         "Actual native confirmation publishes uncertain target and preserves neighbor");
  f.finish();
  native_endpoint(f, test);
  const auto image = load_nand_image(f.directory.file());
  auto observed = f.peer.protocol_copy();
  observed.power_loss();
  const auto live = observed.persistent_image();
  ensure(encode_nand_image(image) == encode_nand_image(live),
         "Actual native completion/readback matches the complete durable cold image");
  const auto cell = image.pages.at(0).cells.at(0);
  const bool fails = test.inhibited && (test.erase || !test.input_one);
  constexpr unsigned programmed_first = 0xa2;
  unsigned expected = programmed_first | (test.inhibited || test.input_one ? 1U : 0U);
  if (test.erase) { expected = fails ? byte_mask - 1 : byte_mask; }
  ensure(cell == NandByte{.value = static_cast<std::uint8_t>(expected), .known = byte_mask},
         "Independent expected first cell is actually on disk");
  persistence_reject(
      [&] {
        const NandWriteStore second(f.directory.file(),
                                    NandReadContinuation::k9f1208_p,
                                    nand_timebase_hz,
                                    native_physics(test));
      },
      "");
  ensure(f.schedule.state().next_external == inherited_tick &&
             f.schedule.state().external_events == 0,
         "Inherited input source is retained exactly");
  std::cout << "PASS DURABLE_NATIVE_MEDIUM name=" << test.name << " confirm=" << at
            << " due=" << due << " RAM-restore-refused=1" << '\n';
}

void publication_failure(const bool completion) {
  unsigned publications = 0;
  NativeFixture f(
      native_cases.front(), NandBusContract::ordered_pins, [&](const NandFilePhase phase) {
        if (phase == NandFilePhase::published && publications == (completion ? 1U : 0U)) {
          throw Fault("Injected durable publication failure");
        }
        if (phase == NandFilePhase::directory_synced) { ++publications; }
      });
  std::string reason;
  try {
    f.finish();
  } catch (const Fault &error) { reason = error.what(); }
  ensure(reason.contains("Injected durable publication failure") &&
             reason.contains("X:0xffffbd/Y:0xfffff5") && reason.contains("pc=") &&
             f.peer.faulted() && !f.ready_seen && !f.stopped,
         "Actual native publication failure stops with address/PC/context before false Ready");
  persistence_reject([&] { static_cast<void>(f.peer.events().next_ticks()); }, "quarantined");
  persistence_reject([&] { static_cast<void>(f.peer.protocol_copy()); }, "quarantined");
  persistence_reject([&] { f.peer.after_native(f.machine.clock().elapsed_ticks(), std::nullopt); },
                     "quarantined");
  const auto cell = load_nand_image(f.directory.file()).pages.at(0).cells.at(0);
  constexpr unsigned programmed = 0xa2;
  ensure(cell == (completion ? NandByte{.value = programmed, .known = byte_mask} : NandByte{}),
         "Published crash projection survives even when the native owner is quarantined");
}
}

void durable_medium_tests() {
  publication_failure(false);
  publication_failure(true);
  for (const auto contract :
       {NandBusContract::ordered_pins, NandBusContract::functional_transactions}) {
    for (const auto &test : native_cases) { execute_native(test, contract); }
  }
}
}
