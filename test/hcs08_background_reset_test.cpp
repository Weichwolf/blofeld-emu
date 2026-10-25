// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
namespace commands = blofeld::test::hcs08_bdc;
namespace wire = blofeld::test::hcs08_wire;
constexpr unsigned old_pc = 0x1234, old_hx = 0x3322, old_sp = 0x4567, old_a = 0xa5;
constexpr unsigned reset_hx = 0x22, reset_sp = 0xff, reset_ccr = 0x68, load_known = 0xee;
constexpr unsigned vector_high = 0xfffe, vector_low = 0xffff, own_a = 0x42;
constexpr unsigned active_status = 0xc8, native_periods = 7, offered_periods = 9;
constexpr unsigned own_entry = 0xa0, loaded_pc = own_entry + 2, stopped_pc = own_entry + 3;
constexpr std::array<std::uint8_t, 3> code{0xa6, own_a, 0x82}; // Own LDA #42; BGND.
constexpr auto held = Hcs08BdcReset::bkgd_held_low;
constexpr auto contract = Hcs08RunContract::reference_instruction_retirement;

class ObservedMemory final : public Memory {
public:
  mutable unsigned peeks = 0, fetches = 0;
  unsigned beginnings = 0;

  std::uint8_t peek(const std::uint16_t address) const override {
    ++peeks;
    return Memory::peek(address);
  }

  void check_execute(const InstructionFetch fetch) const override {
    ++fetches;
    Memory::check_execute(fetch);
  }

  void begin_instruction(const ExecutionContext value) override {
    ++beginnings;
    Memory::begin_instruction(value);
  }
};

void warm_state(Hcs08 &cpu) {
  cpu.set_state({.pc = old_pc, .hx = old_hx, .sp = old_sp, .a = old_a, .ccr = byte_mask});
}

void expected_reset(const Hcs08 &cpu) {
  require(
      cpu.state() == Hcs08State{.pc = 0,
                                .hx = reset_hx,
                                .sp = reset_sp,
                                .a = old_a,
                                .ccr_known = reset_ccr,
                                .ccr = reset_ccr,
                                .pc_known = false} &&
          !cpu.background_requested() && !cpu.faulted(),
      "Pre-vector reset preserves warm A/X, initializes H/SP/I and leaves numerical PC unknown");
}

void no_access(const ObservedMemory &memory) {
  require(
      memory.accesses.empty() && memory.decoded.none() && memory.retired == 0 &&
          memory.peeks == 0 && memory.fetches == 0 && memory.beginnings == 0,
      "Unknown active-reset PC cannot trigger vector, data, decoder, timing or execution access");
}

void active_and_normal_reset() {
  ObservedMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  for (unsigned mask = 0; mask < 4; ++mask) {
    warm_state(cpu);
    memory.known.set(vector_high, (mask & 1U) != 0);
    memory.known.set(vector_low, (mask & 2U) != 0);
    cpu.reset(Hcs08ResetEntry::active_background);
    expected_reset(cpu);
    no_access(memory);
  }
  warm_state(cpu);
  const auto before = cpu.state();
  require(rejected([&] { cpu.reset(Hcs08ResetEntry::unqualified); }).contains("entry mode") &&
              cpu.state() == before && !cpu.faulted(),
          "Invalid reset entry rejects before CPU or memory mutation");
  no_access(memory);
  for (unsigned mask = 0; mask < 3; ++mask) {
    warm_state(cpu);
    memory.known.set(vector_high, (mask & 1U) != 0);
    memory.known.set(vector_low, (mask & 2U) != 0);
    const auto failure = rejected([&] { cpu.reset(); });
    require(
        failure.contains("read address=" + hex(mask == 1 ? vector_low : vector_high)) &&
            failure.contains("pc=" + hex(old_pc)) && cpu.faulted() && memory.retired == 0,
        "Normal reset retains first/second vector-byte failures and original pre-reset context");
  }
  warm_state(cpu);
  constexpr std::array<std::uint8_t, 2> zero_vector{0, 0};
  memory.install(vector_high, zero_vector);
  cpu.reset();
  require(cpu.state().pc == 0 && cpu.state().pc_known,
          "A real zero-valued reset vector is known and distinct from unknown PC representation");
  memory.install(0, code);
  cpu.step();
  require(cpu.state().pc == 2 && cpu.state().a == own_a && memory.retired == 1,
          "Known address0000 executes native immediate LDA with independent PC/A expectation");
}

void unknown_fetches() {
  ObservedMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  warm_state(cpu);
  cpu.reset(Hcs08ResetEntry::active_background);
  auto unknown = cpu.state();
  unknown.pc = old_pc;
  cpu.set_state(unknown);
  expected_reset(cpu);
  const auto snapshot = cpu.checkpoint();
  require(
      rejected([&] { static_cast<void>(cpu.disassemble()); }).contains("pc=unknown pc-known=0") &&
          !cpu.faulted(),
      "Passive disassembly refuses unknown PC before decoder access without faulting CPU");
  no_access(memory);
  require(rejected([&] { cpu.step(); }).contains("execute address=unknown pc=unknown pc-known=0") &&
              cpu.faulted(),
          "Native execution quarantines unknown PC before any fetch or retirement");
  no_access(memory);
  cpu.restore_checkpoint(snapshot);
  expected_reset(cpu);
  no_access(memory);
  const auto failure = rejected([&] { cpu.reset(); });
  require(failure.contains("pc=unknown pc-known=0") &&
              failure.contains("address=" + hex(vector_high)) && cpu.faulted() &&
              !cpu.state().pc_known && memory.retired == 0,
          "Normal reset after a held unknown PC retains unknown prior context when its vector read "
          "fails");
}

void prepare(wire::Target &target) {
  auto image = target.memory.flash.image();
  image.known.reset(vector_high - hcs08_flash::first);
  image.known.reset(vector_low - hcs08_flash::first);
  target.memory.flash = Hcs08Flash(image, wire::chip, Hcs08IdleLaunch::unqualified);
  warm_state(target.cpu);
  target.cpu.reset(Hcs08ResetEntry::active_background);
  expected_reset(target.cpu);
  require(target.memory.accesses.empty() && target.memory.decoded.none() &&
              target.memory.retired == 0,
          "Candidate target active reset does not read unknown vector cells");
}

void register_writes() {
  wire::Target target;
  prepare(target);
  Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, held);
  constexpr std::array<std::uint8_t, 1> status{0xe4};
  constexpr std::array<std::uint8_t, 1> read_a{0x68};
  constexpr std::array<std::uint8_t, 1> read_hx{0x6c};
  constexpr std::array<std::uint8_t, 1> read_sp{0x6f};
  commands::command(bdc, status);
  require(bdc.read_reply() == active_status,
          "Actual READ_STATUS reports enabled active reset state");
  commands::command(bdc, read_a);
  require(bdc.read_reply() == old_a, "Held READ_A returns retained warm A without PC");
  commands::command(bdc, read_hx);
  require(bdc.read_reply() == 0 && bdc.read_reply() == reset_hx,
          "Held READ_HX serializes reset-zero H and retained X");
  commands::command(bdc, read_sp);
  require(bdc.read_reply() == 0 && bdc.read_reply() == reset_sp,
          "Held READ_SP serializes actual reset stack");
  constexpr std::array<std::uint8_t, 2> write_a{0x48, own_a};
  constexpr std::array<std::uint8_t, 2> write_ccr{0x49, reset_ccr};
  constexpr std::array<std::uint8_t, 3> write_hx{0x4c, 0x12, 0x34};
  constexpr std::array<std::uint8_t, 3> write_sp{0x4f, 0x04, 0x7c};
  commands::command(bdc, write_a);
  commands::command(bdc, write_ccr);
  commands::command(bdc, write_hx);
  commands::command(bdc, write_sp);
  constexpr unsigned written_hx = 0x1234;
  constexpr unsigned written_sp = 0x47c;
  require(target.cpu.state() == Hcs08State{.pc = 0,
                                           .hx = written_hx,
                                           .sp = written_sp,
                                           .a = own_a,
                                           .ccr_known = byte_mask,
                                           .ccr = reset_ccr,
                                           .pc_known = false} &&
              target.memory.accesses.empty() && target.memory.retired == 0,
          "Non-PC BDC writes update only actual named registers and preserve unknown PC");
}

void bdc_refusals() {
  for (const std::uint8_t opcode : {hcs08_bdc::read_pc, hcs08_bdc::go}) {
    wire::Target target;
    prepare(target);
    Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, held);
    const std::array<std::uint8_t, 1> frame{opcode};
    const auto failure = rejected([&] { commands::command(bdc, frame); });
    require(
        failure.contains("unknown CPU PC") && failure.contains("pc=unknown pc-known=0") &&
            failure.contains(opcode == hcs08_bdc::go ? "GO" : "READ_PC") && bdc.faulted() &&
            bdc.state().active && bdc.state().serviced == 0 && bdc.state().reply_size == 0 &&
            target.memory.retired == 0 && target.memory.accesses.empty(),
        "Unqualified PC cannot be serialized or released by GO; command fails visibly and held");
    expected_reset(target.cpu);
  }
}

void unknown_memory_refusal() {
  wire::Target target;
  prepare(target);
  Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, held);
  const auto nvm = target.memory.flash.image();
  const auto failure = rejected([&] { static_cast<void>(commands::read(bdc, vector_high)); });
  require(
      failure.contains("pc=unknown pc-known=0") &&
          failure.contains("address=" + hex(vector_high)) && bdc.faulted() && bdc.state().active &&
          bdc.state().reply_size == 0 && target.memory.flash.image() == nvm &&
          target.memory.retired == 0,
      "Held BDC cannot fabricate a missing vector byte; diagnostics retain explicit unknown PC");
}

void clock_refusal() {
  wire::Target target;
  prepare(target);
  Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, Hcs08BdcReset::normal);
  Hcs08RunClock clock(target.memory, bdc, contract);
  require(rejected([&] { clock.advance(); }).contains("pc=unknown pc-known=0") && clock.faulted() &&
              clock.state().executing_periods == 0 && clock.state().retired_instructions == 0 &&
              clock.state().remaining_periods == 0 && target.memory.retired == 0 &&
              target.memory.accesses.empty() && target.memory.decoded.none(),
          "Running-target owner refuses unknown PC before timing lookup and code decoding");
}

void uploaded_suffix(wire::Target &target, Hcs08Bdc &bdc, Hcs08RunClock &clock) {
  const auto nvm = target.memory.flash.image();
  clock.advance();
  require(clock.state().executing_periods == 0 && target.memory.retired == 0,
          "Active debug mode offers a clock period without executing unknown PC");
  for (unsigned n = 0; n < code.size(); ++n) {
    commands::write(bdc, {.address = static_cast<std::uint16_t>(own_entry + n), .value = code[n]});
  }
  require(!target.cpu.state().pc_known && target.memory.retired == 0,
          "Actual RAM upload alone does not invent a CPU entry point");
  constexpr std::array<std::uint8_t, 3> set_pc{0x4b, 0, own_entry};
  constexpr std::array<std::uint8_t, 1> read_pc{0x6b};
  constexpr std::array<std::uint8_t, 1> go{0x08};
  commands::command(bdc, set_pc);
  require(target.cpu.state().pc_known && target.cpu.state().pc == own_entry &&
              target.cpu.state().ccr_known == reset_ccr,
          "Actual WRITE_PC establishes only PC knowledge and preserves partial reset flags");
  commands::command(bdc, read_pc);
  require(bdc.read_reply() == 0 && bdc.read_reply() == own_entry,
          "Actual READ_PC serializes the host-supplied entry after WRITE_PC");
  commands::command(bdc, go);
  require(!bdc.state().active, "Actual GO releases target only after a known WRITE_PC");
  constexpr unsigned period_bound = 20;
  while (!bdc.state().active) {
    require(clock.state().offered_periods < period_bound, "Own native LDA/BGND has bounded cost");
    clock.advance();
    if (target.memory.retired == 1) {
      require(target.cpu.state().pc == loaded_pc && target.cpu.state().a == own_a,
              "First actual instruction loads42 and advances by two bytes");
    }
  }
  require(target.cpu.state() == Hcs08State{.pc = stopped_pc,
                                           .hx = reset_hx,
                                           .sp = reset_sp,
                                           .a = own_a,
                                           .ccr_known = load_known,
                                           .ccr = reset_ccr} &&
              target.cpu.background_requested() && target.memory.retired == 2 &&
              clock.state().retired_instructions == 2 &&
              clock.state().executing_periods == native_periods &&
              clock.state().offered_periods == offered_periods && target.memory.transfers == 0 &&
              target.memory.flash.image() == nvm,
          "Native LDA/BGND returns to BDC with independent full state; vectors remain unknown and "
          "NVM intact");
}

void snapshot_recovery() {
  wire::Target source;
  prepare(source);
  Hcs08Bdc bdc(source.cpu, source.mapping, wire::chip, held);
  Hcs08RunClock clock(source.memory, bdc, contract);
  clock.advance();
  const auto cpu_snapshot = source.cpu.checkpoint();
  const auto memory = source.memory;
  const auto bdc_snapshot = bdc.checkpoint();
  const auto clock_snapshot = clock.checkpoint();
  uploaded_suffix(source, bdc, clock);
  for (unsigned replay = 0; replay < 2; ++replay) {
    wire::Target target(&memory);
    target.cpu.restore_checkpoint(cpu_snapshot);
    expected_reset(target.cpu);
    Hcs08Bdc restored(target.cpu, target.mapping, wire::chip, held);
    restored.restore_checkpoint(bdc_snapshot);
    Hcs08RunClock restored_clock(target.memory, restored, contract);
    restored_clock.restore_checkpoint(clock_snapshot);
    uploaded_suffix(target, restored, restored_clock);
    require(target.cpu.state() == source.cpu.state() && restored.state() == bdc.state() &&
                restored_clock.state() == clock.state() &&
                target.memory.accesses == source.memory.accesses &&
                target.memory.bytes == source.memory.bytes &&
                target.memory.known == source.memory.known,
            "Two fresh joint restorations retain unknown PC and reproduce actual upload/GO "
            "instruction effects");
  }
}
} // namespace

int main() {
  try {
    active_and_normal_reset();
    unknown_fetches();
    register_writes();
    bdc_refusals();
    unknown_memory_refusal();
    clock_refusal();
    snapshot_recovery();
    std::cout
        << "HCS08 pre-vector background reset, unknown PC and native BDC recovery verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
