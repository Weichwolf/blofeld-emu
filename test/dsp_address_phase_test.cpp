// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/dsp_reference_execution.hpp"
#include "blofeld/dsp_shi.hpp"
#include "dsp_address_phase_fixture.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"
#include "reference_execution_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test;
using namespace blofeld::test::address_phase;
namespace access = blofeld::test::reference;
using native_access::fault;

struct Mode {
  const char *pattern;
  unsigned address;
  unsigned result;
  bool updates;
};

constexpr std::array modes{
    Mode{.pattern = "(r)-n", .address = initial_r, .result = 4, .updates = true},
    Mode{.pattern = "(r)+n", .address = initial_r, .result = 2, .updates = true},
    Mode{.pattern = "(r)-", .address = initial_r, .result = decremented, .updates = true},
    Mode{.pattern = "(r)+", .address = initial_r, .result = 0, .updates = true},
    Mode{.pattern = "(r)", .address = initial_r, .result = initial_r, .updates = false},
    Mode{.pattern = "(r+n)", .address = 2, .result = initial_r, .updates = false},
    Mode{.pattern = "-(r)", .address = decremented, .result = decremented, .updates = true}};

std::string operand(const Mode mode, const unsigned index) {
  auto text = std::string(mode.pattern);
  const auto reg = std::to_string(index);
  text.replace(text.find('r'), 1, "r" + reg);
  if (const auto position = text.find('n'); position != std::string::npos) {
    text.replace(position, 1, "n" + reg);
  }
  return text;
}

void one_mode(const unsigned index, const Mode mode, const DspSpace space, const bool store) {
  DspMachine machine;
  DspProgram code;
  const auto reg = std::to_string(index);
  code.immediate("r" + reg, initial_r);
  code.immediate("n" + reg, offset);
  code.immediate("m" + reg, initial_r);
  code.immediate("x0", access::first);
  code.immediate("y0", access::first);
  const auto body = code.pc();
  const std::string name = space == DspSpace::x ? "x" : "y";
  const auto memory = name + ':' + operand(mode, index);
  const auto data = name + '0';
  code.emit("move " + (store ? data + ',' + memory : memory + ',' + data));
  code.install(machine.bus());
  machine.bus().write({.space = space, .address = mode.address, .value = access::second});
  checkpoint::run(machine, 5);
  const auto before = machine.cpu().debug_state();
  auto expected = before;
  expected.cpu.r.at(index) = mode.result;
  auto progress = machine.begin_reference_instruction(execution);
  progress = machine.advance_reference_access(access::ticket(progress));
  const auto context = name + ':' + operand(mode, index) + (store ? " store" : " load");
  require(machine.cpu().debug_state() == before,
          context + ": no unadmitted CPU result is published");
  if (mode.updates) {
    const auto address = update(progress);
    require(address.index == index && address.before == initial_r && address.value == mode.result &&
                address.space == space && address.address == mode.address && address.pc == body &&
                address.omr == reset_omr &&
                address.operation == (store ? DspMemoryOperation::write : DspMemoryOperation::read),
            context + ": independent modulo EA/register/operation expectations");
    const auto marker = access::ticket(progress);
    progress = machine.advance_reference_access(marker);
    static_cast<void>(fault([&] { static_cast<void>(machine.advance_reference_access(marker)); }));
  }
  require(access::request(progress).target.address == mode.address &&
              access::request(progress).target.space == space &&
              machine.cpu().debug_state() == expected &&
              machine.bus().read({.space = space, .address = mode.address}) == access::second,
          context + ": only the native R result is visible before actual memory access");
  const auto saved = machine.checkpoint();
  machine.restore(saved);
  progress = machine.advance_reference_access(access::ticket(machine));
  require(machine.cpu().debug_state() == expected,
          context + ": sampled data still awaits retirement");
  progress = retire(machine, progress);
  require(progress.retired && machine.cpu().state().r.at(index) == mode.result &&
              machine.cpu().state().pc == body + 1 &&
              machine.cpu().debug_state().native_instructions == before.native_instructions + 1 &&
              machine.bus().read({.space = space, .address = mode.address}) ==
                  (store ? access::first : access::second) &&
              (space == DspSpace::x ? machine.cpu().state().x : machine.cpu().state().y) ==
                  (store ? access::first : access::second),
          context + ": restore uses the original computation base, never a second AGU update");
}

void no_change_is_still_an_update() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", initial_r);
  code.immediate("n0", 0);
  code.emit("move x:(r0)+n0,x0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = initial_r, .value = access::first});
  checkpoint::run(machine, 2);
  auto progress = machine.begin_reference_instruction(execution);
  progress = machine.advance_reference_access(access::ticket(progress));
  const auto address = update(progress);
  require(address.before == initial_r && address.value == initial_r,
          "A real post-update by zero remains an AGU event even without a changed register bit");
  progress = machine.advance_reference_access(access::ticket(progress));
  progress = machine.advance_reference_access(access::ticket(progress));
  require(retire(machine, progress).retired && machine.cpu().state().r[0] == initial_r,
          "The zero-offset native update and actual load retire once");
}

void parallel_result_and_fork() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", initial_r);
  code.immediate("m0", initial_r);
  code.immediate("x1", 2);
  code.immediate("a1", 1);
  code.immediate("sr", masked_status | carry | overflow | zero | negative);
  code.emit("add x1,a x:(r0)+,x0");
  code.install(machine.bus());
  checkpoint::run(machine, 5);
  const auto before = machine.cpu().debug_state();
  auto visible = before;
  visible.cpu.r[0] = 0;
  auto progress = machine.begin_reference_instruction(execution);
  progress = machine.advance_reference_access(access::ticket(progress));
  const auto original = machine.checkpoint();
  progress = machine.advance_reference_access(access::ticket(progress));
  require(
      machine.cpu().debug_state() == visible,
      "AG2 publishes only the actual wrapped address; parallel arithmetic/CCR/load remain held");
  const auto after_update = machine.checkpoint();
  const auto stale = access::ticket(progress);
  for (unsigned replay = 0; replay < 2; ++replay) {
    DspMachine fork;
    fork.restore(after_update);
    static_cast<void>(fault([&] { static_cast<void>(fork.advance_reference_access(stale)); }));
    fork.bus().write({.space = DspSpace::x, .address = initial_r, .value = access::first});
    auto sampled = fork.advance_reference_access(access::ticket(fork));
    require(fork.cpu().debug_state() == visible,
            "The restored native continuation preserves published R without exposing ALU/CCR");
    fork.bus().write({.space = DspSpace::x, .address = initial_r, .value = access::second});
    sampled = retire(fork, sampled);
    constexpr std::uint64_t unit = std::uint64_t{1} << word_bits;
    constexpr unsigned ccr_mask = 0xff;
    require(sampled.retired && fork.cpu().state().a == 3 * unit &&
                fork.cpu().state().x == 2 * unit + access::first && fork.cpu().state().r[0] == 0 &&
                (fork.cpu().state().sr & ccr_mask) == unnormalized,
            "The independent native ADD/CCR/modulo/load result survives a fresh-machine fork");
  }
  machine.restore(original);
  require(machine.cpu().debug_state() == before,
          "Restore before AG2 undoes only the accepted later R phase");
}

void long_pair_updates_once() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", initial_r);
  code.immediate("m0", initial_r);
  code.emit("move l:(r0)+,a");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = initial_r, .value = access::first});
  machine.bus().write({.space = DspSpace::y, .address = initial_r, .value = access::second});
  checkpoint::run(machine, 2);
  auto progress = machine.begin_reference_instruction(execution);
  progress = machine.advance_reference_access(access::ticket(progress));
  require(update(progress).value == 0, "The long native X/Y pair has one actual AGU update");
  progress = machine.advance_reference_access(access::ticket(progress));
  progress = machine.advance_reference_access(access::ticket(progress));
  require(access::request(progress).target.space == DspSpace::y && machine.cpu().state().r[0] == 0,
          "The second actual Y read follows the already published R without a duplicate update");
  const auto saved = machine.checkpoint();
  machine.restore(saved);
  progress = machine.advance_reference_access(access::ticket(machine));
  require(retire(machine, progress).retired && machine.cpu().state().r[0] == 0 &&
              machine.cpu().state().a ==
                  (static_cast<std::uint64_t>(access::first) << word_bits) + access::second,
          "Both actual long operands and the single wrapped address survive restoration");
}

void unsupported_io_does_not_read() {
  constexpr unsigned control = 0xffff91;
  constexpr unsigned configuration = 0xa9;
  DspMachine machine;
  DspProgram code;
  code.emit("movep x:<<$ffff94,x0");
  code.install(machine.bus());
  machine.bus().shi().observe_spi({.mosi = false});
  machine.bus().write({.space = DspSpace::x, .address = control, .value = configuration});
  access::send(machine.bus().shi(), access::first);
  auto progress = machine.begin_reference_instruction(execution);
  const auto error =
      fault([&] { static_cast<void>(machine.advance_reference_access(access::ticket(progress))); });
  require(error.contains("Unqualified native memory-address phase") &&
              error.contains("read X:0xffff94") && error.contains("pc=0xff0000") &&
              error.contains("omr=0x000305") &&
              machine.bus().read({.space = DspSpace::x, .address = access::fifo}) == access::first,
          "Unqualified phase placement fails contextually before the real destructive FIFO read");
}

void control_load_overwrites_its_address_later() {
  DspMachine machine;
  DspProgram code;
  code.immediate("r0", initial_r);
  code.immediate("m0", initial_r);
  code.emit("move x:(r0)+,r0");
  code.install(machine.bus());
  machine.bus().write({.space = DspSpace::x, .address = initial_r, .value = access::first});
  checkpoint::run(machine, 2);
  auto progress = machine.begin_reference_instruction(execution);
  progress = machine.advance_reference_access(access::ticket(progress));
  require(update(progress).value == 0,
          "MOVEC exposes its genuine wrapped address before loading R0");
  progress = machine.advance_reference_access(access::ticket(progress));
  const auto saved = machine.checkpoint();
  machine.restore(saved);
  progress = machine.advance_reference_access(access::ticket(machine));
  require(machine.cpu().state().r[0] == 0,
          "The completed control-register load remains separate from its earlier AGU result");
  require(
      retire(machine, progress).retired && machine.cpu().state().r[0] == access::first,
      "Later native retirement publishes the real loaded R0 rather than the earlier address value");
}

void absolute_forms_have_no_update() {
  for (const auto *text : {"move x:<$7,x0", "move x:>$7,x0"}) {
    DspMachine machine;
    DspProgram code;
    code.emit(text);
    code.install(machine.bus());
    machine.bus().write({.space = DspSpace::x, .address = initial_r, .value = access::first});
    const auto before = machine.cpu().debug_state();
    auto progress = machine.begin_reference_instruction(execution);
    progress = machine.advance_reference_access(access::ticket(progress));
    if (access::request(progress).kind == DspNativeAccessKind::extension) {
      progress = machine.advance_reference_access(access::ticket(progress));
    }
    require(access::request(progress).kind == DspNativeAccessKind::read &&
                access::request(progress).target.address == initial_r &&
                machine.cpu().debug_state() == before,
            "Short and extended absolute formats invent no address-register publication");
    progress = machine.advance_reference_access(access::ticket(progress));
    require(retire(machine, progress).retired && machine.cpu().state().x == access::first &&
                machine.cpu().state().r == before.cpu.r,
            "The actual absolute load completes without an unrelated R update");
  }
}
}

int main() {
  try {
    for (unsigned index = 0; index < 8; ++index) {
      for (const auto mode : modes) {
        for (const auto space : {DspSpace::x, DspSpace::y}) {
          for (const bool store : {false, true}) { one_mode(index, mode, space, store); }
        }
      }
    }
    no_change_is_still_an_update();
    parallel_result_and_fork();
    long_pair_updates_once();
    unsupported_io_does_not_read();
    control_load_overwrites_its_address_later();
    absolute_forms_have_no_update();
    clock_checks();
    std::cout << "Native DSP address publication:224 fixed cases and joint phase checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
