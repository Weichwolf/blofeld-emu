// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_flash_fixture.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_nvm;
constexpr unsigned erased_options = 0xc3, bad_hz = 1000000;
constexpr unsigned idle_status = 0xc0, error_status = 0xd0;
constexpr unsigned secure_option = 0xfd, option_secure = 0xc1, option_unsecured = 0xc2;
constexpr unsigned program_result = 0x5a, erase_begin = 0xe000;
constexpr auto reference_abort = Hcs08FlashAbort::reference_command_range;
constexpr unsigned bad_command = 0x26;

void induce_access_error(Hcs08Flash &flash, const unsigned cause) {
  flash.stage({.address = example, .value = example_data});
  switch (cause) {
    case 0: flash.stage({.address = second, .value = example_data}); break;
    case 1: flash.write({.address = hf::command, .value = bad_command}); break;
    case 2: flash.write({.address = hf::protection, .value = 0}); break;
    case 3: flash.write({.address = hf::status, .value = 0}); break;
    case 4:
      flash.write({.address = hf::command, .value = hf::erase_page});
      flash.write({.address = hf::command, .value = hf::erase_page});
      break;
    case 5:
      flash.write({.address = hf::command, .value = hf::erase_page});
      static_cast<void>(flash.read(hf::status));
      break;
    default: throw Fault("Unknown fixed access-error cause");
  }
}

void read_modify_write_case(const Hcs08FlashAgent agent, const unsigned cause) {
  Hcs08Flash flash(initial(), contract);
  setup(flash);
  flash.context({.pc = ram_pc, .ccr = flags}, agent);
  induce_access_error(flash, cause);
  require(flash.read(hf::status) == error_status &&
              flash.state().buffer == Hcs08FlashBuffer::empty && !flash.state().active,
          "The independent command violation must establish idle FACCERR before recovery");
  const auto saved = flash;
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = saved;
    const auto before = restored.state();
    const auto image = restored.image();
    const auto value = static_cast<std::uint8_t>(restored.read(hf::status) | hf::access_error);
    require(value == error_status, "The vendor read-modify-write carries FCBEF/FCCF/FACCERR");
    restored.write({.address = hf::status, .value = value});
    auto expected = before;
    expected.errors = 0;
    require(restored.read(hf::status) == idle_status && restored.state() == expected &&
                restored.image() == image && restored.power_off_image() == image,
            "Error acknowledgement must preserve the complete idle state and all NVM knowledge");
    execute(restored, {});
    require(restored.state().buffer == Hcs08FlashBuffer::empty && !restored.state().active &&
                restored.state().errors == 0 && restored.image().fresh.count() == hf::page_bytes,
            "A new independently staged erase follows recovery, not the flag write itself");
    for (unsigned index = 0; index < hf::bytes; ++index) {
      const auto wanted = index < hf::page_bytes ? hf::erased : image.data[index];
      require(restored.image().known[index] && restored.image().data[index] == wanted,
              "Only the separately completed erase may modify its exact page");
    }
  }
  require(flash.read(hf::status) == error_status && flash.image() == initial(),
          "Recovery copies cannot alter the source error boundary");
}

void read_modify_write_recovery() {
  for (const auto agent : std::array{cpu, bdc}) {
    for (unsigned cause = 0; cause < 6; ++cause) { read_modify_write_case(agent, cause); }
  }
}

void read_modify_write_guards() {
  const auto refusal = [](Hcs08Flash &flash, const std::uint8_t value, const char *reason) {
    const auto state = flash.state();
    const auto image = flash.image();
    require(
        rejected([&] { flash.write({.address = hf::status, .value = value}); }).contains(reason) &&
            flash.state() == state && flash.image() == image,
        "An unqualified empty/active/occupied launch must preserve its complete boundary");
  };
  Hcs08Flash idle(initial(), contract);
  setup(idle);
  refusal(idle, error_status, "idle FCBEF");
  idle.write({.address = hf::command, .value = bad_command});
  refusal(idle, idle_status, "idle FCBEF");
  idle.write({.address = hf::status, .value = hf::access_error});
  execute(idle, {});
  launch(idle, {.opcode = hf::burst});
  idle.advance(transfer);
  idle.write({.address = hf::command, .value = bad_command});
  constexpr unsigned active_error = 0x90;
  refusal(idle, active_error, "empty-buffer launch during an executing command");
  Hcs08Flash pending(initial(), contract);
  setup(pending);
  launch(pending, {});
  pending.write({.address = hf::command, .value = bad_command});
  refusal(pending, active_error, "duplicate launch");
}

void security_recovery() {
  auto image = initial();
  image.data[hf::nv_options - hf::first] = secure_option;
  Hcs08Flash flash(image, contract);
  setup(flash);
  flash.context({.pc = ram_pc, .ccr = flags}, bdc);
  require(flash.read_array(example) == 0 && flash.read(hf::options) == option_secure,
          "Secured BDC reads return documented zero without exposing retained data");
  flash.stage({.address = example, .value = program_result});
  flash.write({.address = hf::command, .value = hf::program});
  require(flash.read(hf::status) == error_status && flash.image() == image,
          "A secured BDC program command sets FACCERR without array mutation");
  flash.write({.address = hf::status, .value = hf::access_error});
  flash.write({.address = hf::options, .value = 2});
  require(flash.read(hf::options) == option_secure, "Direct FOPT writes cannot unlock the target");
  execute(flash, {.opcode = hf::erase_all});
  require(flash.read(hf::options) == option_secure && flash.image().fresh.all(),
          "Mass erase creates known erased storage without silently unlocking latched SEC");
  execute(flash, {.opcode = hf::verify});
  require(flash.read(hf::status) == (idle_status | hf::blank) &&
              flash.read(hf::options) == option_unsecured &&
              flash.read_array(example) == hf::erased,
          "Only a successful full blank check disengages security until reset");
  flash.reset();
  require(flash.read(hf::options) == erased_options, "Erased NVOPT re-engages security on reset");
  setup(flash);
  flash.context({.pc = ram_pc, .ccr = flags}, bdc);
  execute(flash, {.opcode = hf::verify});
  execute(flash, {.address = hf::nv_options, .value = stored_options, .opcode = hf::program});
  flash.reset();
  require(flash.read(hf::options) == option_unsecured,
          "Programmed NVOPT, not a register patch, persists an unsecured restart");
}

void erase_history_and_clocks() {
  Hcs08Flash flash(initial(), contract);
  setup(flash);
  launch(flash, {.opcode = hf::program});
  require(rejected([&] { flash.advance(transfer); }).contains("successful erase"),
          "Unknown erase history cannot be inferred from known bytes or accepted as a program");
  flash.reset(reference_abort);
  setup(flash);
  execute(flash, {});
  execute(flash, {.value = hf::erased, .opcode = hf::program});
  launch(flash, {.value = hf::erased, .opcode = hf::program});
  require(rejected([&] { flash.advance(transfer); }).contains("reprogramming"),
          "Even writing FF consumes erase history; a second program is not safe bitwise AND");
  Hcs08Flash absent(initial(), contract);
  absent.context({.pc = ram_pc, .ccr = flags});
  absent.write({.address = hf::divider, .value = divisor});
  launch(absent, {});
  require(rejected([&] { absent.advance(transfer); }).contains("unknown operating clock"),
          "No automatic programming clock is invented");
  Hcs08Flash outside(initial(), contract);
  outside.context({.pc = ram_pc, .ccr = flags});
  outside.observe_bus_clock(bad_hz);
  outside.write({.address = hf::divider, .value = divisor});
  launch(outside, {});
  require(rejected([&] { outside.advance(transfer); }).contains("outside150..200"),
          "A loaded divider is insufficient without a valid actual reference bus frequency");
}

void cut(const Hcs08Flash &flash, const unsigned first_unknown, const unsigned count) {
  const auto before = flash.image();
  const auto state = flash.state();
  const auto conservative = flash.power_off_image();
  const auto bounded = flash.power_off_image(reference_abort);
  const bool uncertain = state.active && state.active->command != hf::verify;
  const bool accepted =
      state.buffer == Hcs08FlashBuffer::launched && state.staged.command != hf::verify;
  require(conservative.known.count() == ((uncertain || accepted) ? 0U : before.known.count()),
          "Default power-loss retention does not guess a physical corruption bound");
  for (unsigned n = 0; n < hf::bytes; ++n) {
    const bool invalid = n >= first_unknown && n - first_unknown < count;
    require(
        bounded.data[n] == before.data[n] && bounded.known[n] == (before.known[n] && !invalid) &&
            bounded.fresh[n] == (before.fresh[n] && !invalid),
        "Explicit command-range hypothesis marks unknown data rather than inventing torn bytes");
  }
  Hcs08Flash restarted(bounded, contract);
  restarted.context({.pc = ram_pc, .ccr = flags});
  require(
      restarted.read(hf::divider) == 0 && restarted.read(hf::status) == idle_status &&
          restarted.image() == bounded,
      "Restart clears volatile command/divider state and retains exact image knowledge/history");
  if (count != 0) {
    const auto address = static_cast<std::uint16_t>(hf::first + first_unknown);
    require(rejected([&] { static_cast<void>(restarted.read_array(address)); }).contains("unknown"),
            "An interrupted byte/page cannot become an assumed FF value after restart");
  }
}

void power_loss_phases() {
  Hcs08Flash flash(initial(), contract);
  setup(flash);
  cut(flash, 0, 0);
  flash.stage({.address = example, .value = program_result});
  cut(flash, 0, 0);
  flash.write({.address = hf::command, .value = hf::erase_page});
  cut(flash, 0, 0);
  flash.write({.address = hf::status, .value = hf::empty});
  cut(flash, 0, 0);
  flash.advance(transfer);
  cut(flash, 0, hf::page_bytes);
  flash.advance(complete);
  cut(flash, 0, 0);
  launch(flash, {.opcode = hf::burst});
  cut(flash, 0, 0);
  flash.advance(transfer);
  cut(flash, example - erase_begin, 1);
  launch(flash, {.address = second, .opcode = hf::burst});
  cut(flash, example - erase_begin, 1);
  flash.advance(complete);
  cut(flash, 0, 0);
  flash.advance(transfer);
  cut(flash, second - erase_begin, 1);
  flash.advance(complete);
  cut(flash, 0, 0);
  auto interrupted = flash;
  launch(interrupted, {.address = hf::nv_options, .opcode = hf::erase_page});
  interrupted.advance(transfer);
  const auto image = interrupted.power_off_image(reference_abort);
  Hcs08Flash restarted(image, contract);
  require(
      rejected([&] { static_cast<void>(restarted.read(hf::options)); }).contains("unknown reset"),
      "Power loss in the option/vector page leaves reset latches unknown, not secured or unlocked "
      "guesses");
}
} // namespace

int main() {
  try {
    read_modify_write_recovery();
    read_modify_write_guards();
    security_recovery();
    erase_history_and_clocks();
    power_loss_phases();
    std::cout
        << "HCS08 FLASH security, erase history, clock rejection and power-loss phases passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
