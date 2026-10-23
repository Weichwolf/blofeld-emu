// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_system_options.hpp"
#include "hcs08_flash_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_nvm;
constexpr unsigned idle_status = 0xc0, errors = 0x30, protected_status = 0xe0;
constexpr unsigned next_data = 0x33, invalid_command = 0x26;
constexpr unsigned options_readback = 0xc2, stored_result = 0x5a;

void registers() {
  require(rejected([&] { const Hcs08Flash flash(initial()); }).contains("explicit candidate"),
          "An unidentified controller cannot silently select a GT8A peripheral");
  Hcs08Flash flash(initial(), contract);
  setup(flash);
  require(flash.read(hf::divider) == (hf::empty | divisor) &&
              flash.read(hf::options) == options_readback &&
              flash.read(hf::protection) == hf::erased && flash.read(hf::status) == idle_status,
          "Independent reset configuration and first-write DIVLD state");
  flash.write({.address = hf::divider, .value = 0});
  flash.write({.address = hf::options, .value = 3});
  flash.write({.address = hf::protection, .value = 0});
  require(flash.read(hf::divider) == (hf::empty | divisor) &&
              flash.read(hf::options) == options_readback &&
              flash.read(hf::protection) == hf::erased,
          "FCDIV is write-once; CPU FOPT/FPROT stores are ignored, not applied to NVM");
  const auto before = flash.state();
  const auto image = flash.image();
  const auto error = rejected([&] { flash.write({.address = hf::status, .value = hf::erased}); });
  require(error.contains("idle FCBEF") && error.contains("address=0x1825") &&
              error.contains("pc=0x00a0") && error.contains("agent=CPU") &&
              flash.state() == before && flash.image() == image,
          "The original idle FF write remains a visible unresolved default, without mutation");
  require(rejected([&] {
            flash.write({.address = hf::configuration, .value = hf::violation});
          }).contains("KEYACC"),
          "Unsupported key-comparison behavior cannot report success");
  require(rejected([&] { static_cast<void>(flash.read(hf::divider + 2)); }).contains("unsupported"),
          "Reserved addresses are not a silently successful register bank");
  Hcs08Flash hypothesis(initial(), contract, Hcs08IdleLaunch::reference_ignore_empty);
  setup(hypothesis);
  hypothesis.write({.address = hf::status, .value = hf::erased});
  require(hypothesis.read(hf::status) == idle_status && hypothesis.image() == image,
          "Only the explicit empty-buffer hypothesis permits the original idle write");
}

void command_pipeline() {
  Hcs08Flash flash(initial(), contract);
  setup(flash);
  const auto original = flash.image();
  launch(flash, {});
  require(flash.read(hf::status) == 0 && flash.image() == original,
          "Launch clears FCCF/FCBEF without instantly erasing memory");
  flash.advance(transfer);
  for (unsigned n = 0; n < 16; ++n) {
    require(flash.read(hf::status) == 0, "Status polling cannot advance an erase");
  }
  flash.advance(complete);
  require(flash.read(hf::status) == idle_status && flash.image().fresh.count() == hf::page_bytes,
          "Only observed erase completion makes a known fresh page");
  for (unsigned n = 0; n < hf::bytes; ++n) {
    const auto expected = n < hf::page_bytes ? hf::erased : original.data[n];
    require(flash.image().known[n] && flash.image().data[n] == expected,
            "Page erase accepts an interior address and changes exactly its512-byte page");
  }
  launch(flash, {.opcode = hf::burst});
  flash.advance(transfer);
  require(flash.read(hf::status) == hf::empty, "Executing burst exposes its successor buffer");
  launch(flash, {.address = second, .value = next_data, .opcode = hf::burst});
  require(flash.read(hf::status) == 0, "Queued successor retains low FCCF/FCBEF");
  const auto saved = flash;
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = saved;
    restored.advance(complete);
    require(restored.read(hf::status) == 0 &&
                restored.image().data[example - hf::first] == stored_result,
            "Finishing the first byte does not complete the still-queued successor");
    restored.advance(transfer);
    require(restored.read(hf::status) == hf::empty && restored.read(hf::command) == 0,
            "Only burst successors transfer; FCMD is architecturally read-zero");
    restored.advance(complete);
    require(restored.read(hf::status) == idle_status &&
                restored.image().data[second - hf::first] == next_data &&
                restored.image().fresh.count() == hf::page_bytes - 2,
            "Two independent pipeline restorations commit both exact bytes and erase history");
  }
}

void protection_matrix() {
  for (unsigned protection = 0; protection <= hf::erased; ++protection) {
    for (unsigned page = 0; page < hf::bytes / hf::page_bytes; ++page) {
      Hcs08Flash flash(initial(), contract);
      setup(flash);
      flash.context({.pc = ram_pc, .ccr = flags}, bdc);
      flash.write({.address = hf::protection, .value = static_cast<std::uint8_t>(protection)});
      flash.context({.pc = ram_pc, .ccr = flags}, cpu);
      flash.write({.address = hf::protection, .value = hf::erased});
      const unsigned address = hf::first + (page * hf::page_bytes);
      const unsigned end_unprotected = ((protection >> 1U) + 1) * hf::page_bytes;
      const bool protected_page = (protection & 1U) == 0 && address >= end_unprotected;
      launch(flash, {.address = static_cast<std::uint16_t>(address)});
      require(flash.read(hf::status) == (protected_page ? protected_status : 0) &&
                  flash.read(hf::protection) == protection,
              "All256 FPROT values and16 pages follow the independent512-byte protection boundary");
    }
  }
}

void sequence_errors() {
  for (unsigned violation = 0; violation < 6; ++violation) {
    Hcs08Flash flash(initial(), contract);
    setup(flash);
    flash.stage({.address = example, .value = stored_result});
    switch (violation) {
      case 0: flash.stage({.address = second, .value = stored_result}); break;
      case 1: flash.write({.address = hf::command, .value = invalid_command}); break;
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
      default: throw Fault("Invalid own sequence case");
    }
    require(flash.read(hf::status) == (idle_status | hf::access_error) &&
                flash.state().buffer == Hcs08FlashBuffer::empty && flash.image() == initial(),
            "Six independently specified sequence violations latch FACCERR without data writes");
    flash.write({.address = hf::status, .value = errors});
    require(flash.read(hf::status) == idle_status, "Only actual W1C clears the protocol error");
  }
  Hcs08Flash unloaded(initial(), contract);
  unloaded.context({.pc = ram_pc, .ccr = flags});
  unloaded.stage({.address = example});
  unloaded.write({.address = hf::divider, .value = divisor});
  require(unloaded.read(hf::divider) == 0 &&
              unloaded.read(hf::status) == (idle_status | hf::access_error),
          "FACCERR blocks the initial FCDIV write until explicitly cleared");
}

void system_options() {
  constexpr unsigned programmed = 0x53;
  Hcs08SystemOptions options(contract);
  require(options.read() == Hcs08SystemOptions::reset_value && !options.written(),
          "SOPT reset configuration is distinct from a running watchdog model");
  options.write({.address = Hcs08SystemOptions::address, .value = programmed}, {.pc = ram_pc});
  options.write({.address = Hcs08SystemOptions::address, .value = 0}, {.pc = ram_pc});
  require(options.read() == programmed && options.written(),
          "Only the first SOPT write is honored");
  options.reset();
  require(rejected([&] {
            options.write({.address = Hcs08SystemOptions::address, .value = 0}, {.pc = ram_pc});
          }).contains("reserved-bit"),
          "Unqualified SOPT reserved values cannot silently choose a register value");
}
} // namespace

int main() {
  try {
    registers();
    command_pipeline();
    protection_matrix();
    sequence_errors();
    system_options();
    std::cout
        << "HCS08 FLASH registers, sequencing, burst pipeline and4096 protection cases passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
