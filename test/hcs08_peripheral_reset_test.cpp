// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/hcs08_flash.hpp"
#include "blofeld/hcs08_icg.hpp"
#include "blofeld/hcs08_system_options.hpp"
#include "hcs08_flash_fixture.hpp"
#include "hcs08_programmer_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08_nvm;
namespace hp = blofeld::test::hcs08_programmer;
namespace hi = hcs08_icg;
constexpr unsigned delay = 5, trim = 0x5a, first_control = 0x28, second_control = 0x30;
constexpr unsigned before_options = 0x53, after_options = 0x73, idle_status = 0xc0;
constexpr unsigned later_retirements = 32, page_count = 512;
constexpr unsigned transfer_at = 8, invalid_retention_code = 255;
constexpr auto conservative = Hcs08FlashAbort::unqualified_retention;
constexpr auto bounded = Hcs08FlashAbort::reference_command_range;

hp::Memory prepared(const unsigned phase) {
  hp::Memory memory(Hcs08IdleLaunch::unqualified, delay);
  memory.begin_instruction({.pc = ram_pc, .ccr = flags});
  memory.write(Hcs08SystemOptions::address, before_options);
  memory.write(hi::trim, trim);
  memory.write(hi::control1, first_control);
  memory.write(hi::control2, second_control);
  memory.write(hf::divider, divisor);
  memory.write(example, example_data);
  memory.write(hf::command, hf::erase_page);
  if (phase != 0) {
    memory.write(hf::status, hf::empty);
    memory.retire_instruction();
  }
  if (phase == 2) {
    while (memory.retired < transfer_at) { memory.retire_instruction(); }
    require(memory.flash.state().active.has_value(),
            "Finite reference schedule reaches an actual active command");
  }
  require(memory.options.written() && memory.options.read() == before_options &&
              memory.icg.state().trim == trim && memory.icg.state().control1_written &&
              memory.icg.state().control2_written,
          "Actual fixture bus stores establish all peripheral pre-reset state");
  require((phase == 0 && !memory.transfer_due && !memory.complete_due) ||
              (phase == 1 && memory.transfer_due && !memory.complete_due) ||
              (phase == 2 && !memory.transfer_due && memory.complete_due),
          "Reference scheduler arms a real command transfer or completion in the selected phase");
  return memory;
}

void reset_and_check(hp::Memory &memory, const Hcs08FlashAbort retention, const unsigned phase) {
  const auto image = memory.flash.image();
  const auto ram = memory.bytes;
  const auto known = memory.known;
  const auto retired = memory.retired;
  const auto transfers = memory.transfers;
  const auto completions = memory.completions;
  memory.reset_peripherals(retention);
  require(!memory.transfer_due && !memory.complete_due &&
              memory.flash.state() == Hcs08FlashState{} && memory.flash.read(hf::divider) == 0 &&
              memory.flash.read(hf::status) == idle_status,
          "Real peripheral reset drops both scheduled events and volatile FLASH state");
  require(memory.options.read() == Hcs08SystemOptions::reset_value && !memory.options.written() &&
              memory.icg.state() == Hcs08IcgState{.trim = trim},
          "Reset reopens SOPT/ICG first stores, restores filter/control and retains actual trim");
  const auto reset_image = memory.flash.image();
  for (unsigned n = 0; n < hf::bytes; ++n) {
    const bool uncertain =
        phase != 0 && (retention == conservative || (phase == 2 && n < page_count));
    require(reset_image.data[n] == image.data[n] &&
                reset_image.known[n] == (image.known[n] && !uncertain) &&
                reset_image.fresh[n] == (image.fresh[n] && !uncertain),
            "Independent phase/retention expectations preserve bytes and exact NVM knownness");
  }
  for (unsigned n = 0; n < later_retirements; ++n) { memory.retire_instruction(); }
  require(memory.flash.image() == reset_image && memory.flash.state() == Hcs08FlashState{} &&
              memory.bytes == ram && memory.known == known &&
              memory.retired == retired + later_retirements && memory.transfers == transfers &&
              memory.completions == completions && !memory.transfer_due && !memory.complete_due,
          "Further retirements cannot execute canceled jobs or manufacture RAM/FLASH changes");
  memory.write(Hcs08SystemOptions::address, after_options);
  memory.write(Hcs08SystemOptions::address, before_options);
  require(memory.options.written() && memory.options.read() == after_options,
          "The first post-reset SOPT store is honored and the second remains ignored");
}

void phases_and_restores() {
  for (unsigned phase = 0; phase < 3; ++phase) {
    for (const auto retention : std::array{conservative, bounded}) {
      const auto saved = prepared(phase);
      for (unsigned replay = 0; replay < 2; ++replay) {
        auto restored = saved;
        reset_and_check(restored, retention, phase);
        require(saved.options.read() == before_options && saved.icg.state().trim == trim &&
                    saved.icg.state().control1_request == first_control,
                "A fresh reset continuation does not reset its saved source owner");
      }
    }
  }
}

void logical_vector_boundary() {
  auto memory = prepared(2);
  const auto state = memory.flash.state();
  const auto image = memory.flash.image();
  const auto clock = memory.icg.state();
  const auto due = memory.complete_due;
  memory.begin_reset({.pc = ram_pc, .ccr = flags});
  require(rejected([&] { memory.reset_peripherals(); }).contains("during CPU vector transaction") &&
              memory.resetting && memory.flash.state() == state && memory.flash.image() == image &&
              memory.icg.state() == clock && memory.complete_due == due &&
              memory.options.read() == before_options,
          "The logical CPU vector transaction is not a second physical peripheral reset");
  memory.end_reset();
  constexpr auto invalid = static_cast<Hcs08FlashAbort>(invalid_retention_code);
  require(rejected([&] { memory.reset_peripherals(invalid); }).contains("invalid retention") &&
              memory.flash.state() == state && memory.flash.image() == image &&
              memory.icg.state() == clock && memory.complete_due == due &&
              memory.options.read() == before_options,
          "Invalid retention refuses before changing any other peripheral or scheduled event");
}
} // namespace

int main() {
  try {
    phases_and_restores();
    logical_vector_boundary();
    std::cout << "HCS08 peripheral reset: three command phases, two retention contracts, "
                 "12 fresh continuations and vector/invalid-retention controls passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
