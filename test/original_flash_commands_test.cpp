// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_file.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <string>
#include <utility>

namespace {
constexpr unsigned helper_base = 0x1200;
constexpr unsigned helper_bytes = 632;
constexpr unsigned helper_entry = 0x13c0;
constexpr unsigned helper_poll = 0x13ab;
constexpr unsigned caller = 0x1600;
constexpr unsigned stack = 0x1ff0;
constexpr unsigned divider = 0x100;
constexpr unsigned status = 0x105;
constexpr unsigned command = 0x106;
constexpr unsigned masked = 0xd0;
constexpr unsigned array_base = 0x8000;
constexpr unsigned application = 0x8200;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned word_mask = 0xffff;
constexpr unsigned instruction_budget = 1024;
constexpr unsigned program_command = 0x20;
constexpr unsigned erase_command = 0x40;
constexpr unsigned reference_divider = 0x48;
constexpr unsigned oscillator_hz = 12000000;
constexpr unsigned bus_hz = 24000000;
constexpr unsigned top_sector = 0xfe00;
constexpr unsigned security_word = 0xff0e;
constexpr unsigned security_field = 0xff0f;
constexpr unsigned security_register = 0x101;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void configure(blofeld::McuBus &bus) {
  bus.write(divider, reference_divider);
  bus.observe_flash_clock({.oscillator_hz = oscillator_hz, .bus_hz = bus_hz});
}

void load_helper(blofeld::McuBus &bus, const blofeld::Firmware &firmware) {
  const auto &main = firmware.update_blocks.at(4);
  const auto size = (static_cast<unsigned>(main.at(0)) << 8U) | main.at(1);
  require(size == helper_bytes, "Original helper retains its checked size and contents");
  for (std::size_t offset = 0; offset < size; ++offset) {
    bus.write(static_cast<std::uint16_t>(helper_base + offset), main.at(offset + 2));
  }
  // Own diagnostic caller: JSR to the original helper. Argument is at its caller's SP.
  constexpr unsigned jsr_extended = 0x16;
  bus.write(caller, jsr_extended);
  bus.write(caller + 1, helper_entry >> 8U);
  bus.write(caller + 2, helper_entry & byte_mask);
}

struct Call {
  std::uint16_t destination = application;
  std::uint16_t value = 0;
};

class Runner {
public:
  Runner(blofeld::McuBus &bus,
         blofeld::Hc12 &cpu,
         std::ostream &trace,
         std::filesystem::path images = {})
      : bus_(bus), cpu_(cpu), trace_(trace), images_(std::move(images)) {}

  void observe(const blofeld::BusEvent &event) {
    if (event.address == command) {
      require(event.value == program_command || event.value == erase_command,
              "Original helper chooses its documented word-program or sector-erase command");
      capture("selected", launches_ + 1);
    }
    if (event.address == status && event.value == 0x80) {
      transfer_ = true;
      ++launches_;
      capture("launched", launches_);
    }
  }

  std::uint8_t call(const Call input) {
    require(!transfer_ && polls_ == 0,
            "Each isolated call starts without pending reference transitions");
    bus_.write_memory({.address = stack, .size = 2, .value = input.destination});
    cpu_.set_state({.pc = caller, .d = input.value, .sp = stack, .ccr = masked});
    execute_until(caller + 3);
    return static_cast<std::uint8_t>(cpu_.state().d & byte_mask);
  }

  void finalize_vectors() {
    constexpr unsigned vector_entry = 0x12cb;
    constexpr unsigned before_usb = 0x12e0;
    cpu_.set_state({.pc = vector_entry, .sp = stack, .ccr = masked});
    const auto previous_launches = launches_;
    execute_until(before_usb);
    require(launches_ == previous_launches + 3,
            "Original finalization sequence performs its own three recovery-vector writes");
  }

  [[nodiscard]] unsigned launches() const { return launches_; }

private:
  void capture(const char *phase, const unsigned index) const {
    if (images_.empty()) { return; }
    const auto path = images_ / ("command-" + std::to_string(index) + "-" + phase + ".bfnvm");
    blofeld::save_flash_image(path, bus_.power_off_flash_image());
  }

  void execute_until(const std::uint16_t stop) {
    unsigned completed = 0;
    while (cpu_.state().pc != stop) {
      require(completed < instruction_budget,
              "Original helper must return through its own control flow");
      const auto before = cpu_.state();
      if (before.pc == helper_poll && polls_ != 0) {
        if (polls_ == 2) {
          bus_.advance_flash(blofeld::FlashMilestone::complete_command);
          capture("completed", launches_);
          polls_ = 0;
        } else {
          ++polls_;
        }
      }
      trace_ << "pc=" << blofeld::hex(before.pc) << " d=" << blofeld::hex(before.d)
             << " sp=" << blofeld::hex(before.sp) << ' ' << cpu_.disassemble() << '\n';
      cpu_.step();
      if (transfer_) {
        bus_.advance_flash(blofeld::FlashMilestone::transfer_buffer);
        capture("active", launches_);
        transfer_ = false;
        polls_ = 1;
      }
      ++completed;
    }
    require(cpu_.state().sp == stack && !transfer_ && polls_ == 0,
            "Native JSR/RTS and helper stack balance are preserved after every call");
  }

  blofeld::McuBus &bus_;
  blofeld::Hc12 &cpu_;
  std::ostream &trace_;
  std::filesystem::path images_;
  unsigned launches_ = 0;
  unsigned polls_ = 0;
  bool transfer_ = false;
};

void test_guards_and_program(blofeld::McuBus &bus, Runner &runner) {
  constexpr std::array<unsigned, 4> protected_addresses{0, 0x7ffe, 0x8000, 0x81fe};
  for (const auto address : protected_addresses) {
    require(runner.call({.destination = static_cast<std::uint16_t>(address)}) == 0,
            "Original software refuses addresses below the application boundary");
  }
  require(runner.launches() == 0, "Software bounds checks issue no flash commands");
  constexpr unsigned first_word = 0x1234;
  constexpr unsigned second_word = 0xabcd;
  require(runner.call({.value = first_word}) == 1 && runner.launches() == 2,
          "Original boundary call requests one sector erase and one word program");
  require(bus.read_memory({.address = application, .size = 2}) == first_word,
          "Executed original STX supplies the exact programmed word");
  require(runner.call({.destination = application + 2, .value = second_word}) == 1 &&
              runner.launches() == 3,
          "Next word in the erased sector requires only its own program command");
  require(runner.call({.destination = application + 2}) == byte_mask && runner.launches() == 3,
          "Original code rejects a non-erased word before any destructive command");
  require(bus.read_memory({.address = application + 2, .size = 2}) == second_word,
          "The rejected overwrite leaves prior programmed contents intact");
}

void test_security(blofeld::McuBus &bus, Runner &runner) {
  constexpr unsigned expected_word = 0xfffe;
  constexpr unsigned security_bits_base = 0xfffc;
  for (unsigned bits = 0; bits < 4; ++bits) {
    require(runner.call({.destination = top_sector, .value = word_mask}) == 1,
            "Original top-sector erase skips programming an all-ones word");
    require(runner.call({.destination = security_word,
                         .value = static_cast<std::uint16_t>(security_bits_base + bits)}) == 1,
            "Original security-word program returns through its normal success path");
    require(bus.read_memory({.address = security_word, .size = 2}) == expected_word,
            "All four supplied SEC combinations are forced to 10 by original CPU instructions");
    bus.reset();
    require(
        (bus.read(security_register) & 3U) == 2 && (bus.read(security_field) & 3U) == 2,
        "Reset latches the original helper's unsecure bits from actual emulated flash contents");
    configure(bus);
  }
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
  std::filesystem::path images;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  if (!inputs.images.empty()) { std::filesystem::create_directories(inputs.images); }
  blofeld::McuBus bus(firmware.mcu);
  load_helper(bus, firmware);
  configure(bus);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private original flash command trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Runner runner(bus, cpu, trace, inputs.images);
  bus.set_observer([&runner](const blofeld::BusEvent &event) { runner.observe(event); });
  test_guards_and_program(bus, runner);
  test_security(bus, runner);
  runner.finalize_vectors();
  constexpr std::array<unsigned, 3> recovery_vectors{0xfffa, 0xfffc, 0xfffe};
  for (const auto address : recovery_vectors) {
    require(bus.read_memory({.address = static_cast<std::uint16_t>(address), .size = 2}) ==
                array_base,
            "Original completion code restores all three recovery vectors to the resident loader");
  }
  bus.reset();
  cpu.reset();
  require(cpu.state().pc == array_base,
          "Native reset fetches the newly programmed original recovery vector");
  for (unsigned offset = 0; offset < application - array_base; ++offset) {
    require(bus.read(static_cast<std::uint16_t>(array_base + offset)) == firmware.mcu.at(offset),
            "Every original bootloader byte remains intact after all isolated update calls");
  }
  std::cout << "Original updater routines enforce the application boundary, erase/program words, "
            << "reject non-erased overwrites, force SEC=10 for all four source combinations and "
            << "restore the original recovery vectors; " << runner.launches()
            << " commands. Explicit clock/milestone fixtures; no timed full "
            << "transport, analog power-loss or stock-device recovery proof.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count < 3 || count > 4) {
      throw blofeld::Fault(
          "Usage: original-flash-commands-test firmware.zip private.trace [private-images-dir]");
    }
    return run({.firmware = arguments[1],
                .trace = arguments[2],
                .images = count == 4 ? arguments[3] : ""});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
