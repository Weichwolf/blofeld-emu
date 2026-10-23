// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_flash.hpp"
#include "flash_stop_fixture.hpp"

#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

namespace {
using namespace blofeld;
using namespace blofeld::test::flash_stop;
constexpr unsigned helper_base = 0x1200, helper_bytes = 632, helper_entry = 0x13c0;
constexpr unsigned poll = 0x13ab, erased_read = 0x13e2, caller = 0x1600, stack = 0x1ff0;
constexpr unsigned interrupts_masked_stop_enabled = 0x50, jsr = 0x16, limit = 2048;

struct Run {
  Run(const Firmware &firmware, const bool erase_case, std::ostream &output)
      : source(firmware),
        bus(firmware.mcu),
        cpu(bus, BLOFELD_DEFAULT_SLA),
        trace(output),
        erasing(erase_case) {
    configure(bus);
    if (!erasing) { erase_sector(bus, sector); }
    before = bus.power_off_flash_image();
    const auto &helper = source.update_blocks.at(4);
    require(((static_cast<unsigned>(helper.at(0)) << 8U) | helper.at(1)) == helper_bytes,
            "The pinned original object supplies all632 unchanged helper bytes");
    for (unsigned i = 0; i < helper_bytes; ++i) {
      bus.write(static_cast<std::uint16_t>(helper_base + i), helper.at(i + 2));
    }
    bus.write(caller, jsr);
    bus.write(caller + 1, helper_entry >> 8U);
    bus.write(caller + 2, helper_entry & byte_mask);
    bus.write_memory({.address = stack, .size = 2, .value = erasing ? sector : target});
    cpu.set_state(
        {.pc = caller, .d = word_data, .sp = stack, .ccr = interrupts_masked_stop_enabled});
    bus.set_observer([this](const BusEvent &event) {
      if (!event.write) { return; }
      if (event.address == command) { opcode = event.value; }
      if (event.address == status && event.value == launch_bit) {
        ++launches;
        pending = true;
      }
    });
  }

  void abort() {
    require(launches == 1 && !pending && opcode == (erasing ? erase : program),
            "The original helper staged/transferred its own expected command before STOP");
    enter(bus);
    resume(bus);
    require(bus.read(status) == (idle | access_error),
            "The explicit nominal observation supplies CCIF/ACCERR and wake CBEIF, not success");
    stopped = true;
  }

  void execute() {
    while (cpu.state().pc != caller + 3) {
      require(steps < limit,
              "Original flash STOP experiment exceeded its native instruction bound");
      const auto state = cpu.state();
      if (state.pc == poll) {
        ++polls;
        if (polls == 1) {
          require(bus.read(status) == launch_bit,
                  "The original first CCIF poll observes a real busy command");
        } else if (polls == 2) {
          abort();
        }
      }
      trace << "pc=" << hex(state.pc) << " d=" << hex(state.d) << ' ' << cpu.disassemble() << '\n';
      try {
        cpu.step();
      } catch (const Fault &error) {
        require(
            erasing && stopped && state.pc == erased_read &&
                std::string(error.what()).contains("Uninitialized or unmapped MCU read at 0x8400"),
            "Only the native erased-word read may stop the interrupted sector case");
        faulted = true;
        trace << "FAULT " << error.what() << '\n';
        break;
      }
      if (pending) {
        bus.advance_flash(FlashMilestone::transfer_buffer);
        pending = false;
      }
      ++steps;
    }
    require(stopped && polls == 2 && launches == 1,
            "Both native cases reach the declared active STOP cut exactly once");
    if (erasing) {
      require(
          faulted && bus.read(status) == idle,
          "After erase abort, original code clears errors but cannot invent the lost sector word");
    } else {
      require(!faulted && cpu.state().sp == stack && (cpu.state().d & byte_mask) == 1 &&
                  bus.read(status) == (idle | access_error),
              "Original CCIF-only program poll returns1 despite ACCERR and lost target data");
    }
  }

  void verify_image() {
    const auto actual = bus.power_off_flash_image();
    const auto first = erasing ? sector - base : target - base;
    const auto count = erasing ? sector_bytes : 2U;
    for (unsigned i = 0; i < array_bytes; ++i) {
      const bool lost = i >= first && i - first < count;
      require(actual.known[i] == (before.known[i] && !lost) &&
                  actual.bytes[i] == (lost ? 0 : before.bytes[i]) &&
                  actual.fresh[i / 2] == (before.fresh[i / 2] && !lost),
              "Independent full-image expectation preserves only the explicitly aborted extent as "
              "unknown");
    }
    const auto &helper = source.update_blocks.at(4);
    for (unsigned i = 0; i < helper_bytes; ++i) {
      require(bus.read(static_cast<std::uint16_t>(helper_base + i)) == helper.at(i + 2),
              "No original helper byte was patched during the STOP experiment");
    }
    if (!erasing) {
      require(rejected([&] {
                static_cast<void>(bus.read_memory({.address = target, .size = 2}));
              }).contains("Uninitialized or unmapped MCU read at 0x8434"),
              "A success return cannot make the interrupted programmed word readable");
    }
    trace << "STOP_CASE erase=" << erasing << " polls=2 commands=1 lost=" << count
          << " returned-one=" << !erasing << " native-unknown-read=" << faulted << '\n';
  }

  const Firmware &source;
  McuBus bus;
  Hc12 cpu;
  std::ostream &trace;
  bool erasing;
  McuFlashImage before;
  unsigned opcode = 0, launches = 0, polls = 0, steps = 0;
  bool pending = false, stopped = false, faulted = false;
};
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) { throw Fault("Usage: original-flash-stop-test firmware.zip private.trace"); }
    const auto firmware = Firmware::load(arguments[1]);
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    for (const bool erasing : {false, true}) {
      Run run(firmware, erasing, trace);
      run.execute();
      run.verify_image();
    }
    std::cout << "Original CCIF-only false success and strict erased-word failure after nominal "
                 "flash STOP verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
