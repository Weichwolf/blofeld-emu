// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_reset.hpp"
#include "mcu_flash_live_fixture.hpp"
#include "storage/flash_image.hpp"
#include "storage/private_file.hpp"
#include "storage/virtual_device_state.hpp"

#include <array>
#include <charconv>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <unistd.h>

namespace {
using namespace live;
constexpr unsigned read_entry = 0x8800;

blofeld::McuFlashImage worker_image() {
  auto result = image(helper({}), {});
  // Newly authored LDAA $A000; JMP $8000. S12CPUV2 Rev4,p214: B6 hh ll,
  // N/Z follow the loaded byte,V clears,S/X/H/I/C stay unchanged.
  constexpr std::array<std::uint8_t, 6> startup{0xb6, 0xa0, 0, 6, 0x80, 0};
  for (unsigned i = 0; i < startup.size(); ++i) {
    put(result, {.address = read_entry + i, .value = startup[i]});
  }
  constexpr unsigned reset_vector = 0xfffe;
  put(result, {.address = reset_vector, .value = read_entry >> 8U});
  put(result, {.address = reset_vector + 1, .value = 0});
  result.validate();
  return result;
}

unsigned number(const std::string_view text) {
  unsigned result = 0;
  const auto converted = std::from_chars(text.data(), text.data() + text.size(), result);
  require(converted.ec == std::errc{} && converted.ptr == text.data() + text.size(),
          "Expected an unsigned directed case index");
  return result;
}

struct Stop {
  std::string_view mode;
  unsigned publication;
  unsigned phase;
};

void boot(blofeld::McuMachine &machine) {
  machine.bus().set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  machine.advance_system_reset(reset_edges, blofeld::McuResetPad::high);
  require(!machine.system_reset_active() && machine.cpu().state().pc == read_entry,
          "Cold native POR fetches the persisted own boot ROM without state injection");
}

void inspect(const std::filesystem::path &path) {
  const blofeld::McuFlashImage unused;
  auto owner = std::make_shared<blofeld::VirtualDeviceState>(path, unused, []() -> std::string {
    throw blofeld::Fault("Cold reopen must never provision or replace existing device data");
  });
  const auto expected = owner->image();
  blofeld::McuMachine machine(expected, BLOFELD_DEFAULT_SLA, contract(), {3});
  machine.attach_flash_store(owner);
  boot(machine);
  constexpr unsigned offset = target - entry;
  if (expected.known.test(offset)) {
    const auto before = machine.cpu().state();
    const auto step = machine.cpu().step();
    const auto after = machine.cpu().state();
    constexpr unsigned preserved = 0xf1;
    constexpr unsigned negative = 8;
    constexpr unsigned zero = 4;
    constexpr unsigned sign = 0x80;
    const unsigned value = expected.bytes[offset];
    const auto flags =
        (before.ccr & preserved) | ((value & sign) != 0 ? negative : 0U) | (value == 0 ? zero : 0U);
    require(step.instruction_executed && !step.interrupt_vector && after.pc == read_entry + 3 &&
                (after.d >> 8U) == value && (after.d & byte_mask) == (before.d & byte_mask) &&
                after.ccr == flags,
            "Cold guest LDAA reads durable data and matches independent S12CPUV2 flags/registers");
    machine.cpu().step();
    require(machine.cpu().state().pc == entry,
            "The own startup JMP reaches the original test boot");
  } else {
    const auto error = rejected([&] { machine.cpu().step(); });
    require(error.contains("MCU flash read at 0xa000") && error.contains("pc=0x8800") &&
                error.contains("Uninitialized or unmapped MCU read at 0xa000") &&
                machine.timing_faulted(),
            "Cold native reads preserve the uncertain word instead of fabricating a value");
  }
  std::cout << "cold native POR known=" << expected.known.test(offset)
            << " fresh=" << expected.fresh.test(offset / 2) << " pc=" << machine.cpu().state().pc
            << '\n';
}

void run(const std::filesystem::path &path, const Stop stop) {
  const auto algorithm = helper({});
  auto owner = std::make_shared<blofeld::VirtualDeviceState>(
      path, worker_image(), [] { return "EMULIVE00001"; });
  blofeld::write_private_file(path.string() + ".initial",
                              blofeld::encode_flash_image(owner->image()));
  blofeld::McuMachine machine(owner->image(), BLOFELD_DEFAULT_SLA, contract(), {3});
  machine.attach_flash_store(owner);
  boot(machine);
  unsigned publications = 0;
  bool hit = false;
  owner->set_observer([&](const blofeld::PrivateFilePhase phase) {
    if (phase == blofeld::PrivateFilePhase::created) { ++publications; }
    if (publications != stop.publication || static_cast<unsigned>(phase) != stop.phase) { return; }
    hit = true;
    if (stop.mode == "fault") {
      throw blofeld::Fault("Own injected native file publication fault");
    }
    std::cout << "READY pid=" << getpid() << " publication=" << publications
              << " phase=" << static_cast<unsigned>(phase) << '\n'
              << std::flush;
    // The parent keeps this pipe open, observes the file boundary, then sends
    // and verifies actual SIGSTOP before SIGKILL. No emulated time passes here.
    static_cast<void>(std::cin.get());
    throw blofeld::Fault("A directed crash worker was unexpectedly resumed");
  });
  std::string failure;
  try {
    const auto end = ram + algorithm.size() - 2;
    unsigned steps = 0;
    while (machine.cpu().state().pc != end && ++steps < instruction_limit) { machine.cpu().step(); }
  } catch (const blofeld::Fault &error) { failure = error.what(); }
  require(stop.mode == "fault" && hit &&
              failure.contains("Own injected native file publication fault") &&
              failure.contains("pc=") && failure.contains("attempted-ticks=") &&
              machine.timing_faulted(),
          "A real publication failure must retain its original cause and native execution context");
  require(rejected([&] { static_cast<void>(owner->image()); }).contains("reopen"),
          "The file owner remains quarantined after publication failure");
  require(
      rejected([&] { static_cast<void>(machine.bus().read(status_register)); }).contains("reopen"),
      "Neither ready flag can be acknowledged after an uncertain file publication");
  require(rejected([&] {
            static_cast<void>(machine.bus().power_off_flash_image());
          }).contains("reopen"),
          "End-of-budget export cannot overwrite uncertain disk state with stale modeled cells");
  require(rejected([&] {
            machine.begin_system_reset(blofeld::McuResetSource::external, 3);
          }).contains("reopen"),
          "A hardware reset cannot launder a failed external writer lease");
  static_cast<void>(rejected([&] { machine.cpu().step(); }));
  std::cout << "fault blocked publication=" << publications << " phase=" << stop.phase
            << " cause=" << failure << '\n';
}
}

int main(const int argc, char **argv) {
  try {
    require(argc == 5, "Expected path,mode,publication and file phase");
    const std::filesystem::path path(argv[1]);
    const std::string_view mode(argv[2]);
    const auto publication = number(argv[3]);
    const auto phase = number(argv[4]);
    if (mode == "read") {
      inspect(path);
    } else {
      require((mode == "kill" || mode == "fault") && publication >= 1 && publication <= 2 &&
                  phase < 5,
              "Expected a predetermined native program publication boundary");
      run(path, {.mode = mode, .publication = publication, .phase = phase});
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
