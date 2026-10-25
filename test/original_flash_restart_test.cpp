// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"
#include "storage/flash_file.hpp"
#include "storage/flash_image.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <string_view>
#include <vector>

namespace {
constexpr unsigned array_base = 0x8000;
constexpr unsigned application = 0x8200;
constexpr unsigned recovery_receiver = 0x8123;
constexpr unsigned checksum_load = 0x80a3;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned instruction_limit = 200000;
constexpr unsigned protection_offset = 0x7f0d;
constexpr unsigned security_offset = 0x7f0f;
constexpr unsigned vector_offset = 0x7ffe;
constexpr unsigned erased_vector = 0xffff;
constexpr unsigned flash_status = 0x105;
constexpr unsigned idle_flash = 0xc0;
constexpr unsigned ram_begin = 0x1200;
constexpr unsigned fixture_inputs = 0xfb;
constexpr unsigned receiver_entry = 0x1200;
constexpr unsigned receiver_poll = 0x8118;
constexpr unsigned receiver_read = 0x811c;
constexpr unsigned receiver_rx_mask = 16;
constexpr unsigned receiver_divider = 60;
constexpr unsigned receiver_instruction_limit = 100000;
constexpr unsigned first_helper_bytes = 761;
constexpr unsigned wire_byte_limit = 65536;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void require_fault(const auto &operation, const std::string_view expected, std::ostream &trace) {
  try {
    operation();
  } catch (const blofeld::Fault &fault) {
    trace << "observed-fault " << fault.what() << '\n';
    require(std::string_view(fault.what()).contains(expected),
            "Restart must fail for the expected unknown flash cell, not an unrelated access");
    trace << "expected-fault " << fault.what() << '\n';
    return;
  }
  throw blofeld::Fault("Restart unexpectedly accepted unknown persistent flash");
}

void check_loader(const blofeld::McuFlashImage &image, const blofeld::Firmware &firmware) {
  for (unsigned offset = 0; offset < application - array_base; ++offset) {
    require(image.known.test(offset) && image.bytes[offset] == firmware.mcu.at(offset),
            "Every resident-loader byte remains known and unchanged in the saved NVM");
  }
}

void execute_boot(blofeld::McuBus &bus,
                  blofeld::Hc12 &cpu,
                  const blofeld::McuFlashImage &image,
                  const std::string_view expected,
                  std::ostream &trace) {
  bool phy_sampled = false;
  unsigned instructions = 0;
  while (cpu.state().pc != application && cpu.state().pc != recovery_receiver) {
    require(instructions < instruction_limit,
            "Original cold boot must reach an explicit outcome within its instruction budget");
    const auto pc = cpu.state().pc;
    if (pc == phy_poll && !phy_sampled) {
      bus.observe_phy({.edges = 15});
      phy_sampled = true;
      trace << "phy-window pc=" << blofeld::hex(pc) << " edges=15\n";
    }
    try {
      cpu.step();
    } catch (const blofeld::Fault &fault) {
      trace << "observed-boot-fault instructions=" << instructions << ' ' << fault.what() << '\n';
      require(expected == "checksum-unknown" && pc == checksum_load,
              "Only the expected original checksum load may reject unknown application data");
      const std::string_view message(fault.what());
      unsigned unknown = application - array_base;
      constexpr unsigned checksum_limit = 0x7f00;
      while (unknown < checksum_limit && image.known.test(unknown)) { ++unknown; }
      require(unknown < checksum_limit && message.contains(blofeld::hex(array_base + unknown)) &&
                  message.contains("Uninitialized or unmapped MCU read") &&
                  message.contains("pc=0x80a3") && message.contains("ccr="),
              "Checksum fault retains the memory operation and original CPU context");
      trace << "expected-checksum-fault instructions=" << instructions << ' ' << message << '\n';
      return;
    }
    ++instructions;
  }
  const auto state = cpu.state();
  require((expected == "application" && state.pc == application) ||
              (expected == "recovery" && state.pc == recovery_receiver),
          "Original checksum/control flow must select the independently expected boot path");
  trace << "boot-outcome instructions=" << instructions << " pc=" << blofeld::hex(state.pc)
        << " sp=" << blofeld::hex(state.sp) << " d=" << blofeld::hex(state.d) << '\n';
}

std::vector<std::uint8_t> read_wire(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  require(input.is_open(), "Cannot open private recovery wire input");
  std::vector<std::uint8_t> wire;
  char byte = 0;
  while (input.get(byte)) {
    require(wire.size() < wire_byte_limit, "Recovery wire fixture exceeds its fixed input limit");
    wire.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(byte)));
  }
  require(input.eof() && !input.bad(), "Cannot completely read private recovery wire input");
  return wire;
}

struct WireLevel {
  bool high = true;
  unsigned cycles = 0;
};

void drive_wire(blofeld::McuBus &bus, const WireLevel level) {
  bus.set_pin_drive(blofeld::McuPort::s,
                    {.mask = receiver_rx_mask,
                     .levels = static_cast<std::uint8_t>(level.high ? receiver_rx_mask : 0)});
  bus.advance_sci(level.cycles);
}

void receive_byte(blofeld::McuBus &bus, const std::uint8_t byte) {
  drive_wire(bus, {.cycles = 3 * receiver_divider});
  drive_wire(bus, {.high = false, .cycles = 16 * receiver_divider});
  for (unsigned bit = 0; bit < 8; ++bit) {
    drive_wire(bus, {.high = ((byte >> bit) & 1U) != 0, .cycles = 16 * receiver_divider});
  }
  drive_wire(bus, {.cycles = 16 * receiver_divider});
}

struct ReceiveRequest {
  std::span<const std::uint8_t> wire;
  std::span<const std::uint8_t> helper;
  std::string_view expected;
};

void verify_received_helper(blofeld::McuBus &bus,
                            const ReceiveRequest &request,
                            const std::size_t consumed) {
  bool matches = true;
  for (unsigned offset = 0; offset < request.helper.size(); ++offset) {
    matches =
        (bus.read(static_cast<std::uint16_t>(receiver_entry + offset)) == request.helper[offset]) &&
        matches;
  }
  if (request.expected == "receiver-transfer") {
    constexpr unsigned sysex_end = 0xf7;
    require(matches && consumed + 1 == request.wire.size() && request.wire.back() == sysex_end,
            "Original receiver loads every unchanged helper byte and jumps before the last F7");
  } else {
    require(request.expected == "receiver-mismatch" && !matches && consumed < request.wire.size(),
            "Reordered recovery data reaches the helper entry with incomplete/incorrect RAM");
  }
}

void execute_receiver(blofeld::McuBus &bus,
                      blofeld::Hc12 &cpu,
                      const ReceiveRequest &request,
                      std::ostream &trace) {
  require(request.helper.size() == first_helper_bytes &&
              bus.read(blofeld::hc12::sci_baud_high) == 0 &&
              bus.read(blofeld::hc12::sci_baud_low) == receiver_divider &&
              bus.read(blofeld::hc12::sci_control2) == 12,
          "Original boot code configures its own SCI for the receiver fixture");
  bool ready = true;
  std::size_t consumed = 0;
  unsigned empty_polls = 0;
  for (unsigned instructions = 0; instructions < receiver_instruction_limit; ++instructions) {
    const auto state = cpu.state();
    if (state.pc == receiver_entry) {
      verify_received_helper(bus, request, consumed);
      trace << "helper-entry instructions=" << instructions << " consumed=" << consumed
            << " wire-bytes=" << request.wire.size() << " sp=" << blofeld::hex(state.sp) << '\n';
      return;
    }
    if (state.pc == receiver_poll && ready) {
      if (consumed < request.wire.size()) {
        trace << "sci-frame index=" << consumed << " byte=" << blofeld::hex(request.wire[consumed])
              << '\n';
        receive_byte(bus, request.wire[consumed++]);
        ready = false;
      } else {
        ++empty_polls;
        if (empty_polls == 4) {
          require(request.expected == "receiver-wait" &&
                      (bus.read(blofeld::hc12::sci_status1) & 0x20U) == 0,
                  "Absent or unrecognized wire data leaves the original SCI receive poll waiting");
          trace << "waiting-for-wire consumed=" << consumed << " polls=" << empty_polls << '\n';
          return;
        }
      }
    }
    trace << "receiver-pc=" << blofeld::hex(state.pc) << " sp=" << blofeld::hex(state.sp)
          << " d=" << blofeld::hex(state.d) << '\n';
    cpu.step();
    if (state.pc == receiver_read) { ready = true; }
  }
  throw blofeld::Fault("Original recovery receiver exhausted its instruction budget");
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path image;
  std::filesystem::path trace;
  std::string_view expected;
  std::filesystem::path wire;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  const auto image = blofeld::load_flash_image(inputs.image);
  check_loader(image, firmware);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private original restart trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  trace << "cold-image " << inputs.image.filename() << " expected=" << inputs.expected << '\n';
  if (inputs.expected == "options-unknown") {
    const auto offset = image.known.test(protection_offset) ? security_offset : protection_offset;
    require(!image.known.test(offset), "Expected reset-option cell is unknown in the input file");
    require_fault([&image] { const blofeld::McuBus rejected(image); },
                  "Unknown physical MCU flash byte at " + blofeld::hex(offset),
                  trace);
  } else {
    blofeld::McuBus bus(image);
    require(bus.read(blofeld::hc12::flash_clock_divider) == 0 &&
                bus.read(flash_status) == idle_flash &&
                bus.read(blofeld::hc12::flash_security) == image.bytes[security_offset] &&
                bus.read(blofeld::hc12::flash_protection) == image.bytes[protection_offset],
            "A new process samples persistent options and starts the controller idle/unconfigured");
    require_fault([&bus] { static_cast<void>(bus.read(ram_begin)); },
                  "Uninitialized or unmapped MCU read",
                  trace);
    bus.set_pin_drive(blofeld::McuPort::e, {.mask = fixture_inputs});
    blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    if (inputs.expected == "vector-unknown") {
      require(!image.known.test(vector_offset), "The reset vector is unknown in the input file");
      require_fault([&cpu] { cpu.reset(); }, "MCU flash read at 0xfffe", trace);
    } else {
      cpu.reset();
      const auto vector = (static_cast<unsigned>(image.bytes[vector_offset]) << 8U) |
                          image.bytes[vector_offset + 1];
      require(cpu.state().pc == vector, "Native reset must fetch the saved vector without repair");
      trace << "reset-vector " << blofeld::hex(vector) << '\n';
      if (inputs.expected == "vector-erased") {
        require(vector == erased_vector,
                "An erased reset vector must not be replaced by the intact loader address");
      } else {
        require(vector == array_base,
                "Original boot execution requires the actual resident vector");
        const bool receiver_test = inputs.expected.starts_with("receiver-");
        execute_boot(bus, cpu, image, receiver_test ? "recovery" : inputs.expected, trace);
        if (receiver_test) {
          const auto wire = read_wire(inputs.wire);
          const auto &block = firmware.update_blocks.at(0);
          require(block.size() >= first_helper_bytes + 2 &&
                      ((static_cast<unsigned>(block.at(0)) << 8U) | block.at(1)) ==
                          first_helper_bytes,
                  "The original first helper retains its independently checked size");
          const auto helper = std::span<const std::uint8_t>(block).subspan(2, first_helper_bytes);
          execute_receiver(
              bus, cpu, {.wire = wire, .helper = helper, .expected = inputs.expected}, trace);
        }
      }
    }
  }
  std::cout << "Verified original cold-restart outcome=" << inputs.expected << '\n';
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count < 5 || count > 6) {
      throw blofeld::Fault(
          "Usage: original-flash-restart-test firmware.zip private.bfnvm private.trace outcome "
          "[private-wire.syx]");
    }
    return run({.firmware = arguments[1],
                .image = arguments[2],
                .trace = arguments[3],
                .expected = arguments[4],
                .wire = count == 6 ? arguments[5] : ""});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
