// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/flash_image.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/hc12_registers.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <algorithm>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace {
constexpr unsigned application = 0x8200;
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned strap_first = 0x8029;
constexpr unsigned strap_second = 0x8036;
constexpr unsigned boot_limit = 200000;
constexpr unsigned call_limit = 100000;
constexpr unsigned caller = 0x1700;
constexpr unsigned receiver = 0xbad9;
constexpr unsigned sample_pc = 0xbad3;
constexpr unsigned helper = 0x1200;
constexpr unsigned helper_return = 0xbb56;
constexpr unsigned service_table = 0xf81b;
constexpr unsigned first_service = 0x8d70;
constexpr unsigned helper_callback_return = 0x1209;
constexpr unsigned stack = 0x2600;
constexpr unsigned helper_stack = stack - 12;
constexpr unsigned original_first_bytes = 761;
constexpr unsigned original_main_bytes = 632;
constexpr unsigned scratch_bytes = 1024;
constexpr unsigned sentinel = 0xcd;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned data_mask = 0x3f;
constexpr unsigned external_mask = 0x7f;
constexpr unsigned strobe = 0x80;
constexpr unsigned update_mode = 12;
constexpr unsigned interrupt_masked = 0xd0;
constexpr unsigned pe_mask = 0xfb;
constexpr unsigned packed_word_bits = 24;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void boot(blofeld::McuBus &bus, blofeld::Hc12 &cpu, std::ostream &trace) {
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = pe_mask, .levels = 3});
  bus.set_pin_drive(blofeld::McuPort::q, {.mask = byte_mask});
  cpu.reset();
  bool phy = false;
  unsigned instructions = 0;
  while (cpu.state().pc != application) {
    require(instructions < boot_limit, "Original boot must reach its application entry");
    const auto pc = cpu.state().pc;
    if (pc == phy_poll && !phy) {
      bus.observe_phy({.edges = 15});
      phy = true;
    }
    if (pc == strap_first || pc == strap_second) {
      const auto output = bus.observe_pads(blofeld::McuPort::e, 4);
      require(output.known == 4, "Controlled E2/E3 loopback requires a known original output");
      const auto levels = static_cast<std::uint8_t>(3U | ((output.levels & 4U) != 0 ? 8U : 0U));
      bus.set_pin_drive(blofeld::McuPort::e, {.mask = pe_mask, .levels = levels});
    }
    cpu.step();
    ++instructions;
  }
  require(bus.read(blofeld::hc12::modrr) == 0 && cpu.state().sp == stack,
          "Original strap test retains 100-pin routing and completes its own stack/checksum");
  trace << "conditional-boot instructions=" << instructions << " modrr=0\n";
}

std::vector<std::uint8_t> encode(const std::span<const std::uint8_t> payload) {
  require(payload.size() < scratch_bytes, "Fixture payload must fit its isolated target window");
  std::vector<std::uint8_t> bytes{static_cast<std::uint8_t>(payload.size() >> 8U),
                                  static_cast<std::uint8_t>(payload.size() & byte_mask)};
  bytes.insert(bytes.end(), payload.begin(), payload.end());
  while (bytes.size() % 3 != 0) { bytes.push_back(0); }
  std::vector<std::uint8_t> samples;
  for (std::size_t index = 0; index < bytes.size(); index += 3) {
    const auto word = (static_cast<unsigned>(bytes[index]) << 16U) |
                      (static_cast<unsigned>(bytes[index + 1]) << 8U) | bytes[index + 2];
    for (unsigned remaining = packed_word_bits; remaining != 0; remaining -= 6) {
      samples.push_back(static_cast<std::uint8_t>((word >> (remaining - 6)) & data_mask));
    }
  }
  return samples;
}

void prepare(blofeld::McuBus &bus, blofeld::Hc12 &cpu) {
  for (unsigned offset = 0; offset < scratch_bytes; ++offset) {
    bus.write(static_cast<std::uint16_t>(helper + offset), sentinel);
  }
  constexpr unsigned jsr_extended = 0x16;
  bus.write(caller, jsr_extended);
  bus.write(caller + 1, receiver >> 8U);
  bus.write(caller + 2, receiver & byte_mask);
  bus.set_pin_drive(blofeld::McuPort::q, {.mask = external_mask, .levels = update_mode});
  cpu.set_state({.pc = caller, .sp = stack, .ccr = interrupt_masked});
}

void check_target(blofeld::McuBus &bus, const std::span<const std::uint8_t> expected) {
  for (unsigned offset = 0; offset < scratch_bytes; ++offset) {
    const auto value = offset < expected.size() ? expected[offset] : sentinel;
    require(bus.read(static_cast<std::uint16_t>(helper + offset)) == value,
            "Native copy writes exactly the independently expected prefix and preserves its tail");
  }
}

void check_entry(blofeld::McuBus &bus, const blofeld::Hc12 &cpu) {
  require(cpu.state().pc == helper && cpu.state().d == service_table &&
              cpu.state().sp == helper_stack &&
              bus.read_memory({.address = helper_stack, .size = 2}) == helper_return,
          "Original LDD/JSR supplies its real service-table argument and return frame");
}

struct Transfer {
  std::span<const std::uint8_t> payload;
  std::optional<std::size_t> loss_at = std::nullopt;
};

void transfer(blofeld::McuBus &bus,
              blofeld::Hc12 &cpu,
              const Transfer &request,
              std::ostream &trace) {
  prepare(bus, cpu);
  const auto samples = encode(request.payload);
  std::size_t consumed = 0;
  unsigned instructions = 0;
  while (cpu.state().pc != helper) {
    require(instructions < call_limit, "Original receiver must reach an explicit bounded outcome");
    const auto state = cpu.state();
    if (state.pc == sample_pc) {
      const auto output = bus.observe_pads(blofeld::McuPort::q, strobe);
      require(output.known == strobe && output.levels == (consumed % 2 == 0 ? strobe : 0),
              "Original receiver alternates its own Q7 strobe between six-bit observations");
      if (request.loss_at && consumed == *request.loss_at) {
        bus.set_pin_drive(blofeld::McuPort::q, {.mask = 0x40});
      } else {
        require(consumed < samples.size(), "Receiver requested data beyond the complete fixture");
        bus.set_pin_drive(blofeld::McuPort::q,
                          {.mask = external_mask, .levels = samples[consumed++]});
      }
    }
    trace << "pc=" << blofeld::hex(state.pc) << " d=" << blofeld::hex(state.d)
          << " sp=" << blofeld::hex(state.sp) << '\n';
    try {
      cpu.step();
    } catch (const blofeld::Fault &fault) {
      trace << "observed-fault " << fault.what() << '\n';
      const std::string_view message(fault.what());
      require(request.loss_at && consumed == *request.loss_at && state.pc == sample_pc &&
                  message.contains("Unknown MCU GPIO input at 0x0260 mask=0x3f") &&
                  message.contains("pc=0xbad3") && message.contains("ccr="),
              "Only the deliberately withdrawn parallel input may fail this transfer");
      const auto decoded = (consumed / 4) * 3;
      const auto copied = decoded > 2 ? decoded - 2 : 0;
      check_target(bus, request.payload.first(copied));
      trace << "input-loss samples=" << consumed << " copied=" << copied << '\n';
      return;
    }
    ++instructions;
  }
  require(!request.loss_at && consumed == samples.size(),
          "Complete input reaches the helper using the exact expected sample count");
  check_target(bus, request.payload);
  check_entry(bus, cpu);
  trace << "helper-entry bytes=" << request.payload.size() << " samples=" << consumed
        << " instructions=" << instructions << " return=" << blofeld::hex(helper_return) << '\n';
}

void check_first_callback(blofeld::McuBus &bus, blofeld::Hc12 &cpu, std::ostream &trace) {
  unsigned instructions = 0;
  while (cpu.state().pc != first_service) {
    require(instructions < 12, "First helper callback must resolve through the supplied table");
    trace << "callback-pc=" << blofeld::hex(cpu.state().pc) << '\n';
    cpu.step();
    ++instructions;
  }
  require(cpu.state().y == service_table && cpu.state().sp == helper_stack - 4 &&
              bus.read_memory({.address = helper_stack - 4, .size = 2}) == helper_callback_return,
          "Unchanged helper TFR and indirect JSR use the original function table and stack");
}

std::span<const std::uint8_t> original_helper(const std::vector<std::uint8_t> &block,
                                              const unsigned size) {
  require(block.size() >= size + 2 &&
              ((static_cast<unsigned>(block.at(0)) << 8U) | block.at(1)) == size,
          "Supplied original object retains its independently checked helper size");
  return std::span<const std::uint8_t>(block).subspan(2, size);
}

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  blofeld::McuBus bus(firmware.mcu);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private original parallel-update trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  boot(bus, cpu, trace);
  transfer(bus,
           cpu,
           {.payload = original_helper(firmware.update_blocks.at(0), original_first_bytes)},
           trace);
  check_first_callback(bus, cpu, trace);
  transfer(bus,
           cpu,
           {.payload = original_helper(firmware.update_blocks.at(4), original_main_bytes)},
           trace);
  for (const auto size : {0U, 1U, 2U, 3U, 4U, 5U, 8U, 31U, 255U, 256U, 511U, 512U}) {
    std::vector<std::uint8_t> bytes(size);
    constexpr unsigned pattern_stride = 73;
    for (unsigned index = 0; index < size; ++index) {
      bytes[index] = static_cast<std::uint8_t>((index * pattern_stride) & byte_mask);
    }
    transfer(bus, cpu, {.payload = bytes}, trace);
  }
  const std::vector<std::uint8_t> interrupted(64, byte_mask);
  for (unsigned cut = 0; cut < 12; ++cut) {
    transfer(bus, cpu, {.payload = interrupted, .loss_at = cut}, trace);
  }
  const auto image = bus.power_off_flash_image();
  require(std::ranges::equal(image.bytes, firmware.mcu) && image.known.all() && image.fresh.none(),
          "All original normal-copy cases preserve every MCU flash byte and its initial history");
  std::cout << "Original parallel loader: both original helpers, 12 synthetic lengths and 12 "
               "withdrawn-input positions checked; original caller/table/first callback verified. "
               "Conditional pin/entry fixture, no full DSP/MIDI update or calibrated board boot.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) {
      throw blofeld::Fault("Usage: original-parallel-update-test firmware.zip private.trace");
    }
    return run({.firmware = arguments[1], .trace = arguments[2]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
