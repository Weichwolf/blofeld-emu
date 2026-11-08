// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "cpu12/memory.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "isa.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"
#include "vi/vi.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
namespace rt = blofeld::vi;
namespace isa = cpu12::isa;
namespace reg = blofeld::hc12;
using cpu12::Byte;
using isa::Opcode;
constexpr unsigned entry = 0x9000, flash_base = 0x8000, byte_mask = 0xff;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f;
constexpr unsigned source_value = 0x1200, reset_value = 0x1201;
constexpr unsigned before_release = 0x1202, after_release = 0x1203;
constexpr std::uint64_t timebase = 540000000, millisecond = timebase / 1000;
constexpr unsigned oscillator = 12000000;
constexpr unsigned direction_u = reg::ptu + 2;
constexpr Byte all_u_outputs = 0x3f, other_u_outputs = 0x3e;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw std::runtime_error(std::string(message)); }
}

constexpr std::array<Byte, 2> address(const unsigned value) {
  return {static_cast<Byte>(value >> 8U), static_cast<Byte>(value & byte_mask)};
}

McuFlashImage own_image(const std::span<const Byte> program) {
  std::array<Byte, McuFlashImage::byte_count> bytes;
  bytes.fill(isa::nop.front());
  bytes[protection] = byte_mask;
  constexpr Byte unsecured = 0xfe;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  require(program.size() < protection - (entry - flash_base),
          "Own code fits before security fields");
  std::ranges::copy(program, bytes.begin() + (entry - flash_base));
  return McuFlashImage::from_known(bytes);
}

rt::Config configuration() {
  rt::Config result;
  result.mcu = {.clock = {.oscillator_hz = oscillator,
                          .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = McuRunMemory::unstretched_internal};
  result.dsp_clock = {.extal_hz = oscillator,
                      .timebase_hz = timebase,
                      .switch_timing = DspClockSwitch::functional_region_boundary};
  result.dsp = {.reset_dsp_first = true,
                .initial_reset = DspResetInputs{
                    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  result.dma_grant = DspDmaGrant::available;
  result.mcu_u0_release = true;
  result.loopback = {
      .source = McuPort::e, .source_mask = 4, .destination = McuPort::e, .destination_mask = 8};
  result.initial = {{.port = McuPort::e, .drive = {.mask = byte_mask, .levels = 6}}};
  return result;
}

struct Fixture {
  Fixture(const std::span<const Byte> program, const rt::Config &config = configuration())
      : image(own_image(program)),
        instrument(config, {.image = image, .rom = rom, .specification = {}}) {}

  void until(const std::uint64_t ticks) {
    constexpr unsigned maximum_operations = 100000;
    for (unsigned count = 0; count < maximum_operations && instrument.common_ticks() < ticks;
         ++count) {
      instrument.advance();
    }
    require(instrument.common_ticks() >= ticks && !instrument.status().faulted &&
                instrument.status().held_reset && instrument.status().dsp_steps == 0,
            "Owned MCU I/O advances real device time while the unused DSP remains held in reset");
  }

  [[nodiscard]] Byte read(const unsigned location) const {
    const auto value = instrument.inspect_mcu(static_cast<std::uint16_t>(location));
    if (!value.value) {
      throw std::runtime_error("The native MCU did not store its observed byte");
    }
    return *value.value;
  }

  McuFlashImage image;
  std::vector<std::uint32_t> rom;
  rt::VirtualInstrument instrument;
};

constexpr auto polling_program() {
  constexpr auto prefix = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                       std::array<Byte, 1>{other_u_outputs},
                                       isa::opcode<Opcode::staa_extended>(),
                                       address(direction_u));
  return isa::program(prefix,
                      isa::opcode<Opcode::ldaa_extended>(),
                      address(reg::porte),
                      isa::opcode<Opcode::staa_extended>(),
                      address(source_value),
                      isa::opcode<Opcode::ldaa_extended>(),
                      address(reg::ptu),
                      isa::opcode<Opcode::staa_extended>(),
                      address(reset_value),
                      isa::opcode<Opcode::jmp_extended>(),
                      address(entry + prefix.size()));
}

void input_changes_and_destination_replacement() {
  auto config = configuration();
  config.events = {
      {.ticks = 2 * millisecond, .port = McuPort::e, .drive = {.mask = byte_mask, .levels = 2}},
      {.ticks = 4 * millisecond, .port = McuPort::e, .drive = {.mask = byte_mask, .levels = 6}}};
  Fixture fixture(polling_program(), config);
  fixture.until(millisecond);
  require(fixture.read(source_value) == 14 && fixture.read(reset_value) == 1,
          "Cold loopback and U0 pull are visible to actual native register reads");
  fixture.until(3 * millisecond);
  require(fixture.read(source_value) == 2 && fixture.read(reset_value) == 1,
          "A changed external source withdraws the old loopback destination level");
  fixture.until(5 * millisecond);
  require(fixture.read(source_value) == 14 && fixture.read(reset_value) == 1,
          "Replacing the destination drive cannot suppress subsequent source projection");
}

void unrelated_port_writes() {
  constexpr auto prefix = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                       std::array<Byte, 1>{other_u_outputs},
                                       isa::opcode<Opcode::staa_extended>(),
                                       address(direction_u));
  constexpr auto program = isa::program(prefix,
                                        isa::opcode<Opcode::ldaa_extended>(),
                                        address(reg::porte),
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(source_value),
                                        isa::opcode<Opcode::ldaa_extended>(),
                                        address(reg::ptu),
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(reset_value),
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(reg::porta),
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(reg::portb),
                                        isa::opcode<Opcode::jmp_extended>(),
                                        address(entry + prefix.size()));
  auto config = configuration();
  config.events = {
      {.ticks = 2 * millisecond, .port = McuPort::e, .drive = {.mask = byte_mask, .levels = 2}}};
  Fixture fixture(program, config);
  fixture.until(millisecond);
  require(fixture.read(source_value) == 14 && fixture.read(reset_value) == 1,
          "Unrelated A/B writes preserve the real loopback and reset input observations");
  fixture.until(3 * millisecond);
  require(fixture.read(source_value) == 2 && fixture.read(reset_value) == 1,
          "Interleaved A/B writes cannot conceal a changed E source from native reads");
}

void u0_output_release() {
  constexpr auto prefix = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                       std::array<Byte, 1>{1},
                                       isa::opcode<Opcode::staa_extended>(),
                                       address(reg::ptu));
  constexpr auto program = isa::program(prefix,
                                        isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{all_u_outputs},
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(direction_u),
                                        isa::opcode<Opcode::ldaa_extended>(),
                                        address(reg::ptu),
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(before_release),
                                        isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{other_u_outputs},
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(direction_u),
                                        isa::opcode<Opcode::ldaa_extended>(),
                                        address(reg::ptu),
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(after_release),
                                        isa::opcode<Opcode::jmp_extended>(),
                                        address(entry + prefix.size()));
  Fixture fixture(program);
  fixture.until(millisecond);
  require(fixture.read(before_release) == 1 && fixture.read(after_release) == 1,
          "An actual output release restores the U0 input pull without changing its level");
}

void feedback_preserves_contention() {
  constexpr auto program = isa::program(isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{0},
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(reg::porte),
                                        isa::opcode<Opcode::ldaa_immediate>(),
                                        std::array<Byte, 1>{8},
                                        isa::opcode<Opcode::staa_extended>(),
                                        address(reg::ddre),
                                        isa::opcode<Opcode::jmp_extended>(),
                                        address(entry + 10));
  auto config = configuration();
  config.loopback = {
      .source = McuPort::e, .source_mask = 12, .destination = McuPort::e, .destination_mask = 8};
  config.initial[0].drive.mask = byte_mask & ~8U;
  Fixture fixture(program, config);
  bool refused = false;
  constexpr unsigned maximum_operations = 16;
  for (unsigned count = 0; count < maximum_operations && !refused; ++count) {
    try {
      fixture.instrument.advance();
    } catch (const Fault &fault) {
      refused = std::string_view(fault.what()).contains("contention");
      require(refused, "Only the actual opposed feedback output is the expected failure");
    }
  }
  require(refused,
          "A destination write that changes its own source cannot hide the next contention");
}
}

int main() {
  try {
    input_changes_and_destination_replacement();
    unrelated_port_writes();
    u0_output_release();
    feedback_preserves_contention();
    std::cout
        << "Native instrument GPIO projections preserve inputs, release and feedback faults\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
