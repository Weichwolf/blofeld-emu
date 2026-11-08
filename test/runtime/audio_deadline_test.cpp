// SPDX-License-Identifier: GPL-3.0-only
#include "cpu12/memory.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dax_registers.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "isa.hpp"
#include "mcu/hc12_registers.hpp"
#include "mcu/mcu_gpio.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_run_clock.hpp"
#include "storage/flash_image.hpp"
#include "support/isa.hpp"
#include "vi/vi.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace blofeld;
namespace rt = blofeld::vi;
namespace mcu = cpu12::isa;
namespace dsp = dsp56300::isa;
constexpr unsigned entry = 0x8000, boot = 0xff0000;
constexpr unsigned protection = 0x7f0d, security = 0x7f0f, byte_mask = 0xff;
constexpr unsigned unsecured = 0xfe, oscillator = 12000000, audio_hz = 1000000;
constexpr std::uint64_t timebase = 540000000, microsecond = timebase / 1000000;
constexpr std::uint64_t finish = 4 * timebase / 1000;
constexpr unsigned port_d_control = 0xffffd7, port_d_direction = 0xffffd6;
constexpr unsigned peripheral_base = 0xffffc0, stack = 0x2000;
constexpr unsigned spi_idle = 2, spi_mask = 0x16;
constexpr unsigned first_word = 0x123456, second_word = 0x789abc;
constexpr unsigned data_u = hc12::ptu, direction_u = data_u + 2;

void require(const bool value, const std::string_view message) {
  if (!value) { throw std::runtime_error(std::string(message)); }
}

constexpr std::array<cpu12::Byte, 2> address(const unsigned value) {
  return {static_cast<cpu12::Byte>(value >> 8U), static_cast<cpu12::Byte>(value & byte_mask)};
}

McuFlashImage image() {
  constexpr auto store_u = mcu::opcode<mcu::Opcode::staa_extended>();
  constexpr auto program = mcu::program(mcu::opcode<mcu::Opcode::lds_immediate>(),
                                        address(stack),
                                        mcu::opcode<mcu::Opcode::ldaa_immediate>(),
                                        std::array<cpu12::Byte, 1>{spi_idle},
                                        store_u,
                                        address(hc12::portb),
                                        mcu::opcode<mcu::Opcode::ldaa_immediate>(),
                                        std::array<cpu12::Byte, 1>{spi_mask},
                                        store_u,
                                        address(hc12::ddrb),
                                        mcu::opcode<mcu::Opcode::ldaa_immediate>(),
                                        std::array<cpu12::Byte, 1>{0},
                                        store_u,
                                        address(data_u),
                                        mcu::opcode<mcu::Opcode::ldaa_immediate>(),
                                        std::array<cpu12::Byte, 1>{1},
                                        store_u,
                                        address(direction_u),
                                        mcu::opcode<mcu::Opcode::ldaa_immediate>(),
                                        std::array<cpu12::Byte, 1>{0},
                                        store_u,
                                        address(direction_u),
                                        mcu::wai);
  std::array<cpu12::Byte, McuFlashImage::byte_count> bytes;
  bytes.fill(mcu::nop.front());
  std::ranges::copy(program, bytes.begin());
  bytes[protection] = byte_mask;
  bytes[security] = unsecured;
  bytes[bytes.size() - 2] = entry >> 8U;
  bytes.back() = 0;
  return McuFlashImage::from_known(bytes);
}

struct PeripheralWrite {
  unsigned address;
  unsigned value;
};

struct Program {
  void write(const PeripheralWrite access) {
    // MOVEP #imm,x:<<pp: immediate EA=$34, write direction in bit15.
    constexpr unsigned immediate = 0x34;
    constexpr unsigned direction = 0x8000;
    words.push_back(dsp::opcode<dsp::Opcode::movep_memory_high>().front() | (immediate << 8U) |
                    direction | (access.address - peripheral_base));
    words.push_back(access.value);
  }

  void empty(const bool expected) {
    // Test XAEM only. XAUR is deliberately excluded, as required by ED55.
    const auto operation = expected ? dsp::Opcode::jset_port_high : dsp::Opcode::jclr_port_high;
    words.push_back(static_cast<unsigned>(operation) | ((dax::status - peripheral_base) << 8U));
    words.push_back(boot + static_cast<unsigned>(words.size()) + 2);
    words.push_back(dsp::illegal.front());
  }

  void clocks() {
    constexpr unsigned instructions = 64;
    words.insert(words.end(), instructions, dsp::nop.front());
  }

  void enable(const unsigned divider) {
    write({.address = dax::non_audio, .value = 0});
    write({.address = dax::audio_a, .value = first_word});
    write({.address = dax::audio_b, .value = second_word});
    write({.address = dax::control, .value = divider});
    write({.address = port_d_control, .value = 1});
    write({.address = port_d_direction, .value = 1});
  }

  std::vector<std::uint32_t> words;
};

Program program() {
  Program result;
  result.empty(false);
  result.enable(8);
  result.clocks();
  result.empty(true);
  result.write({.address = dax::audio_a, .value = first_word});
  result.write({.address = dax::audio_b, .value = second_word});
  result.empty(false);
  result.clocks();
  result.empty(false);
  result.write({.address = port_d_direction, .value = 0});
  result.empty(false);
  constexpr unsigned slower = 0x18;
  result.enable(slower);
  result.clocks();
  result.empty(true);
  result.words.push_back(dsp::reset.front());
  result.empty(false);
  result.enable(8);
  result.clocks();
  result.empty(true);
  result.words.push_back(dsp::wait.front());
  return result;
}

rt::Config config() {
  rt::Config result;
  result.mcu = {.clock = {.oscillator_hz = oscillator,
                          .switch_timing = McuRunClockSwitch::reference_cycle_boundary},
                .memory_timing = McuRunMemory::internal_flash_reads};
  result.dsp_clock = {.extal_hz = oscillator,
                      .timebase_hz = timebase,
                      .switch_timing = DspClockSwitch::functional_region_boundary};
  result.dsp = {.reset_dsp_first = true,
                .initial_reset = DspResetInputs{
                    .mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2}};
  result.dma_grant = DspDmaGrant::available;
  result.mcu_u0_release = true;
  result.select_n = 2;
  result.clock = 4;
  result.mosi = 16;
  result.initial = {{.port = McuPort::e, .drive = {.mask = 3, .levels = 3}}};
  result.audio_clock = {.frequency_hz = audio_hz,
                        .epoch_ticks = microsecond,
                        .destinations = 4,
                        .scheduling = rt::AudioClockScheduling::transmitter_events};
  return result;
}

void native_deadlines() {
  const auto flash = image();
  const auto rom = program();
  std::ostringstream trace;
  rt::VirtualInstrument instrument(
      config(), {.image = flash, .rom = rom.words, .specification = {}, .trace = &trace});
  constexpr unsigned maximum_operations = 10000;
  unsigned operations = 0;
  while (instrument.common_ticks() < finish && operations++ < maximum_operations) {
    instrument.advance();
  }
  const auto status = instrument.status();
  require(status.common_ticks >= finish && status.waiting && !status.held_reset &&
              !status.faulted && status.dsp.pc == boot + rom.words.size(),
          "Native DSP guards verify deadline absence, activation, FIFO refill, divider change "
          "and RESET before entering WAIT while actual audio time continues");
  require(trace.str().find("DSP-AUDIO-CLOCK") != std::string::npos,
          "The configured source actually dispatches audio events");
}

void register_revisions() {
  DspBus bus;
  auto revision = bus.peripheral_revision();
  bus.write({.space = DspSpace::x, .address = 0, .value = first_word});
  require(bus.read({.space = DspSpace::x, .address = 0}) == first_word &&
              bus.peripheral_revision() == revision,
          "Ordinary native memory does not invalidate peripheral deadlines");
  bus.write({.space = DspSpace::x, .address = dax::non_audio, .value = 0});
  require(bus.peripheral_revision() == ++revision, "A real peripheral write changes its token");
  require(bus.access_peripheral_bit({.space = DspSpace::x,
                                     .address = dax::status,
                                     .bit = 0,
                                     .operation = DspBitOperation::test}) == false &&
              bus.peripheral_revision() == ++revision,
          "A peripheral bit read cannot leave a cached pre-access deadline usable");
  static_cast<void>(bus.read({.space = DspSpace::x, .address = dax::control}));
  require(bus.peripheral_revision() == ++revision, "Register reads advance the actual token");
  bus.reset_peripherals();
  require(bus.peripheral_revision() == ++revision,
          "RESET retires cached deadlines even when there was no GPIO register write");
}
}

int main() {
  try {
    register_revisions();
    native_deadlines();
    std::cout << "Native audio deadlines retain actual register, FIFO and reset effects\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
