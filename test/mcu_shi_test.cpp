// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_shi.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_bus.hpp"
#include "mcu/mcu_gpio.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <iostream>
#include <optional>
#include <string>

namespace {
// Own diagnostic program and explicit synthetic wiring; not a Blofeld board profile.
constexpr unsigned port_b = 1;
constexpr unsigned direction_b = 3;
constexpr unsigned select_n = 2;
constexpr unsigned clock_pin = 4;
constexpr unsigned miso_pin = 8;
constexpr unsigned mosi_pin = 16;
constexpr unsigned serial_outputs = select_n | clock_pin | mosi_pin;
constexpr unsigned mcu_output_direction = 0xf7;
constexpr unsigned program_base = 0x8000;
constexpr unsigned receive_base = 0x1200;
constexpr unsigned hckr = 0xffff90;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned htx = 0xffff93;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned spi24_fifo = 0x29;
constexpr unsigned host_word = 0x2d87b1;
constexpr unsigned dsp_word = 0xa14ec3;
constexpr unsigned word_bits = 24;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned masked_interrupts = 0xd0;
constexpr unsigned load_a_immediate = 0x86;
constexpr unsigned load_a_extended = 0xb6;
constexpr unsigned and_a_immediate = 0x84;
constexpr unsigned store_a_extended = 0x7a;

void require(const bool value, const std::string &message) {
  if (!value) { throw blofeld::Fault(message); }
}

class Program {
public:
  void store(const blofeld::ByteWrite access) {
    emit({load_a_immediate,
          access.value,
          store_a_extended,
          static_cast<std::uint8_t>(access.address >> 8U),
          static_cast<std::uint8_t>(access.address & byte_mask)});
  }

  void sample(const std::uint16_t destination) {
    emit({load_a_extended,
          0,
          port_b,
          and_a_immediate,
          miso_pin,
          store_a_extended,
          static_cast<std::uint8_t>(destination >> 8U),
          static_cast<std::uint8_t>(destination & byte_mask)});
  }

  [[nodiscard]] const std::array<std::uint8_t, 32768> &image() const { return image_; }

  [[nodiscard]] unsigned end() const { return program_base + static_cast<unsigned>(used_); }

private:
  void emit(const std::initializer_list<std::uint8_t> bytes) {
    for (const auto byte : bytes) { image_.at(used_++) = byte; }
  }

  std::array<std::uint8_t, 32768> image_{};
  std::size_t used_ = 0;
};

Program make_program() {
  Program program;
  program.store({.address = port_b, .value = 0}); // Assert SS before the first bit.
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const unsigned data = ((host_word >> (word_bits - 1 - bit)) & 1U) != 0 ? mosi_pin : 0;
    program.store({.address = port_b, .value = static_cast<std::uint8_t>(data)});
    program.store({.address = port_b, .value = static_cast<std::uint8_t>(data | clock_pin)});
    program.sample(static_cast<std::uint16_t>(receive_base + bit));
    program.store({.address = port_b, .value = static_cast<std::uint8_t>(data)});
  }
  program.store({.address = port_b, .value = select_n});
  return program;
}

void follow_wires(blofeld::McuBus &bus, blofeld::DspShi &shi) {
  const auto pads = bus.observe_pads(blofeld::McuPort::b, serial_outputs);
  require((pads.known & (select_n | clock_pin)) == (select_n | clock_pin),
          "Synthetic board cannot invent unknown SS/SCK levels");
  std::optional<bool> mosi;
  if ((pads.known & mosi_pin) != 0) { mosi = (pads.levels & mosi_pin) != 0; }
  shi.observe_spi({.selected = (pads.levels & select_n) == 0,
                   .clock = (pads.levels & clock_pin) != 0,
                   .mosi = mosi});
  const auto output = shi.outputs().miso;
  blofeld::PinDrive drive;
  if (!output.driven) {
    drive.floating = miso_pin;
  } else if (output.level) {
    drive.mask = miso_pin;
    drive.levels = *output.level ? miso_pin : 0;
  }
  bus.set_pin_drive(blofeld::McuPort::b, drive);
}

void run_native_spi_exchange() {
  const auto program = make_program();
  blofeld::McuBus bus(program.image());
  bus.write(port_b, select_n);
  bus.write(direction_b, mcu_output_direction);
  blofeld::DspShi shi;
  follow_wires(bus, shi);
  shi.write({.address = hckr, .value = 0});
  shi.write({.address = hcsr, .value = spi24_fifo});
  shi.write({.address = htx, .value = dsp_word});
  unsigned observed = 0;
  bus.set_observer([&bus, &shi, &observed](const blofeld::BusEvent &event) {
    if (event.address != port_b && event.address != direction_b) { return; }
    ++observed;
    follow_wires(bus, shi);
  });
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state({.pc = program_base, .sp = 0x2000, .ccr = masked_interrupts});
  constexpr unsigned instruction_budget = 1024;
  unsigned completed = 0;
  while (cpu.state().pc != program.end()) {
    require(completed++ < instruction_budget,
            "Native SPI program must finish its own instruction stream");
    cpu.step();
  }
  require(observed == (word_bits * 3) + 2,
          "Only native MCU GPIO writes produced the three-wire serial input events");
  require(shi.read(hrx) == host_word, "SHI FIFO receives the exact independent MCU word");
  unsigned reconstructed = 0;
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const auto sample = bus.read(static_cast<std::uint16_t>(receive_base + bit));
    require(sample == 0 || sample == miso_pin, "Native LDAA/ANDA stores only the sampled MISO bit");
    reconstructed = (reconstructed << 1U) | (sample != 0 ? 1U : 0U);
  }
  require(reconstructed == dsp_word, "Native MCU loads observe the independent DSP transmit word");
  require(!shi.outputs().miso.driven, "Finishing the MCU frame releases the DSP MISO driver");
}
}

int main() {
  try {
    run_native_spi_exchange();
    std::cout
        << "Native HC12 GPIO/SHI full-duplex exchange passed; no DSP CPU or Blofeld boot proven\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
