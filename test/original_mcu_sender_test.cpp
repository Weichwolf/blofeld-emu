// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/firmware.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
constexpr unsigned phy_poll = 0x8049;
constexpr unsigned ready_poll = 0xb98d;
constexpr unsigned sender_end = 0xb9aa;
constexpr unsigned instruction_limit = 2000000;
constexpr unsigned port_b = 1;
constexpr unsigned select_n = 2;
constexpr unsigned clock_pin = 4;
constexpr unsigned mosi_pin = 16;
constexpr unsigned serial_pins = select_n | clock_pin | mosi_pin;
constexpr unsigned other_b_inputs = 0xe8;
constexpr unsigned port_u = 0x278;
constexpr unsigned frame_offset = 0x7875;
constexpr unsigned stage_words = 94;
constexpr unsigned stage_base = 0x6a;
constexpr unsigned word_bits = 24;
constexpr unsigned transmitted_words = stage_words + 3;
constexpr unsigned expected_port_writes = transmitted_words * ((word_bits * 3) + 3);

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void snapshot(std::ostream &trace, const unsigned completed, const blofeld::CpuState state) {
  trace << completed << " pc=" << blofeld::hex(state.pc) << " d=" << blofeld::hex(state.d)
        << " x=" << blofeld::hex(state.x) << " y=" << blofeld::hex(state.y)
        << " sp=" << blofeld::hex(state.sp) << " ccr=" << blofeld::hex(state.ccr, 2) << '\n';
}

// This records stable selected clock edges. It does not resolve simultaneous
// SS/SCK transitions, provide SHI inputs, or assert physical board wiring.
class Capture {
public:
  explicit Capture(std::ostream &trace) : trace_(trace) {}

  void begin(blofeld::McuBus &bus) {
    require(!active_, "Original sender capture is armed once");
    previous_ = sample(bus);
    require((previous_ & select_n) != 0, "Original initialization leaves selection deasserted");
    active_ = true;
  }

  void observe(blofeld::McuBus &bus, const blofeld::BusEvent &event) {
    if (!active_ || event.address != port_b) { return; }
    const auto current = sample(bus);
    trace_ << "PORTB value=" << blofeld::hex(event.value, 2)
           << " serial-pads=" << blofeld::hex(current, 2) << '\n';
    const bool was_selected = (previous_ & select_n) == 0;
    const bool selected = (current & select_n) == 0;
    if (((previous_ ^ current) & (select_n | clock_pin)) == (select_n | clock_pin)) {
      ++unordered_transitions_;
    }
    if (!was_selected && selected) { require(bits_ == 0, "Each original frame starts empty"); }
    if (was_selected && selected && (previous_ & clock_pin) != 0 && (current & clock_pin) == 0) {
      require(bits_ < word_bits, "Original selected frame has at most 24 stable falling edges");
      word_ = (word_ << 1U) | ((current & mosi_pin) != 0 ? 1U : 0U);
      ++bits_;
    }
    if (was_selected && !selected) {
      require(bits_ == word_bits, "Every original frame ends after 24 stable falling edges");
      words_.push_back(word_);
      bits_ = 0;
      word_ = 0;
    }
    previous_ = current;
    ++writes_;
  }

  void verify(const std::span<const std::uint8_t> mcu) const {
    require(active_ && bits_ == 0 && (previous_ & select_n) != 0,
            "Original sender completes with selection released and no partial frame");
    require(words_.size() == transmitted_words && writes_ == expected_port_writes,
            "Original CPU produces 97 complete frames through exactly 7275 PORTB stores");
    require(words_.at(0) == stage_words && words_.at(1) == stage_base,
            "Original sender emits the independently identified DSP length and start address");
    for (std::size_t index = 0; index < words_.size(); ++index) {
      const auto offset = frame_offset + (index * 3);
      const auto expected = (static_cast<std::uint32_t>(mcu[offset]) << 16U) |
                            (static_cast<std::uint32_t>(mcu[offset + 1]) << 8U) | mcu[offset + 2];
      require(words_[index] == expected,
              "Every emitted word, including the trailing word, matches unmodified MCU flash");
    }
  }

  [[nodiscard]] unsigned unordered_transitions() const { return unordered_transitions_; }

private:
  [[nodiscard]] static std::uint8_t sample(blofeld::McuBus &bus) {
    const auto pads = bus.observe_pads(blofeld::McuPort::b, serial_pins);
    require(pads.known == serial_pins,
            "Original GPIO instructions supply all captured serial pins");
    return pads.levels;
  }

  std::ostream &trace_;
  std::vector<std::uint32_t> words_;
  std::uint32_t word_ = 0;
  unsigned bits_ = 0;
  unsigned writes_ = 0;
  unsigned unordered_transitions_ = 0;
  std::uint8_t previous_ = 0;
  bool active_ = false;
};

class Gate {
public:
  void observe(blofeld::McuBus &bus, Capture &capture) {
    if (visits_ == 0) {
      require(bus.observe_pads(blofeld::McuPort::u, 1).known == 0,
              "Releasing PU0 does not manufacture an external ready level");
      verify_unknown(bus);
      // Explicit laboratory observations, not defaults or a measured board profile.
      bus.set_pin_drive(blofeld::McuPort::u, {.mask = 1});
      bus.set_pin_drive(blofeld::McuPort::b, {.mask = other_b_inputs});
      capture.begin(bus);
    } else if (visits_ == 2) {
      bus.set_pin_drive(blofeld::McuPort::u, {.mask = 1, .levels = 1});
    }
    require(visits_ <= 2, "Observed PU0 high releases the original polling loop");
    ++visits_;
  }

  void verify() const {
    require(visits_ == 3,
            "Original BRCLR waits twice for observed low, then consumes observed high");
  }

private:
  static void verify_unknown(blofeld::McuBus &bus) {
    try {
      static_cast<void>(bus.read(port_u));
    } catch (const blofeld::Fault &fault) {
      const std::string message = fault.what();
      require(message.find("0x0278") != std::string::npos &&
                  message.find("mask=0x01") != std::string::npos,
              "Unobserved PU0 reports its address and unknown input mask");
      return;
    }
    throw blofeld::Fault("Unobserved PU0 must remain unknown before the explicit pin experiment");
  }

  unsigned visits_ = 0;
};

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path trace;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  blofeld::McuBus bus(firmware.mcu);
  constexpr unsigned fixture_e_inputs = 0xfb;
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = fixture_e_inputs, .levels = 3});
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.reset();
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private original MCU sender trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Capture capture(trace);
  bus.set_observer(
      [&bus, &capture](const blofeld::BusEvent &event) { capture.observe(bus, event); });
  Gate gate;
  bool phy_sampled = false;
  unsigned completed = 0;
  while (cpu.state().pc != sender_end) {
    require(completed < instruction_limit, "Original MCU sender instruction budget exhausted");
    const auto before = cpu.state();
    if (before.pc == phy_poll && !phy_sampled) {
      bus.observe_phy({.edges = 15});
      phy_sampled = true;
    }
    if (before.pc == ready_poll) {
      snapshot(trace, completed, before);
      gate.observe(bus, capture);
    }
    try {
      const auto result = cpu.step();
      require(result.instruction_executed && !result.interrupt_vector,
              "Untimed original sender fixture executes without synthetic interrupts");
    } catch (const blofeld::Fault &fault) {
      snapshot(std::cerr, completed, before);
      throw blofeld::Fault("Original MCU sender: " + std::string(fault.what()));
    }
    ++completed;
  }
  require(phy_sampled, "Original reset traverses its own PHY polling before application startup");
  gate.verify();
  capture.verify(firmware.mcu);
  snapshot(trace, completed, cpu.state());
  std::cout << "Original MCU reset/application transmitted 97 exact 24-bit words through GPIO in "
            << completed << " instructions, with " << capture.unordered_transitions()
            << " simultaneous SS/SCK changes recorded. Explicit PU0/PB input experiment; "
            << "no DSP receiver, physical route, elapsed timing or complete boot proven.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 3) { throw blofeld::Fault("Usage: mcu-sender-test firmware.zip private.trace"); }
    return run({.firmware = arguments[1], .trace = arguments[2]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
