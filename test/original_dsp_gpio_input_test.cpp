// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::require;
constexpr unsigned vector = 0x10;
constexpr unsigned producer_entry = 0x2662;
constexpr unsigned pointer = 0x32;
constexpr unsigned ring = 0x2d00;
constexpr unsigned words = 128;
constexpr unsigned stamp = 0x1f;
constexpr unsigned scratch_a = 0x36;
constexpr unsigned scratch_r = 0x37;
constexpr unsigned scratch_x = 0x38;
constexpr unsigned guard = 0x2468ac;
constexpr unsigned stamp_seed = 0x13579b;
constexpr unsigned x_seed = 0x123456;
constexpr unsigned r_seed = 0x321;
constexpr unsigned status = 0xc00000;
constexpr unsigned switched = 0x600085;
constexpr unsigned alias_offset = 0xf000;
constexpr unsigned special_request = 0xf4;
constexpr unsigned clock_byte = 0xf8;
constexpr unsigned byte_values = 256;
constexpr unsigned last_byte = 255;
constexpr unsigned limit = 128;
constexpr unsigned original_irq_priorities = 0x4006; // IRQA edge IPL1 and DMA1 IPL0.
constexpr unsigned irq_priority_register = 0xffffff;

void install(DspBus &bus, const Firmware &firmware) {
  unsigned programs = 0;
  for (const auto &record : firmware.dsp) {
    if (record.space == 0) {
      if (record.address != vector && record.address != producer_entry) { continue; }
      ++programs;
    } else {
      const auto end = record.space == 1 ? 0x3000U : 0x2000U;
      if (record.address + record.words.size() > end) { continue; }
    }
    for (std::size_t index = 0; index < record.words.size(); ++index) {
      const auto address = record.address + static_cast<unsigned>(index);
      const bool alias = record.space == 0 && address >= 0x1000;
      bus.write({.space = alias ? DspSpace::x : static_cast<DspSpace>(record.space),
                 .address = alias ? address + alias_offset : address,
                 .value = record.words[index]});
    }
  }
  require(programs == 2, "Load the final original IRQA vector and complete GPIO input record");
}

class Fixture {
public:
  explicit Fixture(const Firmware &firmware) {
    install(machine_.bus(), firmware);
    auto &bus = machine_.bus();
    bus.write({.space = DspSpace::x, .address = pointer, .value = ring});
    bus.write({.space = DspSpace::x, .address = stamp, .value = stamp_seed});
    for (unsigned index = 0; index < words; ++index) {
      expected_.at(index) = guard ^ index;
      bus.write({.space = DspSpace::x, .address = ring + index, .value = expected_.at(index)});
    }
    for (const auto address : {ring - 1, ring + words}) {
      bus.write({.space = DspSpace::x, .address = address, .value = guard});
    }
    bus.observe_irqa(true);
    bus.gpio(DspPort::c).set_drive({.mask = last_byte});
    blofeld::test::DspProgram program;
    program.immediate("omr", switched);
    program.emit("nop");
    program.emit("nop");
    program.emit("nop");
    const auto mapped = program.pc();
    program.immediate("sr", status);
    program.immediate("a0", 0);
    program.immediate("a1", stamp_seed);
    program.immediate("a2", 0);
    program.immediate("x0", x_seed);
    program.immediate("r6", r_seed);
    for (unsigned index = 0; index < 8; ++index) {
      program.immediate("m" + std::to_string(index), blofeld::test::word_mask);
    }
    program.emit("movep #>$f00,x:<<$ffffbe");
    program.emit("movep #>0,x:<<$ffffbd");
    program.emit("movep #>$ff,x:<<$ffffbf");
    program.emit("movep #>" + blofeld::test::number(original_irq_priorities) + ",x:<<$ffffff");
    foreground_ = program.pc();
    program.emit("nop");
    program.install(bus);
    for (unsigned steps = 0; machine_.cpu().state().pc != foreground_; ++steps) {
      require(steps < limit, "Own native input/map/IRQA setup must terminate");
      if (machine_.cpu().state().pc == mapped) { bus.advance_instruction_cycles(3); }
      machine_.cpu().step();
    }
    require(bus.read({.space = DspSpace::x, .address = irq_priority_register}) ==
                    original_irq_priorities &&
                !bus.interrupt_request(0),
            "Native priority setup retains original4006 without fabricating reset DMA events");
  }

  void run(std::ostream &trace) {
    for (unsigned byte = 0; byte < byte_values; ++byte) {
      if (byte == special_request) { continue; }
      input(byte, trace);
    }
    require(received_ == last_byte && timestamps_ == 1,
            "Every ordinary byte, one clock timestamp and genuine ring wrap must execute");
    trace << "RESULT bytes=" << received_ << " irqs=" << received_ << " rtis=" << received_
          << " timestamps=" << timestamps_ << " steps=" << steps_ << '\n';
  }

private:
  void observe(const DspNativeAccess &access, const unsigned byte) {
    if (access.stage != DspNativeAccessStage::after || access.kind != DspNativeAccessKind::write) {
      return;
    }
    require(access.target.space == DspSpace::x && access.value.has_value(),
            "Original GPIO producer stores require a known X-space value");
    const auto address = access.target.address;
    if (address >= ring && address < ring + words) {
      require(address == ring + (received_ % words) && access.value == byte << 16U,
              "Actual producer store must match the independent input byte and cursor");
      ++writes_;
    } else if (address == stamp) {
      require(byte == clock_byte && access.value == 0,
              "Original F8 timestamp must sample the actual reset Timer2 count");
      ++timestamps_;
    } else {
      require(address == pointer || address == scratch_a || address == scratch_r ||
                  address == scratch_x,
              "Unexpected original GPIO producer write at " + blofeld::hex(address));
    }
  }

  void input(const unsigned byte, std::ostream &trace) {
    auto &bus = machine_.bus();
    const auto before = machine_.cpu().state();
    bus.gpio(DspPort::c).set_drive({.mask = last_byte, .levels = byte});
    bus.observe_irqa(false);
    const DspNativeAccessObserver observer{
        .observe = [&](const DspNativeAccess &access) { observe(access, byte); }};
    machine_.cpu().step(observer);
    ++steps_;
    require(machine_.cpu().state().pc == producer_entry && machine_.cpu().state().sp == 1 &&
                machine_.cpu().state().stack_pc == foreground_,
            "The original vector must push its real frame and call the original producer");
    unsigned body_steps = 0;
    while (machine_.cpu().state().pc != foreground_) {
      require(++body_steps <= limit, "Unchanged GPIO ISR must reach its own bounded RTI");
      machine_.cpu().step(observer);
      ++steps_;
    }
    require(machine_.cpu().state() == before && writes_ == received_ + 1 &&
                !bus.interrupt_request(0),
            "Actual RTI restores foreground state and clears only the captured IRQ edge");
    expected_.at(received_ % words) = byte << 16U;
    ++received_;
    for (unsigned index = 0; index < words; ++index) {
      require(bus.read({.space = DspSpace::x, .address = ring + index}) == expected_.at(index),
              "Every ring cell equals independent byte input or its untouched sentinel");
    }
    require(bus.read({.space = DspSpace::x, .address = pointer}) == ring + (received_ % words) &&
                bus.read({.space = DspSpace::x, .address = ring - 1}) == guard &&
                bus.read({.space = DspSpace::x, .address = ring + words}) == guard &&
                bus.read({.space = DspSpace::x, .address = stamp}) ==
                    (timestamps_ == 0 ? stamp_seed : 0U),
            "Actual cursor, both guards and timestamp retain their independent expectations");
    bus.observe_irqa(true);
    trace << "BYTE value=" << byte << " slot=" << (received_ - 1) % words << '\n';
  }

  DspMachine machine_;
  std::array<unsigned, words> expected_{};
  unsigned foreground_ = 0;
  unsigned received_ = 0;
  unsigned writes_ = 0;
  unsigned timestamps_ = 0;
  unsigned steps_ = 0;
};
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-dsp-gpio-input-test firmware.zip private.trace");
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot create private original GPIO input trace");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    Fixture fixture(Firmware::load(std::filesystem::path(arguments[1])));
    fixture.run(trace);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
