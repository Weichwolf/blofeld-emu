// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp/own_dsp_timeline.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using test::esai::require;
namespace native = test::dma_test;
constexpr unsigned limit = 100, offsets = 32;
constexpr std::array prefixes{0U, 8U};
constexpr unsigned source = 0x660, destination = 0x900, pattern = 0x13579b, canary = 0xa55a33;
constexpr unsigned dsr = 0xffffef, ddr = 0xffffee, dco = 0xffffed, dcr = 0xffffec;
constexpr unsigned copy_xy = 0x880244, enable = 0x800000, word_mask = 0xffffff;
constexpr unsigned tplr = 0xffff83, load_seed = 0x12345, compare_seed = 0xabcde;
constexpr unsigned read_x0 = 0x44f000, without_di = 0xffefff;
constexpr unsigned register_cases = 22;
constexpr unsigned high_y_movep = 0x09f480, high_io = 0xffffc0;
constexpr std::array timer_controls{0xffff8fU, 0xffff8bU, 0xffff87U};

struct Port {
  DspPort port;
  unsigned data;
  unsigned mask;
};

constexpr std::array ports{Port{.port = DspPort::c, .data = 0xffffbd, .mask = 0xfff},
                           Port{.port = DspPort::e, .data = 0xffff9d, .mask = 0xfff},
                           Port{.port = DspPort::f, .data = 0xfffff5, .mask = 0x7ff},
                           Port{.port = DspPort::d, .data = 0xffffd5, .mask = 3}};
enum class Kind : std::uint8_t { prescaler, load, compare, control, gpio };

struct Case {
  DspStore store;
  Kind kind;
  std::optional<DspPort> port;
  DspGpioRegister reg = DspGpioRegister::data;
};

std::vector<Case> cases() {
  std::vector<Case> result{{.store = {.space = DspSpace::x, .address = tplr, .value = 7},
                            .kind = Kind::prescaler,
                            .port = std::nullopt}};
  for (const auto control : timer_controls) {
    result.push_back({.store = {.space = DspSpace::x, .address = control - 1, .value = load_seed},
                      .kind = Kind::load,
                      .port = std::nullopt});
    result.push_back(
        {.store = {.space = DspSpace::x, .address = control - 2, .value = compare_seed},
         .kind = Kind::compare,
         .port = std::nullopt});
    result.push_back({.store = {.space = DspSpace::x,
                                .address = control,
                                .value = control == timer_controls.back() ? 0x8000U : 0x100U},
                      .kind = Kind::control,
                      .port = std::nullopt});
  }
  for (const auto port : ports) {
    for (const auto reg :
         {DspGpioRegister::data, DspGpioRegister::direction, DspGpioRegister::control}) {
      result.push_back(
          {.store = {.space = port.port == DspPort::c || port.port == DspPort::d ? DspSpace::x
                                                                                 : DspSpace::y,
                     .address = port.data + static_cast<unsigned>(reg),
                     .value = 1},
           .kind = Kind::gpio,
           .port = port.port,
           .reg = reg});
    }
  }
  return result;
}

native::NativeProgram
program(const DspStore store, const unsigned prefix, const bool read = false) {
  native::NativeProgram p;
  for (unsigned index = 0; index < prefix; ++index) { p.word(0); }
  if (read) {
    p.word(read_x0);
    p.word(store.address);
  } else if (store.space == DspSpace::y && store.address >= high_io) {
    // FM13-135: independently encoded high-Y immediate MOVEP (Port F).
    p.word(high_y_movep | (store.address - high_io));
    p.word(store.value);
  } else {
    p.write(store);
  }
  for (unsigned index = 0; index < limit; ++index) { p.word(0); }
  return p;
}

void configure(DspBus &bus) {
  for (const auto port : ports) {
    auto &gpio = bus.gpio(port.port);
    gpio.set_contract(DspGpioContract::logical_pins);
    gpio.write({.reg = DspGpioRegister::direction, .value = port.mask});
    gpio.write({.reg = DspGpioRegister::data});
  }
  for (const auto control : timer_controls) {
    bus.timers().write({.address = control - 1});
    bus.timers().write({.address = control - 2, .value = canary});
  }
  bus.write({.space = DspSpace::x, .address = source, .value = pattern});
  bus.write({.space = DspSpace::y, .address = destination, .value = canary});
  bus.write({.space = DspSpace::x, .address = dsr, .value = source});
  bus.write({.space = DspSpace::x, .address = ddr, .value = destination});
  bus.write({.space = DspSpace::x, .address = dco});
  bus.write({.space = DspSpace::x, .address = dcr, .value = copy_xy});
}

DspDmaInputs inputs(const unsigned cycle, const unsigned request) {
  return {.cycle = cycle,
          .known_requests = 15,
          .requests = cycle == request ? 1U : 0U,
          .timing = DspDmaTiming::reference_sampled_cycles,
          .grant = DspDmaGrant::available};
}

unsigned actual_value(DspBus &bus, const Case &c) {
  if (c.port) {
    const auto mask = c.reg == DspGpioRegister::data ? 1U : word_mask;
    return bus.gpio(*c.port).read(c.reg, mask);
  }
  auto timers = bus.timers();
  if (c.kind == Kind::load) {
    const auto control = c.store.address + 1;
    timers.write({.address = control - 2, .value = word_mask});
    timers.write({.address = control, .value = 1});
    timers.advance_clk2(1);
    return timers.read(control - 3);
  }
  const auto mask = c.kind == Kind::control ? without_di : word_mask;
  return timers.read(c.store.address, mask);
}

void overlap(const Case &c, const unsigned prefix) {
  unsigned overlaps = 0;
  for (unsigned request = 1; request < offsets; ++request) {
    OwnDspMachine machine(native::native_clocks);
    native::reset_native(machine, program(c.store, prefix));
    configure(machine.bus());
    require(actual_value(machine.bus(), c) != c.store.value,
            "Every native store must visibly change the independently seeded peripheral");
    OwnDspTimeline timeline(
        machine, OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts, 0);
    bool copied = false;
    bool retired = false;
    bool committed = false;
    for (unsigned cycle = 0; cycle < limit && (!copied || !retired); ++cycle) {
      const auto before = machine.cpu().registers();
      const auto event = timeline.advance_one(
          timeline.next_deadline(), OwnDspBoundary::through, inputs(cycle, request));
      if (!event) { throw Fault("Expected a completed independent-I/O reference period"); }
      require(!event->dma.core_stall &&
                  (!event->native || event->native->fault.kind == dsp56300::FaultKind::none),
              "Independent peripheral/F2 effects and both actual DMA phases remain executable");
      if (event->dma.word) {
        require(!copied && event->dma.word->value == pattern,
                "Exactly one genuine DMA word carries the independently seeded RAM payload");
        copied = true;
      }
      if (!committed && actual_value(machine.bus(), c) == c.store.value) {
        require(prefix != 0 || event->captured,
                "The actual store overlaps real P sampling in the initially empty queue");
        overlaps += static_cast<unsigned>(event->dma.phase.has_value());
        committed = true;
      }
      if (event->native && event->native->retired && event->native_pc == native::boot + prefix) {
        auto expected = before;
        expected.pc = native::boot + prefix + 2;
        require(committed && machine.cpu().registers() == expected,
                "Native MOVEP commits before retiring with the independent PC/flag expectation");
        retired = true;
      }
    }
    require(copied && retired && !timeline.faulted() &&
                machine.bus().read({.space = DspSpace::y, .address = destination}) == pattern &&
                (machine.bus().read({.space = DspSpace::x, .address = dcr}) & enable) == 0,
            "Native register programming and actual RAM DMA complete for every phase offset");
    require(actual_value(machine.bus(), c) == c.store.value,
            "Actual peripheral programming remains visible after both native and DMA completion");
  }
  require(overlaps == 2, "Both actual DMA phases overlap the native store and concurrent P fetch");
}

void guard(const DspStore store, const bool read, const std::string &cause) {
  constexpr unsigned prefix = 0;
  OwnDspMachine machine(native::native_clocks);
  native::reset_native(machine, program(store, prefix, read));
  configure(machine.bus());
  OwnDspTimeline timeline(
      machine, OwnDspAutomaticTiming::one_effect_per_cycle_program_priority_with_interrupts, 0);
  for (unsigned cycle = 0; cycle < limit; ++cycle) {
    const auto result = timeline.advance_one(
        timeline.next_deadline(), OwnDspBoundary::through, inputs(cycle, limit));
    if (!result) { throw Fault("Expected a real guarded native peripheral period"); }
    if (!result->native || result->native->fault.kind == dsp56300::FaultKind::none) { continue; }
    const auto &failure = machine.io().failure();
    require(timeline.faulted() && failure && failure->address == store.address &&
                failure->pc == native::boot + prefix && failure->cause &&
                failure->operation ==
                    (read ? OwnDspIo::Operation::read : OwnDspIo::Operation::write) &&
                machine.cpu().progress().instructions == prefix &&
                machine.cpu().registers().pc == native::boot + prefix,
            "Resource admission preserves the actual unretired contextual native I/O failure");
    try {
      std::rethrow_exception(failure->cause);
    } catch (const std::exception &error) {
      require(std::string(error.what()).contains(cause),
              "The original timer handler owns the unknown/read-only/write-only refusal");
    }
    return;
  }
  throw Fault("Invalid native I/O unexpectedly completed");
}
}

int main() {
  try {
    const auto all = cases();
    require(all.size() == register_cases,
            "Ten timer/prescaler and twelve GPIO register cases are present");
    for (const auto prefix : prefixes) {
      for (const auto &c : all) { overlap(c, prefix); }
    }
    for (const auto address : {0xffff82U, 0xffff84U, 0xffff88U, 0xffff8cU}) {
      guard({.space = DspSpace::x, .address = address, .value = 1}, false, "Read-only DSP");
    }
    guard({.space = DspSpace::x, .address = timer_controls.front()},
          true,
          "Unknown DSP timer DI read");
    guard({.space = DspSpace::x, .address = timer_controls.front() - 1},
          true,
          "Write-only DSP timer load read");
    std::cout << "22 native I/O registers,31 DMA offsets, both fetch queues, actual store/F2 "
                 "sampling and six guards passed.\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
