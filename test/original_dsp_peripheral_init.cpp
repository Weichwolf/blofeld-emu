// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_esai.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <exception>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>
#include <string>

namespace {
using blofeld::test::require;
using enum blofeld::DspSpace;
constexpr auto serial = blofeld::DspEsaiInstance::secondary;
constexpr unsigned record_address = 0x2d73;
constexpr unsigned entry = 0x2e8f;
constexpr unsigned active_fault = 0x2eaa;
constexpr unsigned reserved_fault = 0x2eb1;
constexpr unsigned memory_mode = 0x600389;
constexpr unsigned masked_status = 0xc00300;
constexpr unsigned personal_reset = 0x80000;
constexpr unsigned transmit_control = 0xffff95;
constexpr unsigned clock_control = 0xffff96;
constexpr unsigned serial_status = 0xffff93;
constexpr unsigned original_clock = 0x640301;
constexpr unsigned original_transmit = 0x7d01;
constexpr unsigned dma_status = 0xfffff4;
constexpr unsigned original_source = 0x660;
constexpr unsigned dma_source = 0xffffef;
constexpr unsigned instruction_limit = 32;
constexpr unsigned replay_count = 3;
constexpr unsigned first_sample = 6;
constexpr unsigned sample_period = 4;
constexpr unsigned frame_bits = 64;
constexpr unsigned slot_bits = 32;
constexpr unsigned expected_status = 0x3c000;

std::string rejects(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault("Expected an explicit original peripheral initialization failure");
}

const blofeld::DspRecord &component(const blofeld::Firmware &firmware) {
  for (const auto &record : firmware.dsp) {
    if (record.space == 0 && record.address == record_address) { return record; }
  }
  throw blofeld::Fault("Missing complete original extended-services component");
}

struct Register {
  unsigned address;
  unsigned value;
};

struct Inputs {
  bool held_reset;
  bool observed_clock;
};

struct Experiment {
  const blofeld::DspRecord &record;
  std::ostream &trace;
  const bool held_reset;
  const bool observed_clock;
  blofeld::DspMachine machine;
  unsigned writes = 0;

  Experiment(const blofeld::DspRecord &source, std::ostream &output, const Inputs inputs)
      : record(source),
        trace(output),
        held_reset(inputs.held_reset),
        observed_clock(inputs.observed_clock) {
    auto &bus = machine.bus();
    bus.observe_omr(memory_mode);
    bus.advance_instruction_cycles(3);
    for (unsigned index = 0; index < record.words.size(); ++index) {
      bus.write({.space = p, .address = record.address + index, .value = record.words[index]});
    }
    blofeld::test::DspProgram caller;
    caller.immediate("omr", memory_mode);
    for (unsigned index = 0; index < 3; ++index) { caller.emit("nop"); }
    caller.immediate("sr", masked_status);
    if (held_reset) {
      caller.emit("movep #>" + blofeld::test::number(personal_reset) + ",y:<<" +
                  blofeld::test::number(transmit_control));
    }
    caller.immediate("r6", entry);
    caller.emit("jsr (r6)");
    caller.emit("nop");
    caller.install(bus);
    machine.cpu().reset_spi_boot();
    bus.observe_esai_transmit(serial,
                              {.high_clock = observed_clock ? std::optional{false} : std::nullopt,
                               .timing = blofeld::DspEsaiClockTiming::reference_reset_edges});
    require(rejects([&] {
              static_cast<void>(bus.read({.space = x, .address = dma_source}));
            }).contains("Unknown DSP DMA address/counter"),
            "DMA source starts unknown; no phase/source pointer is supplied");
    for (unsigned steps = 0; machine.cpu().state().pc != entry; ++steps) {
      require(steps < instruction_limit, "Own native caller instruction bound");
      machine.cpu().step();
      bus.advance_instruction_cycles(1);
    }
  }

  void observe(const blofeld::DspNativeAccess &access) {
    if (access.kind != blofeld::DspNativeAccessKind::write ||
        access.stage != blofeld::DspNativeAccessStage::after) {
      return;
    }
    if (!access.value.has_value()) { throw blofeld::Fault("Completed write lacks its value"); }
    require(access.pc >= entry && access.pc < reserved_fault,
            "Completed writes originate from unchanged original initializer instructions");
    trace << "WRITE " << std::hex << access.pc << ' ' << static_cast<unsigned>(access.target.space)
          << ' ' << access.target.address << ' ' << *access.value << '\n';
    ++writes;
  }

  blofeld::DspCheckpoint prefix() {
    const auto stop = held_reset ? reserved_fault : active_fault;
    const blofeld::DspNativeAccessObserver observer{
        .observe = [this](const auto &access) { observe(access); }};
    unsigned steps = 0;
    while (machine.cpu().state().pc != stop) {
      require(steps++ < instruction_limit, "Original initializer prefix instruction bound");
      machine.cpu().step(observer);
      machine.bus().advance_instruction_cycles(1);
    }
    const unsigned expected_writes = held_reset ? 17U : 13U;
    require(writes == expected_writes && steps == expected_writes + 1,
            "Original CLR and complete native register-write prefix are retained");
    verify();
    return machine.checkpoint();
  }

  void verify() {
    // Primary manual: DOR0..3, DCO/DSR/DDR0 and DCO/DSR/DDR3 register layout.
    constexpr std::array expected{Register{.address = 0xfffff3, .value = 1},
                                  Register{.address = 0xfffff2, .value = 0xffffc1},
                                  Register{.address = 0xfffff1, .value = 1},
                                  Register{.address = 0xfffff0, .value = 0xffffc1},
                                  Register{.address = 0xffffed, .value = 0xfc0},
                                  Register{.address = dma_source, .value = original_source},
                                  Register{.address = 0xffffee, .value = 0xffff80},
                                  Register{.address = 0xffffe1, .value = 0xfc0},
                                  Register{.address = 0xffffe3, .value = original_source},
                                  Register{.address = 0xffffe2, .value = 0xffffd2},
                                  Register{.address = 0xffffec, .value = 0},
                                  Register{.address = 0xffffe0, .value = 0},
                                  Register{.address = dma_status, .value = 0x3f}};
    auto &bus = machine.bus();
    for (const auto item : expected) {
      require(bus.read({.space = x, .address = item.address}) == item.value,
              "Original writes establish DMA preloads; both channels remain disabled");
    }
    require(bus.read({.space = y, .address = clock_control}) ==
                    (held_reset ? original_clock : 0U) &&
                bus.read({.space = y, .address = transmit_control}) ==
                    (held_reset ? original_transmit : 0U),
            "ESAI controls retain only the completed original prefix");
    require(machine.cpu().state().sp == 1, "The genuine original call has not returned");
    for (unsigned index = 0; index < record.words.size(); ++index) {
      require(bus.inspect_program(record.address + index) == record.words[index],
              "Original component program words remain unchanged");
    }
  }

  void fault_replays(const blofeld::DspCheckpoint &saved) {
    const auto before = machine.cpu().debug_state();
    std::string expected;
    for (unsigned replay = 0; replay < replay_count; ++replay) {
      const auto error = rejects([&] { machine.cpu().step(); });
      require(error.contains(held_reset ? "pc=0x002eb1" : "pc=0x002eaa") &&
                  error.contains(held_reset ? "X:0xffff80" : "Y:0xffff96") &&
                  error.contains(held_reset ? "Unimplemented" : "Unsupported/active"),
              "The actual unsupported original access fails with native PC/address context");
      if (replay == 0) { expected = error; }
      require(error == expected, "Trusted restore repeats the exact native fault");
      trace << "FAULT " << error << '\n';
      static_cast<void>(rejects([&] { machine.cpu().step(); }));
      static_cast<void>(rejects([&] { static_cast<void>(machine.checkpoint()); }));
      machine.restore(saved);
      require(machine.cpu().debug_state() == before, "Coordinated restore retains the full CPU");
      verify();
    }
  }

  std::string zero_frame() {
    require(held_reset && observed_clock, "A known initialized HCKT source is required");
    const auto before = machine.cpu().debug_state();
    std::string bits;
    auto previous = machine.bus().esai_clock_outputs(serial);
    const auto limit = first_sample + ((frame_bits - 1) * sample_period);
    for (unsigned cycle = 1; cycle <= limit; ++cycle) {
      for (const bool high : {true, false}) {
        machine.bus().observe_esai_transmit(
            serial,
            {.high_clock = high, .timing = blofeld::DspEsaiClockTiming::reference_reset_edges});
        const auto pins = machine.bus().esai_clock_outputs(serial);
        require(pins.driven == 3 && pins.known == 3, "Original mux selects both generated clocks");
        if ((previous.levels & 1U) == 0 && (pins.levels & 1U) != 0) {
          const auto data = machine.bus().esai_transmit_outputs(serial);
          if (data.driven != 0) {
            require(data.driven == 1 && data.known == 1 && data.levels == 0,
                    "The actual original CLR/TX0 write supplies known zero data");
            require(cycle == first_sample + (bits.size() * sample_period),
                    "Every opposite-edge sample has an independent literal divider phase");
            require(((pins.levels & 2U) != 0) == (bits.size() < slot_bits),
                    "Generated word-length sync spans exactly the first32-bit slot");
            bits += '0';
          }
        }
        previous = pins;
      }
    }
    require(bits == std::string(frame_bits, '0'), "Exactly one two-slot zero frame is received");
    require(machine.bus().read({.space = y, .address = serial_status}) == expected_status,
            "Unrefilled second slot retains the real empty/even/odd/underrun flags");
    require(machine.cpu().debug_state() == before,
            "External clocks do not execute or skip CPU work");
    verify();
    return bits;
  }
};

void run(const blofeld::Firmware &firmware, std::ostream &trace) {
  unsigned cases = 0;
  unsigned frames = 0;
  for (const bool reset : {false, true}) {
    for (const bool clock : {false, true}) {
      trace << "CASE " << cases++ << " reset=" << reset << " clock=" << clock << '\n';
      Experiment experiment(
          component(firmware), trace, {.held_reset = reset, .observed_clock = clock});
      const auto saved = experiment.prefix();
      experiment.fault_replays(saved);
      if (reset && clock) {
        const auto first = experiment.zero_frame();
        experiment.machine.restore(saved);
        require(experiment.zero_frame() == first, "The complete original-prefix frame replays");
        frames += 2;
      } else if (reset) {
        require(rejects([&] {
                  static_cast<void>(experiment.machine.bus().esai_transmit_outputs(serial));
                }).contains("Unknown DSP ESAI clock/frame"),
                "Missing HCKT cannot manufacture initialized output");
      }
    }
  }
  trace << "RESULT cases=" << cases << " cold-faults=2 reset-prefixes=2 fault-replays=12"
        << " zero-frame-replays=" << frames << '\n';
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected original firmware and private trace");
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    run(blofeld::Firmware::load(arguments[1]), trace);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
