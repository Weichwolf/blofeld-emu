// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

namespace {
using namespace blofeld;
using blofeld::test::DspProgram;
using blofeld::test::require;
constexpr unsigned entry = 0x29be, end = 0x29e3, mode = 0x600389;
constexpr unsigned slot = 0x800, ring = 0x1000, ring_words = 512;
constexpr unsigned fm_slot = 0x920, scratch = 0x930, output = 0x940;
constexpr unsigned descriptor = 0xed86, coefficient_begin = 0xde96, coefficient_end = descriptor;
constexpr unsigned guard = 0x13579b, mask = 0xffffff, phase_bits = 15;
constexpr unsigned step_limit = 128, maximum_cases = 150000, restored_modifier = 7;
constexpr unsigned expected_steps = 36;
constexpr unsigned absolute_jump = 0x0af080; // DSP56300FM JMP with absolute extension.

struct Input {
  unsigned id = 0, bank = 0, phase = 0, rate = 0, depth = 0, fm = 0;
  unsigned next = 0, result = 0, pointer = 0;
  int stride = 0;
  std::array<unsigned, 9> samples{};
};

Input read_input(std::istream &stream) {
  Input input;
  stream >> input.id >> input.bank >> input.phase >> input.rate >> input.depth >> input.fm >>
      input.next >> input.result >> input.pointer >> input.stride;
  for (auto &sample : input.samples) { stream >> sample; }
  require(static_cast<bool>(stream) && input.bank < 4 && input.phase <= mask &&
              input.rate <= mask && input.depth <= mask && input.fm <= mask && input.next <= mask &&
              input.result <= mask && input.pointer >= coefficient_begin &&
              input.pointer < coefficient_end && (input.stride == 1 || input.stride == -1),
          "Invalid bounded private sample-oracle row");
  return input;
}

void put(DspBus &bus, const DspSpace space, const unsigned address, const unsigned value) {
  bus.write({.space = space, .address = address, .value = value});
}

unsigned get(DspBus &bus, const DspSpace space, const unsigned address) {
  return bus.read({.space = space, .address = address});
}

unsigned caller(DspBus &bus) {
  DspProgram program;
  program.immediate("x1", mode);
  program.emit("move x1,omr");
  program.immediate("x1", blofeld::test::masked_status);
  program.emit("move x1,sr");
  for (unsigned index = 0; index < 8; ++index) {
    program.immediate("m" + std::to_string(index), mask);
  }
  program.immediate("m1", 15);
  program.immediate("m5", restored_modifier);
  program.immediate("r0", slot);
  program.immediate("r4", slot);
  program.immediate("r5", fm_slot);
  program.immediate("r3", scratch);
  program.immediate("r7", output);
  program.immediate("n0", 3);
  program.immediate("n4", 5);
  program.immediate("n3", 2);
  program.immediate("n7", 2);
  program.immediate("r2", entry);
  program.emit("jmp (r2)");
  const auto stop = program.pc();
  program.emit("nop");
  program.word(absolute_jump);
  program.word(blofeld::test::boot);
  program.install(bus);
  return stop;
}

class Fixture {
public:
  explicit Fixture(const Firmware &firmware) {
    auto &bus = machine_.bus();
    bus.observe_omr(mode);
    bus.advance_instruction_cycles(3);
    for (const auto record_index : {39U, 40U, 114U}) {
      const auto &record = firmware.dsp.at(record_index);
      const auto space = record_index == 114 ? DspSpace::p : DspSpace::x;
      require(record.space == static_cast<unsigned>(space),
              "Original sampler record space changed");
      for (unsigned index = 0; index < record.words.size(); ++index) {
        put(bus, space, record.address + index, record.words.at(index));
      }
    }
    stop_ = caller(bus);
    machine_.cpu().reset_spi_boot();
  }

  void run(const Input &input) {
    current_ = &input;
    coefficient_reads_ = 0;
    sample_reads_ = 0;
    writes_ = 0;
    seed(input);
    unsigned kernel_steps = 0;
    unsigned total_steps = 0;
    do {
      require(++total_steps <= step_limit, "Original sample callback exceeded its native bound");
      const auto pc = machine_.cpu().state().pc;
      require((pc >= entry && pc < end) || pc >= blofeld::test::boot,
              "Original sample callback left its declared caller/kernel");
      machine_.cpu().step({.observe = [this](const auto &access) { observe(access); }});
      machine_.bus().advance_instruction_cycles(1);
      if (pc >= entry && pc < end) {
        ++kernel_steps;
        visited_.insert(pc);
      }
    } while (machine_.cpu().state().pc != stop_);
    verify(input);
    require(
        kernel_steps == expected_steps && coefficient_reads_ == 8 && sample_reads_ == 8 &&
            writes_ == 3,
        "Each original sampler call executes36 instructions, eight coefficient/sample reads and "
        "three writes");
    ++calls_;
    instructions_ += kernel_steps;
    current_ = nullptr;
  }

  void summary(std::ostream &trace) const {
    require(visited_.size() == expected_steps, "All36 original sampler instruction starts execute");
    trace << "RESULT " << calls_ << ' ' << instructions_ << ' ' << visited_.size() << '\n';
  }

private:
  void seed(const Input &input) {
    auto &bus = machine_.bus();
    for (const auto space : {DspSpace::x, DspSpace::y}) {
      for (unsigned index = 0; index < 9; ++index) { put(bus, space, slot + index, guard); }
    }
    put(bus, DspSpace::x, slot, input.phase);
    put(bus, DspSpace::y, slot, input.rate);
    put(bus, DspSpace::x, slot + 1, stop_);
    put(bus, DspSpace::y, slot + 1, input.depth);
    put(bus, DspSpace::y, slot + 2, fm_slot);
    put(bus, DspSpace::x, slot + 3, ring);
    put(bus, DspSpace::y, slot + 3, descriptor + (input.bank * 4));
    put(bus, DspSpace::x, fm_slot, input.fm);
    put(bus, DspSpace::y, scratch, guard);
    for (unsigned index = 0; index < 3; ++index) { put(bus, DspSpace::x, output + index, guard); }
    for (unsigned index = 0; index < input.samples.size(); ++index) {
      const auto offset = ((input.phase >> phase_bits) + index) % ring_words;
      put(bus, DspSpace::y, ring + offset, input.samples.at(index));
    }
  }

  void observe(const DspNativeAccess &access) {
    if (access.pc < entry || access.pc >= end || access.stage != DspNativeAccessStage::after) {
      return;
    }
    const auto address = access.target.address;
    if (access.kind == DspNativeAccessKind::read && access.target.space == DspSpace::x &&
        address >= coefficient_begin && address < coefficient_end) {
      const auto expected = static_cast<int>(current_->pointer) +
                            (current_->stride * static_cast<int>(coefficient_reads_));
      require(coefficient_reads_ < 8 && address == static_cast<unsigned>(expected),
              "Native folded coefficient read address differs from independent oracle");
      ++coefficient_reads_;
    }
    if (access.kind == DspNativeAccessKind::read && access.target.space == DspSpace::y &&
        address >= ring && address < ring + ring_words) {
      const auto offset = ((current_->phase >> phase_bits) + sample_reads_) % ring_words;
      require(sample_reads_ < 8 && address == ring + offset &&
                  access.value == current_->samples.at(sample_reads_),
              "Native eight-tap sample reads must preserve modulo512 order and exact PCM words");
      ++sample_reads_;
    }
    if (access.kind != DspNativeAccessKind::write) { return; }
    require(access.target.space == DspSpace::x &&
                (address == slot || address == slot + 7 || address == output),
            "Original sample kernel wrote outside its declared phase/output cells");
    ++writes_;
  }

  void verify(const Input &input) {
    auto &bus = machine_.bus();
    const auto state = machine_.cpu().state();
    require(get(bus, DspSpace::x, slot) == input.next &&
                get(bus, DspSpace::x, slot + 7) == input.result &&
                get(bus, DspSpace::x, output) == input.result,
            "Original sample phase/output differs from the independent integer oracle, case=" +
                std::to_string(input.id) + " expected-phase=" + std::to_string(input.next) +
                " actual-phase=" + std::to_string(get(bus, DspSpace::x, slot)) +
                " expected-output=" + std::to_string(input.result) +
                " actual-output=" + std::to_string(get(bus, DspSpace::x, output)));
    require(state.sp == 0 && state.r[0] == slot + 8 && state.r[4] == slot + 8 &&
                state.r[5] == fm_slot && state.r[3] == scratch + 2 && state.r[7] == output + 2 &&
                state.m[1] == restored_modifier && state.n[6] == (input.stride == 1 ? 1 : mask),
            "Original sample continuation, raw stride and copied M5 retain their independent ABI");
    require(get(bus, DspSpace::x, output + 1) == guard &&
                get(bus, DspSpace::x, output + 2) == guard &&
                get(bus, DspSpace::x, slot + 8) == guard,
            "Output and slot guards survive the native callback");
  }

  DspMachine machine_;
  unsigned stop_ = 0, coefficient_reads_ = 0, sample_reads_ = 0, writes_ = 0, calls_ = 0;
  std::uint64_t instructions_ = 0;
  const Input *current_ = nullptr;
  std::set<unsigned> visited_;
};
}

int main(const int count, char **arguments) {
  try {
    require(count == 4, "Usage: original-dsp-sample firmware.zip private.inputs private.trace");
    std::ifstream inputs(arguments[2]);
    std::string signature;
    unsigned cases = 0;
    inputs >> signature >> cases;
    require(signature == "blofeld-sampler-v1" && cases > 0 && cases <= maximum_cases,
            "Invalid bounded sampler oracle header");
    std::ofstream trace(arguments[3]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    Fixture fixture(Firmware::load(arguments[1]));
    for (unsigned index = 0; index < cases; ++index) {
      const auto input = read_input(inputs);
      require(input.id == index, "Private sampler cases must remain ordered and complete");
      fixture.run(input);
      trace << "CASE " << input.id << ' ' << input.next << ' ' << input.result << '\n';
    }
    inputs >> std::ws;
    require(inputs.eof(), "Unexpected trailing sampler oracle input");
    fixture.summary(trace);
    std::cout
        << "Unmodified native sample interpolation and exact eight-tap PCM accesses verified.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
