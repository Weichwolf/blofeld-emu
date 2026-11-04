// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "firmware/firmware.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
using namespace blofeld;
namespace native = test::dma_test;
constexpr unsigned mask = 0xffffff;
constexpr unsigned mode = 0x600389;
constexpr unsigned ring_begin = 0x10000;
constexpr unsigned ring_end = 0x16000;
constexpr unsigned work_begin = 0xc400;
constexpr unsigned work_words = 256;
constexpr unsigned parameter_base = 0x9c6;
constexpr unsigned clear_calls = 48;
constexpr unsigned calls = clear_calls + 2;
constexpr unsigned chunk_words = 512;
constexpr unsigned output_pointer = 0x12;
constexpr unsigned output_words = 32;
constexpr unsigned filter_end = 0x1f58;
constexpr unsigned original_lookahead = 0x1f93;
constexpr unsigned first_lookahead_address = 0x12ab8;
constexpr unsigned ready_entry = 0x1f40;
constexpr unsigned clear_entry = 0x1f2c;
constexpr unsigned cursor_address = 0x530;
constexpr unsigned callback_address = 0x6cc;
constexpr unsigned maximum_steps = 200000;
constexpr unsigned expected_lookaheads = 2 * 12;
constexpr unsigned original_record_count = 127;
using Words = std::vector<std::uint32_t>;

void require(const bool condition, const std::string_view reason) {
  if (!condition) { throw std::runtime_error(std::string(reason)); }
}

struct Caller {
  native::NativeProgram program;
  unsigned setup_return, call_address, return_address, final_return;
};

struct Operand {
  unsigned opcode, value;
};

void operand(native::NativeProgram &program, const Operand input) {
  program.word(input.opcode);
  program.word(input.value);
}

Caller caller(const bool live) {
  // Authored caller only: FM MOVE/MOVEC, CLR, JSR and DO encodings.
  constexpr unsigned immediate_x1 = 0x45f400;
  constexpr unsigned x1_to_omr = 0x04c5ba;
  constexpr unsigned x1_to_sr = 0x04c5b9;
  constexpr unsigned immediate_modifier = 0x05f420;
  constexpr unsigned immediate_r0 = 0x60f400;
  constexpr unsigned immediate_r1 = 0x61f400;
  constexpr unsigned immediate_r4 = 0x64f400;
  constexpr unsigned load_r5 = 0x65f000;
  constexpr unsigned clear_a = 0x200013;
  constexpr unsigned clear_b = 0x20001b;
  constexpr unsigned eight_to_a1 = 0x2c0800;
  constexpr unsigned eight_to_b1 = 0x2d0800;
  constexpr unsigned call_absolute = 0x0bf080;
  constexpr unsigned call_r5 = 0x0be580;
  constexpr unsigned do_immediate = 0x060080;
  constexpr unsigned irq_mask = 0x300;
  constexpr unsigned descriptor = 0x6c8;
  constexpr unsigned reset_entry = 0x1fe5;
  constexpr unsigned live_entry = 0x2033;
  constexpr unsigned input_block = 0x640;
  constexpr unsigned byte_bits = 8;
  native::NativeProgram program;
  for (unsigned i = 0; i < 3; ++i) { program.word(0); }
  operand(program, {.opcode = immediate_x1, .value = mode});
  program.word(x1_to_omr);
  for (unsigned i = 0; i < 3; ++i) { program.word(0); }
  operand(program, {.opcode = immediate_x1, .value = irq_mask});
  program.word(x1_to_sr);
  for (unsigned i = 0; i < 8; ++i) {
    operand(program, {.opcode = immediate_modifier + i, .value = mask});
  }
  operand(program, {.opcode = immediate_r1, .value = descriptor});
  operand(program, {.opcode = immediate_r0, .value = parameter_base + (live ? 0U : 1U)});
  if (!live) {
    for (const auto opcode : {clear_a, clear_b, eight_to_a1, eight_to_b1}) { program.word(opcode); }
  }
  operand(program, {.opcode = call_absolute, .value = live ? live_entry : reset_entry});
  const auto returned = program.pc();
  program.word(0);
  constexpr unsigned loop_prefix = 2;
  constexpr unsigned body_last = 9;
  const auto begin = program.pc() + loop_prefix;
  operand(program, {.opcode = do_immediate | (calls << byte_bits), .value = begin + body_last});
  operand(program, {.opcode = immediate_r0, .value = input_block});
  operand(program, {.opcode = immediate_r4, .value = cursor_address});
  operand(program, {.opcode = load_r5, .value = callback_address});
  program.word(clear_a);
  program.word(call_r5);
  program.word(0); // Actual callback return boundary inside the caller's DO.
  program.word(0); // Core-owned loop end, repeated fifty times.
  program.word(0); // Final boundary after the genuine outer-loop unwind.
  return {.program = std::move(program),
          .setup_return = returned,
          .call_address = begin + 7,
          .return_address = begin + 8,
          .final_return = begin + body_last + 1};
}

void put(DspBus &bus, const DspSpace space, const unsigned address, const unsigned value) {
  bus.write({.space = space, .address = address, .value = value});
}

void install(DspBus &bus, const Firmware &firmware, const unsigned diffusion) {
  bus.set_write_contract(DspWriteContract::functional_decode);
  bus.observe_omr(mode);
  bus.advance_instruction_cycles(3);
  constexpr unsigned x_words = 0x3000;
  constexpr unsigned y_words = 0x2000;
  for (unsigned at = 0; at < x_words; ++at) { put(bus, DspSpace::x, at, 0); }
  for (unsigned at = 0; at < y_words; ++at) { put(bus, DspSpace::y, at, 0); }
  for (const auto &record : firmware.dsp) {
    for (unsigned i = 0; i < record.words.size(); ++i) {
      put(bus, static_cast<DspSpace>(record.space), record.address + i, record.words[i]);
    }
  }
  constexpr unsigned tag = 0x10000;
  constexpr unsigned tag_stride = 17;
  for (unsigned i = 0; i < work_words; ++i) {
    put(bus, DspSpace::x, work_begin + i, tag + (tag_stride * i));
    put(bus, DspSpace::y, work_begin + i, 0);
  }
  constexpr unsigned ring_tag = 0x20000;
  constexpr unsigned tag_modulus = 251;
  constexpr unsigned ring_stride = 37;
  for (unsigned i = 0; i < ring_end - ring_begin; ++i) {
    put(bus, DspSpace::y, ring_begin + i, ring_tag + ((i % tag_modulus) * ring_stride));
  }
  constexpr unsigned input_base = 0x620;
  constexpr unsigned input_tag = 0x2000;
  constexpr unsigned input_stride = 113;
  constexpr unsigned input_words = 34;
  for (unsigned i = 0; i < input_words; ++i) {
    put(bus, DspSpace::x, input_base + i, input_tag + (input_stride * i));
  }
  constexpr std::array<unsigned, 18> parameters{
      8, 64, 56, 89, 64, 0, 0, 80, 32, 0, 64, 0, 0, 0, 0, 0, 0, 64};
  for (unsigned i = 0; i < parameters.size(); ++i) {
    put(bus, DspSpace::x, parameter_base + i, parameters[i]);
  }
  put(bus, DspSpace::x, parameter_base + 9, diffusion);
  constexpr unsigned part_pointer = 0x12;
  constexpr unsigned part = 0x860;
  put(bus, DspSpace::x, part_pointer, part);
  put(bus, DspSpace::x, 6, 0);
  constexpr unsigned mode_flags = 0x15;
  put(bus, DspSpace::x, mode_flags, 0);
}

Words memory(DspBus &bus) {
  Words result;
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (const auto window : bus.data_windows(space)) {
      require(window.words.size() == window.known.size(), "Invalid captured memory view");
      result.push_back(window.address);
      for (std::size_t i = 0; i < window.words.size(); ++i) {
        result.push_back(window.known[i]);
        result.push_back(window.words[i]);
      }
    }
  }
  constexpr unsigned program_end = 0x3000;
  for (unsigned address = 0; address < program_end; ++address) {
    const auto word = bus.sample_program(address);
    result.push_back(word.has_value() ? 1U : 0U);
    result.push_back(word.value_or(0));
  }
  return result;
}

void original_program(DspBus &bus, const Firmware &firmware) {
  std::map<unsigned, unsigned> expected;
  for (const auto &record : firmware.dsp) {
    if (record.space != 0) { continue; }
    for (unsigned i = 0; i < record.words.size(); ++i) {
      expected[record.address + i] = record.words[i];
    }
  }
  for (const auto &[address, word] : expected) {
    require(bus.read({.address = address}) == word,
            "Every ordered original P word must remain unchanged after the original clear-to-ready "
            "lifecycle");
  }
}

struct Store {
  unsigned address = 0, value = 0, pc = 0;
  bool selected = false;
  bool operator==(const Store &) const = default;
};

struct Observation {
  DspBus *bus;
  std::array<Store, output_words> input{};
  unsigned used = 0;
};

void observe_write(void *context, const DspStore &store, const unsigned pc) {
  if (store.space != DspSpace::y || pc < ready_entry || pc >= filter_end ||
      store.address < ring_begin) {
    return;
  }
  auto &observation = *static_cast<Observation *>(context);
  require(observation.used < observation.input.size(),
          "Unexpected additional original input store");
  const auto resource =
      observation.bus->memory_resource({.target = {.space = store.space, .address = store.address},
                                        .operation = DspMemoryOperation::write});
  observation.input[observation.used++] = {
      .address = store.address, .value = store.value, .pc = pc, .selected = resource.selected};
}

void inspect_clear(DspBus &bus, const unsigned completed) {
  const auto cleared = completed * chunk_words;
  for (unsigned index = 0; index < ring_end - ring_begin; ++index) {
    constexpr unsigned ring_tag = 0x20000;
    constexpr unsigned tag_modulus = 251;
    constexpr unsigned tag_stride = 37;
    const auto expected = index < cleared ? 0 : ring_tag + ((index % tag_modulus) * tag_stride);
    require(bus.read({.space = DspSpace::y, .address = ring_begin + index}) == expected,
            "Each actual clear call must erase exactly512 words and retain the untouched suffix");
  }
  const auto output = bus.read({.space = DspSpace::x, .address = output_pointer});
  for (unsigned index = 0; index < output_words; ++index) {
    require(bus.read({.space = DspSpace::y, .address = output + index}) == 0,
            "The original clear callback must zero its actual32-word output block");
  }
  require(bus.read({.space = DspSpace::x, .address = cursor_address}) == ring_begin + cleared &&
              bus.read({.space = DspSpace::x, .address = callback_address}) ==
                  (completed == clear_calls ? ready_entry : clear_entry),
          "Only the48th real clear installs ready, retaining cursor16000 without host repair");
}

struct Outcome {
  dsp56300::Registers registers;
  Words storage;
  std::array<Store, output_words> input;
  std::uint64_t instructions, reads, writes;
  bool operator==(const Outcome &) const = default;
};

struct Scene {
  bool live;
  unsigned diffusion, word;
};

struct Counters {
  unsigned setup_returns = 0, returned = 0, clears = 0, ready = 0, lookaheads = 0;
};

void inspect_boundary(OwnDspMachine &machine,
                      const dsp56300::Registers &state,
                      const Caller &code,
                      Counters &count) {
  if (state.pc == code.setup_return) {
    require(
        state.sp == 0 &&
            machine.bus().read({.space = DspSpace::x, .address = cursor_address}) == ring_begin &&
            machine.bus().read({.space = DspSpace::x, .address = callback_address}) == clear_entry,
        "Original setup must actually return and install its own clear cursor/callback");
    ++count.setup_returns;
  }
  if (state.pc == code.call_address) {
    require(state.r[5] == (count.returned < clear_calls ? clear_entry : ready_entry),
            "The native caller must load the actual firmware-owned callback before its JSR");
  }
  if (state.pc == clear_entry) { ++count.clears; }
  if (state.pc == ready_entry) {
    require(
        machine.bus().read({.space = DspSpace::x, .address = cursor_address}) ==
            (count.ready == 0 ? ring_end : ring_begin + 16),
        "Ready must receive the real completed-clear cursor and then its own published successor");
    ++count.ready;
  }
  if (state.pc == original_lookahead) {
    if (count.lookaheads == 0) {
      require(
          state.r[3] == first_lookahead_address && state.sp == 5,
          "Original comb address and genuine caller/comb DO stack must match independent inputs");
    }
    ++count.lookaheads;
  }
  if (state.pc != code.return_address) { return; }
  require(state.sp == 2, "Each native callback RTS must retain only the caller's own DO frame");
  ++count.returned;
  if (count.returned <= clear_calls) { inspect_clear(machine.bus(), count.returned); }
}

Outcome
execute(const Firmware &firmware, const Caller &code, const Scene scene, std::ostream &trace) {
  OwnDspMachine machine(native::native_clocks);
  install(machine.bus(), firmware, scene.diffusion);
  machine.bus().set_unselected_data_read_word(scene.word);
  constexpr unsigned output_base = 0x860;
  for (unsigned index = 0; index < output_words; ++index) {
    put(machine.bus(), DspSpace::y, output_base + index, mask - index);
  }
  native::reset_native(machine, code.program);
  Observation observation{.bus = &machine.bus()};
  Counters count;
  unsigned steps = 0;
  while (machine.cpu().registers().pc != code.final_return) {
    const auto state = machine.cpu().registers();
    require(++steps < maximum_steps, "Original clear-to-ready lifecycle exceeded its bound");
    inspect_boundary(machine, state, code, count);
    const bool input = state.pc >= ready_entry && state.pc < filter_end;
    const auto result = input ? machine.run_observed_unclocked(
                                    {}, {.context = &observation, .completed = observe_write})
                              : machine.run_unclocked();
    if (result.fault.kind != dsp56300::FaultKind::none) {
      if (const auto failure = machine.io().failure(); failure && failure->cause) {
        std::rethrow_exception(failure->cause);
      }
      throw Fault("Original Reverb lifecycle native failure at " + hex(result.fault.pc, 6));
    }
    machine.bus().advance_instruction_cycles(1);
  }
  require(
      count.setup_returns == 1 && count.returned == calls && count.clears == clear_calls &&
          count.ready == 2 && count.lookaheads == expected_lookaheads &&
          machine.cpu().registers().sp == 0 && machine.bus().unselected_reads() == count.lookaheads,
      "All48 original clears and two real ready callbacks must return through the native caller");
  require(observation.used == output_words && machine.bus().unselected_writes() == 1,
          "Only the actual first ready-input write at16000 selects no storage");
  for (unsigned index = 0; index < observation.input.size(); ++index) {
    const auto expected = index == 0 ? ring_end : ring_begin + index;
    require(
        observation.input[index].address == expected &&
            observation.input[index].selected == (index != 0),
        "Actual postincrement stores retain16000 then10001..1001f; no host cursor normalization");
  }
  // From the original coefficient law: T=127+80^2, K=T*(256-32)/2;
  // declared input23729, zero reset histories, floor(23729*731024/2^23)=2067.
  constexpr unsigned expected_first_filter_word = 2067;
  require(observation.input.front().value == expected_first_filter_word,
          "The real first input store must equal the independent original filter expectation");
  original_program(machine.bus(), firmware);
  trace << "SCENE " << scene.live << ' ' << scene.diffusion << ' ' << scene.word << ' '
        << machine.cpu().progress().instructions << ' ' << count.lookaheads << ' ' << count.clears
        << ' ' << count.returned << ' ' << observation.input[0].value << '\n'
        << std::flush;
  return {.registers = machine.cpu().registers(),
          .storage = memory(machine.bus()),
          .input = observation.input,
          .instructions = machine.cpu().progress().instructions,
          .reads = machine.bus().unselected_reads(),
          .writes = machine.bus().unselected_writes()};
}

struct Inputs {
  const char *firmware;
  const char *trace;
};

void run(const Inputs inputs) {
  const auto firmware = Firmware::load(inputs.firmware);
  std::ofstream trace(inputs.trace);
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  require(firmware.dsp.size() == original_record_count,
          "Expected every unchanged original runtime record");
  constexpr unsigned canary = 0xa55aa5;
  for (const bool live : {false, true}) {
    const auto code = caller(live);
    for (const unsigned diffusion : {0U, 64U, 127U}) {
      const auto baseline =
          execute(firmware, code, {.live = live, .diffusion = diffusion, .word = 0}, trace);
      for (const unsigned word : {canary, mask}) {
        require(
            execute(firmware, code, {.live = live, .diffusion = diffusion, .word = word}, trace) ==
                baseline,
            "A vacant word changes original state, mapped memory/knowledge or two-ready outcome");
      }
    }
  }
  std::cout
      << "PASS18scenes/864clears/900callbackRTS/432actualXreads; identical final architecture and "
         "mapped P/X/Y words/knowledge across three data-word hypotheses. "
         "Actual clear-to-ready callback lifecycle; no full connected instrument or physical "
         "timing claim.\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Expected original firmware ZIP and private trace paths");
    run({.firmware = arguments[1], .trace = arguments[2]});
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
