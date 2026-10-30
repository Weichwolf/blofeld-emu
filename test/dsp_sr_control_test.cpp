// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_sr_move.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using test::DspProgram;
using test::require;
constexpr unsigned initial_status = 0x301;
constexpr unsigned masked_negative_status = 0x308;
constexpr unsigned masked_zero_status = 0x304;
constexpr unsigned long_opcode = 0x05f439;
constexpr unsigned short_prefix = 0x0500b9;
constexpr unsigned x0_opcode = 0x04c4b9;
constexpr unsigned r0_opcode = 0x04d0b9;
constexpr unsigned cookie = 0x123456;
constexpr unsigned padding = 64;
constexpr unsigned cycle_budget = 48;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
enum class Source : std::uint8_t { long_immediate, short_immediate, x0, r0 };

struct Input {
  Source source;
  unsigned status;
};

DspFetchRunInputs contract() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory};
}

unsigned append_writer(DspProgram &code, const Input input) {
  const auto writer = code.pc();
  switch (input.source) {
    case Source::long_immediate:
      code.word(long_opcode);
      code.word(input.status);
      break;
    case Source::short_immediate: code.word(short_prefix | (input.status << 8U)); break;
    case Source::x0: code.word(x0_opcode); break;
    case Source::r0: code.word(r0_opcode); break;
    default: throw Fault("Invalid own SR source case");
  }
  return writer;
}

void decoder_checks() {
  struct Expected {
    unsigned opcode;
    unsigned words;
  };

  constexpr std::array cases{Expected{.opcode = long_opcode, .words = 2},
                             Expected{.opcode = short_prefix, .words = 1},
                             Expected{.opcode = x0_opcode, .words = 1},
                             Expected{.opcode = r0_opcode, .words = 1}};
  for (const auto item : cases) {
    const auto decoded = decode_sr_move(item.opcode);
    require(decoded && decoded->words == item.words, "Independent MOVEC SR format word counts");
  }
  constexpr unsigned omr = 0x05f43a;
  constexpr unsigned memory_source = 0x05f039;
  constexpr unsigned wide = 0x1000000;
  require(!decode_sr_move(omr) && !decode_sr_move(memory_source) && !decode_sr_move(wide),
          "Other control destinations,memory inputs and wider words remain outside this route");
}

void one_case(const Input input) {
  DspMachine machine;
  DspProgram code;
  code.word(long_opcode);
  code.word(initial_status);
  unsigned seed_steps = 1;
  if (input.source == Source::x0 || input.source == Source::r0) {
    code.immediate(input.source == Source::x0 ? "x0" : "r0", input.status);
    ++seed_steps;
  }
  const auto writer = append_writer(code, input);
  const auto follower = code.pc();
  code.immediate("x1", cookie);
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned index = 0; index < seed_steps; ++index) { machine.cpu().step(); }
  require(machine.cpu().state().pc == writer && machine.cpu().state().sr == initial_status,
          "Independent native seeds establish the initial architectural status");
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  DspFetchRun run(contract(), machine);
  bool captured = false;
  bool retired = false;
  for (unsigned index = 0; index < cycle_budget; ++index) {
    const auto cycle =
        run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
    if (!cycle.pipeline) { throw Fault("Missing actual automatic pipeline cycle"); }
    const auto &pipeline = *cycle.pipeline;
    if (pipeline.execute1 && pipeline.execute1->address == writer) {
      require(machine.cpu().state().sr == initial_status && !pipeline.transfer,
              "SR E1 remains held;no fabricated ordinary register writeback");
      captured = true;
    }
    if (pipeline.retirement_published && pipeline.execute2 &&
        pipeline.execute2->address == writer) {
      require(captured && machine.cpu().state().sr == input.status,
              "Actual E2 publishes all independently expected status bits");
      retired = true;
    }
    if (pipeline.retirement_published && pipeline.execute2 &&
        pipeline.execute2->address == follower) {
      require(captured && retired && machine.cpu().state().sr == input.status &&
                  (machine.cpu().state().x >> test::word_bits) == cookie,
              "A real subsequent register transfer preserves the native SR result");
      std::cout << "PASS SR_CONTROL source=" << static_cast<unsigned>(input.source)
                << " status=" << hex(input.status, 6) << " cycles=" << run.completed_cycles()
                << '\n';
      return;
    }
  }
  throw Fault("SR control did not complete within the own bounded sequence");
}
}

int main() {
  try {
    decoder_checks();
    for (const unsigned status : {0U,
                                  0x100U,
                                  0x200U,
                                  0x300U,
                                  0x301U,
                                  0x304U,
                                  0x308U,
                                  0x400U,
                                  0x800U,
                                  0x100000U,
                                  0x200000U}) {
      one_case({.source = Source::long_immediate, .status = status});
    }
    one_case({.source = Source::short_immediate, .status = 4});
    one_case({.source = Source::x0, .status = masked_negative_status});
    one_case({.source = Source::r0, .status = masked_zero_status});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
