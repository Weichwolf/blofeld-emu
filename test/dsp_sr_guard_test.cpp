// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp/dsp_sr_move.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using test::DspProgram;
using test::require;
// DSP56300FM 13-130/131: MOVEC #imm24,SR and direct source field eeeeee.
constexpr unsigned immediate_sr = 0x05f439;
constexpr unsigned direct_sr = 0x04c0b9;
constexpr unsigned masked_status = 0x301;
constexpr unsigned reset_omr = 0x305;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned word_bits = 24;
constexpr unsigned cookie = 0x123456;
constexpr unsigned padding = 64;
constexpr unsigned extal = 12000000;
constexpr unsigned timebase = 540000000;
constexpr unsigned timer_csr = 0xffff8f;
constexpr unsigned timer_load = 0xffff8e;
constexpr unsigned timer_compare = 0xffff8d;
constexpr unsigned interrupt_priority = 0xfffffe;
constexpr unsigned timer_priority = 0x100;

DspFetchRunInputs contract() {
  return {.extal_hz = extal,
          .timebase_hz = timebase,
          .timing = DspFetchTiming::reference_cycle_order,
          .execution = DspReferenceExecution::staged_register_boundaries,
          .pipeline = DspFetchPipeline::automatic_memory};
}

void install(DspMachine &machine, DspProgram &code, const unsigned seeds) {
  for (unsigned index = 0; index < padding; ++index) { code.word(0); }
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  for (unsigned index = 0; index < seeds; ++index) { machine.cpu().step(); }
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
}

void direct_sources() {
  constexpr std::array names{
      "x0", "x1", "y0", "y1", "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"};
  for (unsigned index = 0; index < names.size(); ++index) {
    const unsigned field = index < 4 ? index + 4 : index + 12;
    const unsigned status = 0x300U | field;
    DspMachine machine;
    DspProgram code;
    code.word(immediate_sr);
    code.word(masked_status);
    code.immediate(names.at(index), status);
    const auto writer = code.pc();
    code.word(direct_sr | (field << 8U));
    const auto follower = code.pc();
    code.immediate("y0", cookie);
    install(machine, code, 2);
    auto expected = machine.cpu().state();
    expected.pc = follower + 2;
    expected.sr = status;
    expected.y = (expected.y & (std::uint64_t{word_mask} << word_bits)) | cookie;
    DspFetchRun run(contract(), machine);
    bool writer_retired = false;
    bool done = false;
    for (unsigned cycle = 0; cycle < padding && !done; ++cycle) {
      const auto step =
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available});
      if (!step.pipeline || !step.pipeline->retirement_published) { continue; }
      const auto word = step.pipeline->execute2;
      if (!word) { throw Fault("Missing direct-source retirement identity"); }
      if (word->address == writer) {
        require(machine.cpu().state().sr == status,
                "Each literal X/Y/R source writes its actual value");
        writer_retired = true;
      } else {
        require(word->address == follower && writer_retired && machine.cpu().state() == expected,
                "Complete direct-source architectural endpoint includes the genuine follower");
        done = true;
      }
    }
    require(done, "Every supported direct SR source completes within the own bounded program");
  }
}

void decoder_guards() {
  for (unsigned field = 0; field < 64; ++field) {
    const bool allowed = (field >= 4 && field <= 7) || (field >= 16 && field <= 23);
    require(
        decode_sr_move(direct_sr | (field << 8U)).has_value() == allowed,
        "Literal FM direct fields admit exactly X0/X1/Y0/Y1/R0..R7, retaining control/SSH guards");
  }
  constexpr std::array refused{0x05f039U, 0x05f43bU, 0x1000000U};
  for (const auto opcode : refused) {
    require(!decode_sr_move(opcode),
            "Memory inputs, other control targets and wide words are excluded");
    bool failed = false;
    try {
      static_cast<void>(decode_pipeline_word(
          {.address = 0x100, .value = opcode}, reset_omr, DspFetchPipeline::automatic_memory));
    } catch (const Fault &error) {
      const std::string message = error.what();
      require(message.contains("P:0x000100") && message.contains("pc=") && message.contains("omr="),
              "Unsupported SR-like forms report the actual address and execution context");
      failed = true;
    }
    require(failed, "Unsupported SR-like forms cannot silently become ordinary register transfers");
  }
}

void mode_guards() {
  // DSP56300FM SR: CE, SA, FV and SC remain outside the qualified automatic contract.
  for (const unsigned mode : {0x80000U, 0x20000U, 0x10000U, 0x2000U}) {
    DspMachine machine;
    DspProgram code;
    code.word(immediate_sr);
    code.word(masked_status | mode);
    code.word(0);
    install(machine, code, 1);
    const auto before = machine.cpu().debug_state();
    bool failed = false;
    std::uint64_t cycles = 0;
    try {
      DspFetchRun run(contract(), machine);
      try {
        static_cast<void>(
            run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
      } catch (...) {
        cycles = run.completed_cycles();
        throw;
      }
    } catch (const Fault &error) {
      const std::string message = error.what();
      const bool unsupported_pending =
          (mode == 0x10000U || mode == 0x2000U) &&
          message.contains("Program-fetch start requires an ordinary pending opcode");
      require((message.contains("mode/interrupt") || unsupported_pending) && cycles == 0 &&
                  machine.cpu().debug_state() == before,
              "Unsupported modes fail before native execution or any automatic stage clock");
      failed = true;
    }
    require(failed, "SR support does not grant unqualified processor modes");
  }
}

void pending_interrupt() {
  DspMachine machine;
  DspProgram code;
  code.word(immediate_sr);
  code.word(masked_status);
  code.word(immediate_sr);
  code.word(0);
  code.immediate("x1", cookie);
  install(machine, code, 1);
  for (const auto store :
       {DspStore{.space = DspSpace::x, .address = timer_csr},
        DspStore{.space = DspSpace::x, .address = interrupt_priority, .value = timer_priority},
        DspStore{.space = DspSpace::x, .address = timer_load, .value = 1},
        DspStore{.space = DspSpace::x, .address = timer_compare, .value = 1},
        DspStore{.space = DspSpace::x, .address = timer_csr, .value = 7}}) {
    machine.bus().write(store);
  }
  machine.bus().timers().advance_clk2(1);
  require(machine.bus().interrupt_request(0).has_value() && !machine.bus().interrupt_request(3),
          "Actual timer IRQ exists and is initially masked by the real SR");
  DspFetchRun run(contract(), machine);
  bool failed = false;
  try {
    for (unsigned cycle = 0; cycle < padding; ++cycle) {
      static_cast<void>(
          run.advance_cycle(machine, {.known_requests = 15, .grant = DspDmaGrant::available}));
    }
  } catch (const Fault &error) {
    const std::string message = error.what();
    require(message.contains("interrupt") && message.contains("pc=") &&
                machine.bus().interrupt_request(0).has_value(),
            "Unqualified pending IRQ stays visible and unacknowledged across an SR unmask");
    failed = true;
  }
  require(failed, "A pending interrupt cannot be silently skipped to execute the follower");
}
}

int main() {
  try {
    direct_sources();
    std::cout << "PASS SR_DIRECT sources=12\n";
    decoder_guards();
    std::cout << "PASS SR_DECODE fields=64 refusals=3\n";
    mode_guards();
    std::cout << "PASS SR_MODES refused=4\n";
    pending_interrupt();
    std::cout << "PASS SR_GUARDS direct=12 fields=64 decode-refusals=3 modes=4 pending-IRQ=1\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
