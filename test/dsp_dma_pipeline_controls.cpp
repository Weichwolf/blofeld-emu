// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp_dma_pipeline_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <string>

namespace blofeld::test::dma_pipeline {
void source_conflict() {
  Rig rig({"move x0,x:(r0)", "nop"}, word_copy);
  rig.until_enable();
  unsigned reads = 0;
  unsigned writes = 0;
  bool triggered = false;
  for (unsigned index = 0; index < 12; ++index) {
    const bool trigger = !triggered && rig.approaching(rig.body_pc.front());
    triggered = triggered || trigger;
    const auto point = rig.tick(DspDmaGrant::available, triggered ? 1U : 0U);
    if (trigger) {
      require(point.bus.dma.phase && point.bus.dma.phase->kind == DspDmaPhaseKind::source_read &&
                  !point.bus.dma.core_stall && !point.bus.dma.word && point.bus.memory &&
                  point.bus.memory->data.value == second &&
                  rig.memory(DspSpace::x, source) == second &&
                  rig.memory(DspSpace::y, destination) == untouched,
              "A fresh DMA source read waits while the real native E1 store owns its partition");
    }
    if (const auto phase = point.bus.dma.phase) {
      if (phase->kind == DspDmaPhaseKind::source_read) { ++reads; }
      if (point.bus.dma.word) {
        ++writes;
        require(point.bus.dma.word->value == second,
                "The deferred DMA read captures the literal actual native store result");
      }
    }
  }
  require(triggered && reads == 2 && writes == 1 && rig.memory(DspSpace::y, destination) == second,
          "One rejected read proposal and one real capture produce exactly one destination word");
}

void captured_source() {
  Rig rig({"move x0,x:(r0)", "nop"});
  rig.until_enable();
  const auto capture = rig.tick();
  require(capture.bus.dma.phase && capture.bus.dma.phase->kind == DspDmaPhaseKind::source_read &&
              !capture.bus.dma.word,
          "The software-triggered channel captures its source before the following core store");
  const auto write = rig.tick();
  require(write.bus.dma.word && write.bus.dma.word->value == first && write.bus.memory &&
              write.bus.memory->data.value == second && !write.bus.dma.core_stall &&
              rig.memory(DspSpace::x, source) == second &&
              rig.memory(DspSpace::y, destination) == first,
          "Real core source overwrite overlaps independent DMA destination and preserves capture");
}

void destination_hold_restore() {
  Rig rig({"move x0,y:(r1)", "nop"});
  rig.until_enable();
  const auto captured = rig.tick();
  require(captured.bus.dma.phase && captured.bus.dma.phase->kind == DspDmaPhaseKind::source_read,
          "The destination-conflict fixture starts from an actually captured word");
  const auto checkpoint = rig.run().checkpoint(rig.machine);
  for (unsigned suffix = 0; suffix < 2; ++suffix) {
    rig.run().restore(checkpoint, rig.machine);
    for (unsigned held = 0; held < 3; ++held) {
      const auto before = rig.machine.cpu().debug_state();
      const auto pipeline = rig.pipeline();
      const auto fetch = rig.machine.reference_fetch();
      const auto count = rig.run().completed_cycles();
      const auto point = rig.tick(DspDmaGrant::wait);
      const auto after = rig.pipeline();
      require(
          point.bus.dma.core_stall && !point.bus.dma.word &&
              rig.machine.cpu().debug_state() == before && after.stages == pipeline.stages &&
              after.awaiting_retirement == pipeline.awaiting_retirement &&
              rig.machine.reference_fetch().pending_address == fetch.pending_address &&
              rig.run().completed_cycles() == count + 1 &&
              rig.memory(DspSpace::y, destination) == untouched,
          "A held destination freezes actual CPU/E2/AG2/F2 owners while reference time advances");
    }
    const auto before = rig.machine.cpu().debug_state();
    const auto completed = rig.tick();
    require(completed.bus.dma.core_stall && completed.bus.dma.word &&
                completed.bus.dma.word->value == first &&
                rig.machine.cpu().debug_state() == before &&
                rig.memory(DspSpace::y, destination) == first,
            "The final DMA destination write still owns the conflicting cycle");
    const auto resumed = rig.tick();
    require(!resumed.bus.dma.core_stall && resumed.bus.memory &&
                resumed.bus.memory->data.value == second &&
                rig.memory(DspSpace::y, destination) == second,
            "The retained actual native store resumes exactly after the DMA write completes");
    static_cast<void>(rig.tick());
    require(rig.machine.cpu().state().pc >= rig.body_pc.front() + 1,
            "The native pipeline retires the resumed instruction without replaying the DMA word");
  }
}

void disable_captured_word() {
  Rig rig({"movep #>" + test::number(software_copy & ~enable) + ",x:<<$ffffec", "nop"},
          software_copy,
          3);
  rig.until_enable();
  const auto captured = rig.tick();
  require(captured.bus.dma.phase && captured.bus.dma.phase->kind == DspDmaPhaseKind::source_read,
          "Disabling starts with an actual pending source word");
  bool disabled = false;
  for (unsigned index = 0; index < 8 && !disabled; ++index) {
    const auto point = rig.tick(DspDmaGrant::wait);
    disabled = point.bus.memory && point.bus.memory->data.target.address == control_register;
  }
  require(disabled && (rig.memory(DspSpace::x, control_register) & enable) == 0 &&
              rig.memory(DspSpace::y, destination) == untouched,
          "A genuine native DE clear is accepted while the captured destination is held");
  const auto final = rig.tick();
  require(
      final.bus.dma.word && final.bus.dma.word->value == first &&
          rig.memory(DspSpace::y, destination) == first &&
          rig.memory(DspSpace::x, counter_register) == 2,
      "FM10.5.3.5: DE clear finishes the current word rather than discarding it or the full block");
  for (unsigned index = 0; index < 8; ++index) {
    const auto point = rig.tick();
    require(!point.bus.dma.phase && !point.bus.dma.word,
            "The stopped software-triggered channel cannot transfer another word");
  }
  require(rig.memory(DspSpace::y, destination + 1) == untouched,
          "Remaining destination words are unchanged after the captured transfer completes");
}

void controller_guards() {
  for (const auto address :
       {source_register, destination_register, counter_register, control_register}) {
    const unsigned value = address == control_register ? word_copy ^ 1U : 1U;
    Rig rig({"movep #>" + test::number(value) + ",x:<<" + test::number(address)}, word_copy);
    rig.until_enable();
    const auto before = rig.memory(DspSpace::x, address);
    const auto message = reject(
        [&] {
          for (unsigned index = 0; index < 8; ++index) { static_cast<void>(rig.tick()); }
        },
        "Busy DMA configuration must remain visibly refused");
    require(message.find("DSP DMA register write") != std::string::npos && rig.run().faulted() &&
                rig.memory(DspSpace::x, address) == before &&
                rig.memory(DspSpace::y, destination) == untouched,
            "The controller's atomic busy-register guard survives native pipeline integration");
    static_cast<void>(reject([&] { static_cast<void>(rig.run().checkpoint(rig.machine)); },
                             "Faulted automatic owners cannot export successful checkpoints"));
  }
  Rig peripheral({"movep #>0,x:<<$fffffc"}, word_copy);
  peripheral.until_enable();
  const auto message = reject(
      [&] {
        for (unsigned index = 0; index < 8; ++index) { static_cast<void>(peripheral.tick()); }
      },
      "Unqualified concurrent peripheral accesses must remain refused");
  require(message.find("with active DMA") != std::string::npos &&
              !peripheral.machine.bus().once_latch(),
          "A controller-port exception does not remove other active-DMA peripheral guards");
}

void request_knowledge_restore() {
  Rig rig({"nop"}, word_copy);
  rig.until_enable();
  const auto checkpoint = rig.run().checkpoint(rig.machine);
  const auto before = rig.machine.cpu().debug_state();
  const auto ticks = rig.run().elapsed_ticks();
  const auto message = reject([&] { static_cast<void>(rig.tick(DspDmaGrant::available, 0, 0)); },
                              "Unknown requests must not turn into a successful idle cycle");
  require(message.find("Unknown DSP DMA request input") != std::string::npos &&
              rig.machine.cpu().debug_state() == before && rig.run().elapsed_ticks() == ticks &&
              rig.memory(DspSpace::y, destination) == untouched,
          "Missing request knowledge fails before CPU execution, clock completion or destination");
  rig.run().restore(checkpoint, rig.machine);
  const auto captured = rig.tick(DspDmaGrant::available, 1);
  const auto stored = rig.tick(DspDmaGrant::available, 1);
  require(captured.bus.dma.phase && !captured.bus.dma.word && stored.bus.dma.word &&
              stored.bus.dma.word->value == first && rig.memory(DspSpace::y, destination) == first,
          "A trusted joint restore uses a real fresh known request and captured source data");
}
}
