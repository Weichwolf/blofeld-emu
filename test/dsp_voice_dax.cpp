// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_voice_dax.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_dax.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_voice_repeat.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <ostream>

namespace blofeld::test {
namespace {
constexpr unsigned source3 = 0xffffe3;
constexpr unsigned dax_audio = 0xffffd2;
constexpr unsigned control0 = 0xffffec;
constexpr unsigned control3 = 0xffffe0;
constexpr unsigned original_control0 = 0xaab604;
constexpr unsigned original_control3 = 0xa8b600;
constexpr unsigned original_enable = 0x2ebb;
constexpr unsigned original_return = 0x2ec2;
constexpr unsigned original_count = 4100; // Two MOVEP, REP,4095 NOP, BSET,RTS.
constexpr unsigned ring = 0x660;
constexpr unsigned half_words = 32;
constexpr unsigned ring_count = 0xfc0;
constexpr unsigned half_count = 0x7c0;
constexpr unsigned units_per_frame = 128;
constexpr unsigned frames_per_half = 16;
constexpr unsigned maximum_halves = 12;
constexpr unsigned core_rate = 54000000;
constexpr unsigned aci_rate = 12288000;

unsigned read(DspBus &bus, const unsigned at) {
  return bus.read({.space = DspSpace::x, .address = at});
}
}

void VoiceDax::append(DspProgram &program) {
  constexpr std::array preloads{
      DspStore{.space = DspSpace::x, .address = 0xfffff1, .value = 1},
      DspStore{.space = DspSpace::x, .address = 0xfffff0, .value = 0xffffc1},
      DspStore{.space = DspSpace::x, .address = source3, .value = ring},
      DspStore{.space = DspSpace::x, .address = source3 - 1, .value = dax_audio},
      DspStore{.space = DspSpace::x, .address = source3 - 2, .value = ring_count},
      DspStore{.space = DspSpace::y, .address = 0xffff9e, .value = 0xa3c},
      DspStore{.space = DspSpace::y, .address = 0xffff9d, .value = 0x200},
      DspStore{.space = DspSpace::x, .address = 0xffffd1, .value = 0},
      DspStore{.space = DspSpace::x, .address = dax_audio, .value = 0},
      DspStore{.space = DspSpace::x, .address = dax_audio, .value = 0},
      DspStore{.space = DspSpace::x, .address = 0xffffd0, .value = 8},
      DspStore{.space = DspSpace::x, .address = 0xffffd7, .value = 3},
      DspStore{.space = DspSpace::x, .address = 0xffffd6, .value = 3}};
  for (const auto store : preloads) {
    program.emit("movep #>" + number(store.value) +
                 (store.space == DspSpace::x ? ",x:<<" : ",y:<<") + number(store.address));
  }
  // Explicit original suffix entry, not a continuation past a rejected initializer.
  program.immediate("r6", original_enable);
  program.emit("jsr (r6)");
  caller_return = program.pc();
  program.immediate("r6", 0); // Restore the declared foreground caller's scratch precondition.
}

void VoiceDax::begin(DspBus &bus) {
  words = total_words = 0;
  edge = original_begin = original_steps = 0;
  transfers = {};
  bmc.clear();
  bus.observe_dax({.core_clock = false,
                   .aci = false,
                   .rates = {.core_hz = core_rate, .aci_hz = aci_rate},
                   .timing = DspDaxTiming::reference_rising_edges});
}

void VoiceDax::observe(DspBus &bus, const std::uint64_t at) {
  require(at == edge + 1, "DAX reference observations must retain every declared edge");
  edge = at;
  bus.observe_dax({.core_clock = false,
                   .aci = (edge & 1U) != 0,
                   .rates = {.core_hz = core_rate, .aci_hz = aci_rate},
                   .timing = DspDaxTiming::reference_rising_edges});
  if (edge % 4 != 3) { return; }
  const auto output = bus.dax_output();
  require(output.driven, "Actual DAX ADO must drive every expected biphase unit");
  bmc += output.level ? '1' : '0';
}

void VoiceDax::accept(const DspDmaWord &word) {
  require(word.channel == 3 && words < transfers.size(),
          "Secondary audio DMA cannot drop a word or change its channel");
  transfers.at(words++) = word;
  ++total_words;
}

void VoiceDax::verify(DspBus &bus,
                      const unsigned half,
                      const std::array<DspDmaWord, 32> &primary) const {
  const auto start = ring + ((half % 2) * half_words);
  const auto next = ring + (((half + 1) % 2) * half_words);
  require(words == half_words && read(bus, source3) == next &&
              read(bus, source3 - 1) == dax_audio &&
              read(bus, source3 - 2) == (half % 2 == 0 ? half_count : ring_count) &&
              read(bus, control3) == original_control3,
          "Original DMA3 retains its fixed DAX destination, DE and complete ring reload");
  for (unsigned index = 0; index < words; ++index) {
    const auto &word = transfers.at(index);
    require(word.source == DspDmaAddress{.address = start + index} &&
                word.destination == DspDmaAddress{.address = dax_audio} &&
                word.value == primary.at(index).value,
            "Actual DMA3 reads the same independently checked original ring words as DMA0");
  }
}

void VoiceDax::emit(std::ostream &trace) const {
  trace << std::hex;
  for (unsigned index = 0; index < words; ++index) {
    const auto &word = transfers.at(index);
    trace << "DUAL_DAX_DMA " << index << ' ' << word.source.address << ' '
          << word.destination.address << ' ' << word.value << '\n';
  }
  trace << std::dec;
}

void VoiceDax::finish(DspBus &bus, const unsigned halves, std::ostream &trace) {
  require(halves > 0 && halves <= maximum_halves, "Declared dual-audio stream half bound");
  const auto frames = (halves * frames_per_half) + 1;
  const auto units = static_cast<std::size_t>(frames) * units_per_frame;
  // HCKT/core/DMA stay paused; actual ACI edges finish the already captured B payload.
  while (bmc.size() < units) { observe(bus, edge + 1); }
  require(bmc.size() == units && edge == 3 + (4 * (units - 1)) &&
              total_words == halves * half_words && original_steps == original_count,
          "The complete DAX tail retains priming, payload, phase and original suffix scope");
  trace << std::hex << "DUAL_SETUP " << original_steps << ' ' << read(bus, control0) << ' '
        << read(bus, control3) << '\n';
  trace << "DUAL_FINAL " << total_words << ' ' << frames << ' ' << bmc.size() << ' ' << edge << '\n'
        << "DUAL_BMC " << bmc << '\n'
        << std::dec;
}

void VoiceRepeat::setup_instruction(DspMachine &machine) {
  const auto pc = machine.cpu().state().pc;
  if (dax.enabled && pc == original_enable) {
    dax.original_begin = machine.cpu().debug_state().native_instructions;
  }
  instruction(machine);
  if (dax.enabled && pc == original_return) {
    dax.original_steps = machine.cpu().debug_state().native_instructions - dax.original_begin;
    require(dax.original_steps == original_count && machine.cpu().state().sp == 1 &&
                machine.cpu().state().pc == dax.caller_return &&
                read(machine.bus(), control0) == original_control0 &&
                read(machine.bus(), control3) == original_control3,
            "Actual original enables, REP4095/NOP, BSET and RTS retain their real native frame");
  }
}

void VoiceRepeat::finalize(DspBus &bus, const DspCore &cpu, std::ostream &trace) {
  if (!dax.enabled) { return; }
  const auto before = cpu.debug_state();
  dax.finish(bus, half, trace);
  require(cpu.debug_state() == before, "DAX tail clocking cannot execute a hidden DSP instruction");
}
} // namespace blofeld::test
