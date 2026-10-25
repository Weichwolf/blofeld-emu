// SPDX-License-Identifier: GPL-3.0-only
#include "dsp_voice_master.hpp"

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp_norm_fixture.hpp"

#include <ios>
#include <ostream>

namespace blofeld::test {
namespace {
constexpr unsigned voice_bus = 0x5a0;
constexpr unsigned main_bus = 0x620;
constexpr unsigned dma_buffer = 0x660;
constexpr unsigned history = 0x46;
constexpr unsigned phase_slot = 0x802;
constexpr unsigned shared_bus_slot = 0x12;
constexpr unsigned master_gain = 0x17;
constexpr unsigned original_copy = 0x298;
constexpr unsigned original_mix = 0x251;
constexpr unsigned original_master = 0x27b;
constexpr unsigned chain_gain = 0x6a0;
constexpr unsigned flags = 6;
constexpr unsigned replace = 1U << 6U;
constexpr unsigned signal = 1U << 9U;
constexpr unsigned words = 32;
constexpr unsigned maximum_volume = 127;

unsigned read(DspBus &bus, const DspSpace space, const unsigned address) {
  return bus.read({.space = space, .address = address});
}

void row(DspBus &bus,
         std::ostream &trace,
         const char *label,
         const DspSpace space,
         const unsigned base) {
  trace << label << std::hex;
  for (unsigned i = 0; i < words; ++i) { trace << ' ' << read(bus, space, base + i); }
  trace << '\n' << std::dec;
}
}

void VoiceMaster::append(DspProgram &program) {
  entry = program.pc();
  program.immediate("r0", voice_bus);
  program.immediate("r3", original_copy);
  program.emit("jsr (r3)");
  copied = program.pc();
  program.immediate("r5", voice_bus);
  program.emit("move x:>" + number(chain_gain) + ",y0");
  program.emit("move x:>" + number(chain_gain + 1) + ",y1");
  program.immediate("r3", original_mix);
  program.emit("bset #6,x:<" + number(flags));
  program.emit("jsr (r3)");
  mixed = program.pc();
  program.emit("bclr #6,x:<" + number(flags));
  program.immediate("r0", main_bus);
  program.immediate("r1", dma_buffer);
  program.emit("move x:>" + number(phase_slot) + ",n1");
  program.immediate("r3", original_master);
  program.emit("jsr (r3)");
  filtered = program.pc();
  // Explicit diagnostic alternation, never a manufactured DMA observation.
  program.immediate("r0", phase_slot);
  program.emit("bchg #5,x:(r0)");
  program.emit("rts");
}

void VoiceMaster::initialize(DspBus &bus) {
  const auto put = [&](const DspSpace space, const unsigned at, const unsigned value) {
    bus.write({.space = space, .address = at, .value = value});
  };
  require(read(bus, DspSpace::x, chain_gain) == 0 && read(bus, DspSpace::x, chain_gain + 1) == 0,
          "The original default chain must retain both zero effect gains");
  require((read(bus, DspSpace::x, flags) & replace) == 0,
          "The own caller must not leave replacement mixing enabled before rendering");
  put(DspSpace::x, master_gain, 512 * maximum_volume * maximum_volume);
  put(DspSpace::x, phase_slot, 0);
  // Declared readable neighbors and cold histories, not recovered power-on RAM.
  for (unsigned i = 0; i < words * 4; ++i) { put(DspSpace::x, main_bus + i, 0); }
  for (unsigned i = 0; i < 2; ++i) {
    put(DspSpace::x, voice_bus + words + i, 0);
    put(DspSpace::x, history + i, 0);
    put(DspSpace::y, history + i, 0);
  }
  put(DspSpace::y, dma_buffer, 0);
  auto &port = bus.gpio(DspPort::e);
  port.write({.reg = DspGpioRegister::direction, .value = signal, .mask = signal});
  port.write({.reg = DspGpioRegister::data, .value = 0, .mask = signal});
}

void VoiceMaster::observe(DspBus &bus, const DspState &state, std::ostream &trace) const {
  if (state.pc == copied) {
    require((read(bus, DspSpace::x, flags) & replace) == 0,
            "Actual voice and copy calls precede replacement-mode selection");
    row(bus, trace, "MASTER_COPY", DspSpace::y, read(bus, DspSpace::x, shared_bus_slot));
  }
  if (state.pc == mixed) {
    require((read(bus, DspSpace::x, flags) & replace) != 0,
            "The native caller must select actual replacement mixing");
    row(bus, trace, "MASTER_MIX", DspSpace::x, main_bus);
    row(bus, trace, "MASTER_CLEARED", DspSpace::x, voice_bus);
    row(bus, trace, "MASTER_SEND", DspSpace::x, main_bus + words);
  }
  if (state.pc != filtered) { return; }
  require((read(bus, DspSpace::x, flags) & replace) == 0,
          "The native caller must clear replacement mode before master output");
  const auto phase = read(bus, DspSpace::x, phase_slot);
  require(phase == 0 || phase == words, "Own caller phase must select a complete output half");
  trace << "MASTER_PHASE " << phase << '\n';
  row(bus, trace, "MASTER_OUTPUT", DspSpace::x, dma_buffer + phase);
  row(bus, trace, "MASTER_OTHER", DspSpace::x, dma_buffer + words - phase);
  trace << "MASTER_HISTORY" << std::hex;
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    for (unsigned i = 0; i < 2; ++i) { trace << ' ' << read(bus, space, history + i); }
  }
  trace << '\n'
        << "MASTER_RETURN " << state.a << ' ' << state.b << ' ' << state.sp << ' ' << state.r[0]
        << ' ' << state.r[1] << ' ' << state.r[2] << ' ' << state.r[5] << '\n'
        << std::dec;
}
}
