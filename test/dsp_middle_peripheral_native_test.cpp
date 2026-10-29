// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using test::DspProgram;
constexpr unsigned bit16_mode = 0x020000;
constexpr unsigned ordinary_sr = 0xc00300;
constexpr unsigned raw_mask = 0xffffff;
constexpr unsigned shift_msp = 24;
constexpr unsigned extension_shift = 48;
constexpr unsigned extension_mask = 0xff;
constexpr unsigned half_byte = 8;
constexpr unsigned seed_count = 7;
constexpr std::uint64_t other = 0x11223344556677;
constexpr std::array encodings{
    0x08cc3cU, 0x09cc35U, 0x04cc90U, 0x04cc3fU, 0x08cd3cU, 0x09cd35U, 0x04cd90U, 0x04cd3fU};
constexpr std::array<std::uint64_t, 7> values{0,
                                              0x7f7fffff123456,
                                              0x80800000654321,
                                              0xffffffffffabcd,
                                              0x55123456abcdef,
                                              0x80654321112233,
                                              0x12000001345678};
constexpr std::array scales{0U, 0x400U, 0x800U};
constexpr std::array flags{0U, 0x3fU, 0xc0U, 0xffU};
unsigned cases = 0;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

struct Sample {
  bool b;
  std::uint64_t value;
  unsigned scale;
  unsigned ccr;
  bool arithmetic16;
};

void one(const Sample sample) {
  DspMachine machine;
  DspProgram code;
  for (const bool b : {false, true}) {
    const auto value = b == sample.b ? sample.value : other;
    const std::string prefix = b ? "b" : "a";
    code.immediate(prefix + '0', value & raw_mask);
    code.immediate(prefix + '1', (value >> shift_msp) & raw_mask);
    code.immediate(prefix + '2', (value >> extension_shift) & extension_mask);
  }
  const auto sr = ordinary_sr | sample.scale | sample.ccr | (sample.arithmetic16 ? bit16_mode : 0U);
  code.immediate("sr", sr);
  const auto pc = code.pc();
  code.word(encodings.at(sample.b ? 4U : 0U));
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  for (unsigned i = 0; i < seed_count; ++i) { machine.cpu().step(); }
  const auto before = machine.cpu().debug_state();
  require(before.cpu.pc == pc && before.cpu.sr == sr &&
              (sample.b ? before.cpu.b : before.cpu.a) == sample.value,
          "Own native part/SR seed must match the independent56bit value");
  require(!machine.bus().once_latch(), "The native seeds do not write OnCE");
  auto expected = before.cpu;
  ++expected.pc;
  auto word = static_cast<unsigned>((sample.value >> shift_msp) & raw_mask);
  // FM3.1.3 and3.5/Table3-4: raw MSP bypasses scaling/limiting; SA exports16MSBs.
  if (sample.arithmetic16) { word >>= half_byte; }
  machine.cpu().step();
  const auto after = machine.cpu().debug_state();
  const auto latch = machine.bus().once_latch();
  if (latch != word || after.cpu != expected) {
    throw Fault("Native MOVEP middle mismatch source=" + std::string(sample.b ? "b1" : "a1") +
                " value=" + test::number(sample.value) + " sr=" + test::number(sr) + " expected=" +
                test::number(word) + " actual=" + (latch ? test::number(*latch) : "none") +
                " after-sr=" + test::number(after.cpu.sr));
  }
  require(after.native_instructions == before.native_instructions + 1 &&
              after.native_cycles == before.native_cycles + 1,
          "FMTableA-1: raw register MOVEP retires one instruction/nominal native clock");
  ++cases;
}

void native_checks() {
  for (const auto opcode : encodings) {
    require(decode_memory_stage(opcode).has_value(),
            "All eight independently assembled raw-middle peripheral stores are admitted");
  }
  for (const bool b : {false, true}) {
    for (const auto value : values) {
      for (const auto scale : scales) {
        for (const auto ccr : flags) {
          for (const bool sa : {false, true}) {
            one({.b = b, .value = value, .scale = scale, .ccr = ccr, .arithmetic16 = sa});
          }
        }
      }
    }
  }
}
}

int main() {
  try {
    native_checks();
    std::cout << "Native MOVEP middle semantics: cases=" << cases
              << " admitted-encodings=" << encodings.size() << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
