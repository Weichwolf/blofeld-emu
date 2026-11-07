// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_boot_context.hpp"
#include "dsp_boot_reference.hpp"
#include "firmware/firmware.hpp"
#include "original_dsp_reference.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <span>
#include <string>

namespace {
constexpr unsigned bootstrap_base = 0xff0000;
constexpr unsigned mcu_frame_offset = 0x7875;
constexpr std::size_t stage_words = 94;
constexpr unsigned stage_base = 0x6a;
constexpr unsigned instruction_limit = 20000;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned pll_pc = 0x6f;
constexpr unsigned clear_pc = 0x72;
constexpr unsigned known_long_ram_end = 0x2000;
constexpr unsigned clear_count = 0x20000;
constexpr unsigned bcr = 0xfffffb;
constexpr unsigned bcr_reset = 0x1fffff;
constexpr unsigned word_bits = 24;
enum class Execution : std::uint8_t { instruction, access };

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

class Runner {
public:
  Runner(blofeld::DspMachine &machine, std::ostream &trace, const Execution execution)
      : machine_(machine), trace_(trace), execution_(execution) {}

  void step() {
    require(completed_ < instruction_limit, "DSP bootstrap instruction budget exhausted");
    const auto before = machine_.cpu().registers();
    trace_ << completed_ << " pc=" << blofeld::hex(before.pc, 6)
           << " sr=" << blofeld::hex(before.sr, 6) << " omr=" << blofeld::hex(before.omr, 6)
           << " r0=" << blofeld::hex(before.r[0], 6) << " r1=" << blofeld::hex(before.r[1], 6)
           << " lc=" << blofeld::hex(before.lc, 6) << '\n';
    if (execution_ == Execution::instruction) {
      blofeld::test::dma_test::step_native(machine_);
    } else {
      auto progress = machine_.begin_fetch(dsp56300::InstructionPublication::retirement);
      constexpr unsigned maximum_effects = 64;
      for (unsigned effect = 0; !progress.retired && effect < maximum_effects; ++effect) {
        blofeld::test::require_original_native(machine_, progress);
        if (!progress.pending) {
          throw blofeld::Fault("Original instruction lost its pending native effect");
        }
        progress = machine_.advance(progress.pending->ticket);
      }
      blofeld::test::require_original_native(machine_, progress);
      require(progress.retired && !progress.pending &&
                  progress.fault.kind == dsp56300::FaultKind::none,
              "Each original native instruction retires within its actual effect bound");
    }
    ++completed_;
  }

  void send(const std::uint32_t value) {
    auto &shi = machine_.bus().shi();
    shi.observe_spi({.selected = true, .mosi = false});
    step();
    for (unsigned bit = 0; bit < word_bits; ++bit) {
      const bool level = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
      shi.observe_spi({.selected = true, .clock = true, .mosi = level});
      step();
      shi.observe_spi({.selected = true, .mosi = level});
      step();
    }
    shi.observe_spi({.mosi = false});
    step();
  }

  [[nodiscard]] unsigned completed() const { return completed_; }

private:
  blofeld::DspMachine &machine_;
  std::ostream &trace_;
  Execution execution_;
  unsigned completed_ = 0;
};

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
};

blofeld::test::boot_context::State run(const Inputs &inputs, const Execution execution) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  const auto rom = blofeld::test::read_spi_boot_prefix(inputs.rom);
  blofeld::DspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  bus.install_rom({.start = bootstrap_base, .words = rom});
  bus.shi().observe_spi({.mosi = false});
  const auto &cpu = machine.cpu();
  constexpr std::uint64_t reset_hold = 90;
  constexpr std::uint64_t reset_recovery = 90;
  machine.begin_hardware_reset(
      {.mode_known = 15, .mode_levels = 5, .pinit = false, .recovery_cycles = 2});
  require(machine.advance_hardware_reset_ticks(reset_hold) == reset_hold, "Actual reset hold");
  machine.release_hardware_reset();
  require(machine.advance_hardware_reset_ticks(reset_recovery) == reset_recovery,
          "Actual reset recovery");
  std::ofstream context(inputs.trace.string() + ".boot-context.trace");
  context.exceptions(std::ios::failbit | std::ios::badbit);
  blofeld::test::boot_context::record(blofeld::test::boot_context::Phase::reset,
                                      blofeld::test::boot_context::capture(machine),
                                      context);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private DSP bootstrap trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Runner runner(machine, trace, execution);
  while ((bus.shi().read(hcsr) & 1U) == 0) { runner.step(); }
  require(bus.once_latch() == 0,
          "Documented bootstrap performs its own OnCE GDB initialization write");
  const auto frame =
      std::span<const std::uint8_t>(firmware.mcu).subspan(mcu_frame_offset, (stage_words + 2) * 3);
  require(blofeld::test::dsp_boot_word(frame.first<3>()) == stage_words &&
              blofeld::test::dsp_boot_word(frame.subspan<3, 3>()) == stage_base,
          "Unmodified MCU image supplies the independently identified first-stage frame");
  for (std::size_t offset = 0; offset < frame.size(); offset += 3) {
    runner.send(
        blofeld::test::dsp_boot_word(std::span<const std::uint8_t, 3>(frame.subspan(offset, 3))));
  }
  while (cpu.registers().pc != stage_base) { runner.step(); }
  require(bus.read({.space = blofeld::DspSpace::x, .address = bcr}) == bcr_reset,
          "The executed published SPI prefix supplies no Port F BIU setup");
  blofeld::test::boot_context::record(blofeld::test::boot_context::Phase::loaded,
                                      blofeld::test::boot_context::capture(machine),
                                      context);
  require(cpu.registers().sp == 0 && cpu.registers().r[0] == stage_base + stage_words &&
              cpu.registers().r[1] == stage_base,
          "Native ROM DO loop, stack and jump complete without host PC patching");
  for (std::size_t index = 0; index < stage_words; ++index) {
    const auto expected = blofeld::test::dsp_boot_word(
        std::span<const std::uint8_t, 3>(frame.subspan((index + 2) * 3, 3)));
    require(bus.read({.address = static_cast<std::uint32_t>(stage_base + index)}) == expected,
            "Every original first-stage word was written by executed DSP MOVEP instructions");
  }
  while (cpu.registers().pc != pll_pc) { runner.step(); }
  constexpr unsigned original_memory_mode = 0x600389;
  require(
      cpu.registers().omr == original_memory_mode && cpu.registers().r[4] == 0,
      "Unmodified first-stage instructions select their own memory mode and clear their pointer");
  runner.step();
  require(bus.pll().read() == bus.read({.address = pll_pc + 1}),
          "Original MOVEP writes its own unmodified PCTL operand");
  runner.step();
  require(cpu.registers().pc == clear_pc && cpu.registers().lc == clear_count,
          "Fractional short immediate X0 supplies the full 24-bit REP count, not integer two");
  blofeld::test::boot_context::record(blofeld::test::boot_context::Phase::clear,
                                      blofeld::test::boot_context::capture(machine),
                                      context);
  while (cpu.registers().r[4] != known_long_ram_end - 1) { runner.step(); }
  const auto last_pair = machine.checkpoint();
  runner.step();
  require(cpu.registers().pc == clear_pc && cpu.registers().lc == clear_count - known_long_ram_end,
          "Bounded native REP yields at the first reserved Y address without skipping iterations");
  require(bus.read({.space = blofeld::DspSpace::x, .address = bcr}) == bcr_reset,
          "The unchanged original stage still retains the unqualified reset BIU control");
  blofeld::test::boot_context::record(blofeld::test::boot_context::Phase::frontier,
                                      blofeld::test::boot_context::capture(machine),
                                      context);
  for (const auto space : {blofeld::DspSpace::x, blofeld::DspSpace::y}) {
    for (unsigned address = 0; address < known_long_ram_end; ++address) {
      require(bus.read({.space = space, .address = address}) == 0,
              "Original REP and long-memory stores clear every mapped low X/Y word pair");
    }
  }
  if (execution == Execution::instruction) {
    blofeld::test::boot_context::check_switch(machine, context);
    std::ofstream access_trace(inputs.trace.string() + ".reference-access.trace");
    access_trace.exceptions(std::ios::failbit | std::ios::badbit);
    blofeld::test::verify_original_reference_access(last_pair, machine, access_trace);
  }
  std::cout << (execution == Execution::instruction ? "Instruction" : "Access")
            << " boundaries: documented NXP SPI bootstrap loaded all original 94 DSP words; "
            << runner.completed() << " completed reference steps. Original first stage programmed "
            << "PCTL and cleared 8192 X/Y word pairs; reserved-address write behavior remains "
            << "unverified. "
            << "No complete DSP/board boot is proven.\n";
  return blofeld::test::boot_context::capture(machine);
}

void compare_traces(const Inputs &legacy, const Inputs &reference) {
  std::ifstream first(legacy.trace, std::ios::binary);
  std::ifstream second(reference.trace, std::ios::binary);
  require(first.is_open() && second.is_open(), "Cannot compare both original execution traces");
  first.exceptions(std::ios::badbit);
  second.exceptions(std::ios::badbit);
  require(std::equal(std::istreambuf_iterator<char>(first),
                     std::istreambuf_iterator<char>(),
                     std::istreambuf_iterator<char>(second),
                     std::istreambuf_iterator<char>()),
          "Every original pre-instruction trace byte matches across both execution profiles");
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) {
      throw blofeld::Fault("Usage: dsp-boot-test firmware.zip private-rom.bin private.trace");
    }
    const Inputs legacy{.firmware = arguments[1], .rom = arguments[2], .trace = arguments[3]};
    auto reference = legacy;
    reference.trace += ".continuation.trace";
    const auto legacy_state = run(legacy, Execution::instruction);
    const auto reference_state = run(reference, Execution::access);
    require(legacy_state == reference_state,
            "Both complete original prefixes retain identical native "
            "progress/registers/stack/reset time");
    compare_traces(legacy, reference);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
