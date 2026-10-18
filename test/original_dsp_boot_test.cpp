// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/firmware.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <string>
#include <vector>

namespace {
constexpr unsigned bootstrap_base = 0xff0000;
constexpr std::size_t bootstrap_words = 58;
constexpr unsigned mcu_frame_offset = 0x7875;
constexpr std::size_t stage_words = 94;
constexpr unsigned stage_base = 0x6a;
constexpr unsigned instruction_limit = 20000;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned pll_pc = 0x6f;
constexpr unsigned clear_pc = 0x72;
constexpr unsigned known_long_ram_end = 0x2000;
constexpr unsigned clear_count = 0x20000;
constexpr unsigned word_bits = 24;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::uint32_t read_word(const std::span<const std::uint8_t, 3> bytes) {
  return (static_cast<std::uint32_t>(bytes[0]) << 16U) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) | bytes[2];
}

std::vector<std::uint32_t> read_document_rom(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  std::array<char, bootstrap_words * 3> bytes{};
  file.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  require(file.gcount() == static_cast<std::streamsize>(bytes.size()) &&
              file.get() == std::char_traits<char>::eof(),
          "Document-derived SPI bootstrap prefix must have the pinned length");
  std::vector<std::uint32_t> words;
  for (std::size_t offset = 0; offset < bytes.size(); offset += 3) {
    const std::array<std::uint8_t, 3> triplet{static_cast<std::uint8_t>(bytes[offset]),
                                              static_cast<std::uint8_t>(bytes[offset + 1]),
                                              static_cast<std::uint8_t>(bytes[offset + 2])};
    words.push_back(read_word(triplet));
  }
  return words;
}

class Runner {
public:
  Runner(blofeld::DspBus &bus, blofeld::DspCore &cpu, std::ostream &trace)
      : bus_(bus), cpu_(cpu), trace_(trace) {}

  void step() {
    require(completed_ < instruction_limit, "DSP bootstrap instruction budget exhausted");
    const auto before = cpu_.state();
    trace_ << completed_ << " pc=" << blofeld::hex(before.pc, 6)
           << " sr=" << blofeld::hex(before.sr, 6) << " omr=" << blofeld::hex(before.omr, 6)
           << " r0=" << blofeld::hex(before.r[0], 6) << " r1=" << blofeld::hex(before.r[1], 6)
           << " lc=" << blofeld::hex(before.lc, 6) << '\n';
    cpu_.step();
    ++completed_;
  }

  void send(const std::uint32_t value) {
    bus_.shi().observe_spi({.selected = true, .mosi = false});
    step();
    for (unsigned bit = 0; bit < word_bits; ++bit) {
      const bool level = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
      bus_.shi().observe_spi({.selected = true, .clock = true, .mosi = level});
      step();
      bus_.shi().observe_spi({.selected = true, .mosi = level});
      step();
    }
    bus_.shi().observe_spi({.mosi = false});
    step();
  }

  [[nodiscard]] unsigned completed() const { return completed_; }

private:
  blofeld::DspBus &bus_;
  blofeld::DspCore &cpu_;
  std::ostream &trace_;
  unsigned completed_ = 0;
};

struct Inputs {
  std::filesystem::path firmware;
  std::filesystem::path rom;
  std::filesystem::path trace;
};

int run(const Inputs &inputs) {
  const auto firmware = blofeld::Firmware::load(inputs.firmware);
  const auto rom = read_document_rom(inputs.rom);
  blofeld::DspBus bus;
  bus.install_rom({.start = bootstrap_base, .words = rom});
  bus.shi().observe_spi({.mosi = false});
  blofeld::DspCore cpu(bus);
  std::ofstream trace(inputs.trace);
  require(trace.is_open(), "Cannot create private DSP bootstrap trace");
  trace.exceptions(std::ios::failbit | std::ios::badbit);
  Runner runner(bus, cpu, trace);
  while ((bus.shi().read(hcsr) & 1U) == 0) { runner.step(); }
  require(bus.once_latch() == 0,
          "Documented bootstrap performs its own OnCE GDB initialization write");
  const auto frame =
      std::span<const std::uint8_t>(firmware.mcu).subspan(mcu_frame_offset, (stage_words + 2) * 3);
  require(read_word(frame.first<3>()) == stage_words &&
              read_word(frame.subspan<3, 3>()) == stage_base,
          "Unmodified MCU image supplies the independently identified first-stage frame");
  for (std::size_t offset = 0; offset < frame.size(); offset += 3) {
    runner.send(read_word(std::span<const std::uint8_t, 3>(frame.subspan(offset, 3))));
  }
  while (cpu.state().pc != stage_base) { runner.step(); }
  require(cpu.state().sp == 0 && cpu.state().r[0] == stage_base + stage_words &&
              cpu.state().r[1] == stage_base,
          "Native ROM DO loop, stack and jump complete without host PC patching");
  for (std::size_t index = 0; index < stage_words; ++index) {
    const auto expected =
        read_word(std::span<const std::uint8_t, 3>(frame.subspan((index + 2) * 3, 3)));
    require(bus.read({.address = static_cast<std::uint32_t>(stage_base + index)}) == expected,
            "Every original first-stage word was written by executed DSP MOVEP instructions");
  }
  while (cpu.state().pc != pll_pc) { runner.step(); }
  constexpr unsigned original_memory_mode = 0x600389;
  require(
      cpu.state().omr == original_memory_mode && cpu.state().r[4] == 0,
      "Unmodified first-stage instructions select their own memory mode and clear their pointer");
  runner.step();
  require(bus.pll().read() == bus.read({.address = pll_pc + 1}),
          "Original MOVEP writes its own unmodified PCTL operand");
  runner.step();
  require(cpu.state().pc == clear_pc && cpu.state().lc == clear_count,
          "Fractional short immediate X0 supplies the full 24-bit REP count, not integer two");
  while (cpu.state().r[4] != known_long_ram_end) { runner.step(); }
  require(cpu.state().pc == clear_pc && cpu.state().lc == clear_count - known_long_ram_end,
          "Bounded native REP yields at the first reserved Y address without skipping iterations");
  for (const auto space : {blofeld::DspSpace::x, blofeld::DspSpace::y}) {
    for (unsigned address = 0; address < known_long_ram_end; ++address) {
      require(bus.read({.space = space, .address = address}) == 0,
              "Original REP and long-memory stores clear every mapped low X/Y word pair");
    }
  }
  std::cout << "Documented NXP SPI bootstrap loaded all original 94 DSP words through pin events; "
            << runner.completed() << " completed reference steps. Original first stage programmed "
            << "PCTL and cleared 8192 X/Y word pairs; reserved-address write behavior remains "
            << "unverified. No complete DSP/board boot is proven.\n";
  return 0;
}
}

int main(const int count, char **arguments) {
  try {
    if (count != 4) {
      throw blofeld::Fault("Usage: dsp-boot-test firmware.zip private-rom.bin private.trace");
    }
    return run({.firmware = arguments[1], .rom = arguments[2], .trace = arguments[3]});
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
