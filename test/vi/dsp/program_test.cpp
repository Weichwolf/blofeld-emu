// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/program.hpp"
#include "isa.hpp"

#include <array>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>

namespace {
using namespace dsp56300;
using blofeld::vi::dsp::Program;
using blofeld::vi::dsp::ProgramChoice;
using blofeld::vi::dsp::ProgramWord;
constexpr Word entry = 16, marker = 32, switched = 0x80;
constexpr std::array words{static_cast<Word>(isa::Opcode::stop)};
constexpr std::array entries{entry};
const std::array code{CodeSpan{.address = entry, .words = words}};
const std::array variants{
    CodeVariant{.id = 0, .code = code, .entries = entries, .failed_fetches = {}},
    CodeVariant{.id = 1, .code = code, .entries = entries, .failed_fetches = {}}};
constexpr std::array known{ProgramWord{.address = marker, .value = Word{0}}};
constexpr std::array unknown{ProgramWord{.address = marker, .value = std::nullopt}};
const std::array choices{
    ProgramChoice{.variant = 0, .resumes = entries, .words = known, .mode_mask = switched},
    ProgramChoice{.variant = 1,
                  .resumes = entries,
                  .words = unknown,
                  .mode_mask = switched,
                  .mode = switched}};

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

struct Bus final : Io {
  [[nodiscard]] std::optional<Word> inspect_program(const Word at) const noexcept override {
    if (at == entry) { return words[0]; }
    return at == marker ? value : std::nullopt;
  }

  bool read(Space /*space*/, Word /*at*/, Word /*pc*/, Word & /*value*/) noexcept override {
    return false;
  }

  bool write(Space /*space*/, Word /*at*/, Word /*pc*/, Word /*value*/) noexcept override {
    return false;
  }

  bool bit(Space /*space*/,
           Word /*at*/,
           Word /*pc*/,
           Word /*bit*/,
           BitAction /*action*/,
           bool & /*previous*/) noexcept override {
    return false;
  }

  std::span<const MemoryWindow> memory_windows(const Space space) noexcept override {
    return expose && space == Space::x ? std::span(&window, 1) : std::span<const MemoryWindow>{};
  }

  std::array<Word, 1> data{};
  std::array<std::uint8_t, 1> known_bytes{1};
  MemoryWindow window{.address = marker, .words = data, .known = known_bytes, .writable = true};
  std::optional<Word> value;
  bool expose = false;
};

void selected(const NativeImage &image) {
  const Program program(image, {.variants = variants}, choices, {});
  require(program.contains({.variant = 0, .address = entry}) &&
              !program.contains({.variant = 0, .address = entry + 1}) &&
              !program.contains({.variant = 2, .address = entry}) &&
              !program.transition({.variant = 0, .address = entry}),
          "Only declared code words and actual transfer contracts are indexed");
  for (const Word mode : {Word{0}, switched}) {
    Bus bus;
    if (mode == 0) { bus.value = 0; }
    Core core(Memory::external(), &bus);
    auto seed = core.registers();
    seed.pc = entry;
    seed.omr = mode;
    core.set_registers(seed);
    require(program.install(core, bus) == (mode == 0 ? 0U : 1U) && core.registers() == seed &&
                core.progress() == Progress{},
            "Known zero differs from an unknown word; installation does not execute ISA");
    const auto result = core.run(1);
    require(result.fault == Fault{} && result.retired == 1 &&
                result.state == ExecutionState::stopped,
            "Selected code executes through the public native image contract");
  }
}

void mapped() {
  const std::array regions{
      RegionLayout{.space = Space::x, .first = marker, .size = 1, .tracked = true}};
  const std::array layouts{VariantLayout{.id = 0, .memory = {}},
                           VariantLayout{.id = 1, .memory = regions}};
  const auto image = Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
  const std::array by_map{ProgramChoice{.variant = 0, .resumes = entries, .words = {}},
                          ProgramChoice{.variant = 1, .resumes = entries, .words = {}}};
  const Program program(image, {.variants = variants}, by_map, {}, layouts);
  for (const bool exposed : {false, true}) {
    Bus bus;
    bus.expose = exposed;
    Core core(Memory::external(), &bus);
    auto seed = core.registers();
    seed.pc = entry;
    core.set_registers(seed);
    require(
        program.install(core, bus) == (exposed ? 1U : 0U) && core.registers() == seed,
        "Equal code and PCs are distinguished by actual memory grants rather than catalogue order");
    const auto result = core.run(1);
    require(result.retired == 1 && result.fault == Fault{},
            "Both admitted memory shapes execute their native code");
  }
}

void mixed_layouts() {
  const std::array regions{
      RegionLayout{.space = Space::x, .first = marker, .size = 1, .tracked = true}};
  const std::array layouts{VariantLayout{.id = 1, .memory = regions}};
  const auto image = Core::compile(
      {.variants = variants},
      {.memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}});
  const Program program(image, {.variants = variants}, choices, {}, layouts);
  for (const Word mode : {Word{0}, switched}) {
    Bus bus;
    bus.expose = true;
    if (mode == 0) { bus.value = 0; }
    Core core(Memory::external(), &bus);
    auto seed = core.registers();
    seed.pc = entry;
    seed.omr = mode;
    core.set_registers(seed);
    require(program.install(core, bus) == (mode == 0 ? 0U : 1U) && core.registers() == seed,
            "Generic and fixed variants share one image with their own admission contracts");
    require(core.run().state == ExecutionState::stopped,
            "Both memory contracts run their original native STOP");
  }
}

void rejected(const NativeImage &image) {
  const Program program(image, {.variants = variants}, choices, {});
  Bus bus;
  bus.value = 1;
  Core core(Memory::external(), &bus);
  auto seed = core.registers();
  seed.pc = entry;
  core.set_registers(seed);
  bool failed = false;
  try {
    static_cast<void>(program.install(core, bus));
  } catch (const std::runtime_error &error) {
    failed = std::string_view(error.what()).contains("P:$000010");
  }
  require(failed && core.registers() == seed && core.progress() == Progress{},
          "Unprepared entry/data combinations fail with PC context before execution");
  const std::array ambiguous{ProgramChoice{.variant = 0, .resumes = entries, .words = {}},
                             ProgramChoice{.variant = 1, .resumes = entries, .words = {}}};
  const Program conflict(image, {.variants = variants}, ambiguous, {});
  failed = false;
  try {
    static_cast<void>(conflict.install(core, bus));
  } catch (const std::runtime_error &error) {
    failed = std::string_view(error.what()).contains("Ambiguous");
  }
  require(failed && core.registers() == seed && core.progress() == Progress{},
          "Ambiguous variants never silently win by catalogue order");
  failed = false;
  try {
    const Program missing(image, {.variants = variants}, std::span(choices).first(1), {});
  } catch (const std::invalid_argument &) { failed = true; }
  require(failed, "Every source variant needs an explicit admission contract");
  constexpr std::array invalid_transfer{CodeTransition{.from = 0, .to = 1, .entry = entry + 1}};
  failed = false;
  try {
    const Program missing(image, {.variants = variants}, choices, invalid_transfer);
  } catch (const std::invalid_argument &) { failed = true; }
  require(failed, "Every declared transfer needs a prepared target installation point");
}
}

int main() try {
  const auto image = Core::compile({.variants = variants}, {});
  selected(image);
  mapped();
  mixed_layouts();
  rejected(image);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
