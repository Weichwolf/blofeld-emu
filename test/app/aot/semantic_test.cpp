// SPDX-License-Identifier: GPL-3.0-only
#include "aot/semantic.hpp"
#include "cpu12/core.hpp"
#include "cpu12/memory.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "files/read.hpp"
#include CPU12_TEST_ISA
#include DSP56300_TEST_ISA

#include "llvm-c/Core.h"
#include "llvm-c/Error.h"
#include "llvm-c/TargetMachine.h"
#include "llvm-c/Transforms/PassBuilder.h"
#include "llvm-c/Types.h"
#include "llvm/ADT/ArrayRef.h"
#include "llvm/Support/BLAKE3.h"
#include "llvm/Support/SHA256.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <dlfcn.h>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
unsigned builders = 0, passes = 0;
constexpr std::size_t digest_bytes = 32, record_header = 2 * digest_bytes;
namespace aot = blofeld::app::aot::detail;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <class Function> Function original(const char *name) {
  const auto *address = dlsym(RTLD_NEXT, name);
  require(address != nullptr, "Resolve the packaged LLVM interface");
  return std::bit_cast<Function>(address);
}

struct CpuInput {
  using Core = cpu12::Core;
  static constexpr cpu12::Address start = 0x200;
  std::array<cpu12::Byte, 4> words{static_cast<cpu12::Byte>(cpu12::isa::Opcode::ldaa_immediate),
                                   0x5a,
                                   static_cast<cpu12::Byte>(cpu12::isa::Opcode::bra_relative),
                                   0xfe};
  std::array<cpu12::CodeSpan, 1> spans{cpu12::CodeSpan{.address = start, .words = words}};
  std::array<cpu12::Address, 1> entries{start};
  std::array<cpu12::CodeVariant, 1> variants{
      cpu12::CodeVariant{.id = 7, .code = spans, .entries = entries}};
  std::array<cpu12::RegionLayout, 2> regions{
      cpu12::RegionLayout{.first = start, .size = words.size(), .writable = false},
      cpu12::RegionLayout{.first = 0x1000, .size = 8}};
  std::array<cpu12::VariantLayout, 1> layouts{cpu12::VariantLayout{.id = 7, .memory = regions}};

  [[nodiscard]] cpu12::CodeImage image() const { return {.variants = variants}; }

  [[nodiscard]] cpu12::CompileOptions options() const {
    return {
        .memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}};
  }

  void change_code() { words[1] ^= cpu12::Byte{1}; }
};

struct DspInput {
  using Core = dsp56300::Core;
  static constexpr dsp56300::Word start = 16;
  std::array<dsp56300::Word, 2> words{
      static_cast<dsp56300::Word>(dsp56300::isa::Opcode::move_immediate_r0) | (0x5aU << 8),
      static_cast<dsp56300::Word>(dsp56300::isa::Opcode::stop)};
  std::array<dsp56300::CodeSpan, 1> spans{dsp56300::CodeSpan{.address = start, .words = words}};
  std::array<dsp56300::Word, 1> entries{start};
  std::array<dsp56300::CodeVariant, 1> variants{
      dsp56300::CodeVariant{.id = 7, .code = spans, .entries = entries, .failed_fetches = {}}};
  std::array<dsp56300::RegionLayout, 2> regions{
      dsp56300::RegionLayout{
          .space = dsp56300::Space::program, .first = 0, .size = 64, .writable = false},
      dsp56300::RegionLayout{.space = dsp56300::Space::x, .first = 0x1000, .size = 8}};
  std::array<dsp56300::VariantLayout, 1> layouts{
      dsp56300::VariantLayout{.id = 7, .memory = regions}};

  [[nodiscard]] dsp56300::CodeImage image() const { return {.variants = variants}; }

  [[nodiscard]] dsp56300::CompileOptions options() const {
    return {
        .memory = layouts, .interrupts = {}, .transitions = {}, .callbacks = {}, .bindings = {}};
  }

  void change_code() { words[0] ^= dsp56300::Word{1U << 8}; }
};

std::vector<std::filesystem::path> records(const std::filesystem::path &root) {
  std::vector<std::filesystem::path> result;
  for (const auto &file : std::filesystem::directory_iterator(root)) {
    if (file.path().extension() == ".artifact") { result.push_back(file.path()); }
  }
  return result;
}

void write(const std::filesystem::path &path, const std::span<const std::uint8_t> bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  output.write(reinterpret_cast<const char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  require(static_cast<bool>(output), "Write an intentionally invalid cache record");
}

void stale(std::vector<std::uint8_t> &bytes, const std::string_view compiler) {
  const auto encoded =
      std::span(reinterpret_cast<const std::uint8_t *>(compiler.data()), compiler.size());
  const auto match = std::ranges::search(bytes, encoded);
  require(!match.empty(), "The record owns the producer compiler identity");
  *match.begin() ^= std::uint8_t{1};
  auto semantic = std::span(bytes).subspan(record_header);
  const auto inner = llvm::SHA256::hash(
      llvm::ArrayRef(semantic.data() + digest_bytes, semantic.size() - digest_bytes));
  std::ranges::copy(inner, semantic.begin());
  const auto outer = llvm::BLAKE3::hash(llvm::ArrayRef(semantic.data(), semantic.size()));
  std::ranges::copy(outer, bytes.begin() + digest_bytes);
}

template <class Input> void region_modes(const std::filesystem::path &root) {
  Input input;
  auto options = input.options();
  if constexpr (requires { options.region_modes; }) {
    using Core = typename Input::Core;
    const std::array selected{dsp56300::Word{0}};
    const auto all = aot::prepare_semantic<Core>(input.image(), options, root);
    options.region_modes = selected;
    const auto hinted = aot::prepare_semantic<Core>(input.image(), options, root);
    require(all.identity() != hinted.identity() && records(root).size() == 2,
            "Distinct fast-mode hints retain separate semantic cache records");
    const auto before = builders + passes;
    const auto warm = aot::prepare_semantic<Core>(input.image(), options, root);
    require(warm.identity() == hinted.identity() && before == builders + passes &&
                records(root).size() == 2,
            "A matching fast-mode hint restores without lowering or overwriting another mode");
  }
}

void dsp_memory_modes(const std::filesystem::path &root) {
  const DspInput input;
  auto options = input.options();
  const auto prepare = [&] {
    return aot::prepare_semantic<DspInput::Core>(input.image(), options, root);
  };
  const auto exact = prepare();
  const auto exact_files = records(root);
  require(exact_files.size() == 1, "Exact memory publishes one semantic artifact");
  const auto &exact_file = exact_files.front();
  const auto exact_modified = std::filesystem::last_write_time(exact_file);
  options.memory_mode = dsp56300::CompileOptions::MemoryMode::direct;
  const auto direct = prepare();
  const auto both_files = records(root);
  require(exact.identity() != direct.identity() && both_files.size() == 2 &&
              std::ranges::find(both_files, exact_file) != both_files.end() &&
              std::filesystem::last_write_time(exact_file) == exact_modified,
          "Direct memory creates a separate semantic artifact and leaves exact intact");
  const auto before = builders + passes;
  options.memory_mode = dsp56300::CompileOptions::MemoryMode::exact;
  const auto warm = prepare();
  require(warm.identity() == exact.identity() &&
              std::ranges::equal(warm.artifact(), exact.artifact()) &&
              std::filesystem::last_write_time(exact_file) == exact_modified &&
              records(root).size() == 2 && before == builders + passes,
          "Returning to exact reuses the original semantic artifact and retains direct");
}

template <class Input> void lifecycle(const std::filesystem::path &root) {
  using Core = typename Input::Core;
  Input input;
  const auto prepare = [&] {
    return aot::prepare_semantic<Core>(input.image(), input.options(), root);
  };
  const auto before = builders + passes;
  const auto cold = prepare();
  require(builders + passes > before && !cold.bitcode().empty(),
          "Cold preparation really lowers the authored ISA program");
  const auto files = records(root);
  require(files.size() == 1, "Cold preparation publishes one checked semantic artifact");
  constexpr std::size_t maximum_bytes = std::size_t{1} << 30U;
  const auto original_bytes = blofeld::host::read_file(files[0], maximum_bytes);
  const auto modified = std::filesystem::last_write_time(files[0]);
  const auto warm_builders = builders;
  const auto warm_passes = passes;
  const auto warm = prepare();
  require(builders == warm_builders && passes == warm_passes &&
              cold.identity() == warm.identity() &&
              std::ranges::equal(cold.bitcode(), warm.bitcode()) &&
              std::filesystem::last_write_time(files[0]) == modified &&
              blofeld::host::read_file(files[0], maximum_bytes) == original_bytes,
          "A warm hit restores identical owned IR without builders, passes or cache rewrites");
  for (unsigned damage = 0; damage < 4; ++damage) {
    auto bytes = original_bytes;
    if (damage == 0) { bytes.front() ^= std::uint8_t{1}; }
    if (damage == 1) { bytes.back() ^= std::uint8_t{1}; }
    if (damage == 2) { bytes.resize(bytes.size() / 2); }
    if (damage == 3) { stale(bytes, Core::compiler_identity()); }
    write(files[0], bytes);
    const auto previous = builders + passes;
    const auto rebuilt = prepare();
    require(builders + passes > previous && rebuilt.identity() == cold.identity() &&
                blofeld::host::read_file(files[0], maximum_bytes) == original_bytes,
            "Wrong key, corrupt, truncated and checksum-resigned stale compiler records rebuild "
            "exactly");
  }
  std::filesystem::resize_file(files[0], maximum_bytes + 1);
  const auto oversized_builders = builders + passes;
  static_cast<void>(prepare());
  require(builders + passes > oversized_builders &&
              blofeld::host::read_file(files[0], maximum_bytes) == original_bytes,
          "An oversized sparse record rebuilds without allocating its advertised size");
  input.change_code();
  const auto code_builders = builders + passes;
  const auto changed_code = prepare();
  require(builders + passes > code_builders && changed_code.identity() != cold.identity() &&
              records(root).size() == 2,
          "A changed code word gets its own preparation key and actual changed IR");
  ++input.regions.back().size;
  const auto layout_builders = builders + passes;
  static_cast<void>(prepare());
  require(builders + passes > layout_builders && records(root).size() == 3,
          "A changed valid layout cannot reuse the previous semantic record");
}
}

extern "C" LLVMBuilderRef LLVMCreateBuilderInContext(LLVMContextRef C) {
  ++builders;
  return original<decltype(&LLVMCreateBuilderInContext)>("LLVMCreateBuilderInContext")(C);
}

extern "C" LLVMErrorRef LLVMRunPasses(LLVMModuleRef M,
                                      const char *Passes,
                                      LLVMTargetMachineRef TM,
                                      LLVMPassBuilderOptionsRef Options) {
  ++passes;
  return original<decltype(&LLVMRunPasses)>("LLVMRunPasses")(M, Passes, TM, Options);
}

int main(const int argc, const char *const *const argv) try {
  require(argc == 2, "Provide the generated fixture directory");
  const auto root = std::filesystem::absolute(argv[1]);
  std::filesystem::remove_all(root);
  lifecycle<CpuInput>(root / "cpu12");
  lifecycle<DspInput>(root / "dsp56300");
  region_modes<DspInput>(root / "region-modes");
  dsp_memory_modes(root / "memory-modes");
  std::cout << "Verified actual cold/warm semantic reuse and deterministic rejection/rebuild for "
               "both cores.\n";
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
