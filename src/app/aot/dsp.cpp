// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"

#include "aot/code.hpp"
#include "aot/llvm.hpp"
#include "aot/semantic.hpp"
#include "dsp/channels.hpp"
#include "dsp/program.hpp"
#include "dsp_bindings.hpp"
#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif

#include "llvm-c/Core.h"
#include "llvm-c/Types.h"

#include <array>
#include <cstddef>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <filesystem>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace blofeld::app::aot {
namespace {
namespace native = host::aot;

class Channels {
public:
  explicit Channels(const std::span<const std::byte> bitcode) {
    using namespace vi::dsp;
    const std::array addresses{
        reinterpret_cast<const void *>(&blofeld_dsp_shi_read),
        reinterpret_cast<const void *>(&blofeld_dsp_shi_write),
        reinterpret_cast<const void *>(&blofeld_dsp_peer_sample),
        reinterpret_cast<const void *>(&blofeld_dsp_peer_data),
        reinterpret_cast<const void *>(&blofeld_dsp_peer_write),
        reinterpret_cast<const void *>(&blofeld_dsp_peer_ready),
        reinterpret_cast<const void *>(&blofeld_dsp_peer_interrupt),
#if defined(BLOFELD_CENSUS)
        reinterpret_cast<const void *>(&vi::Census::unit_entry),
        reinterpret_cast<const void *>(&vi::Census::unit_retired),
        reinterpret_cast<const void *>(&vi::Census::discarded_write),
        reinterpret_cast<const void *>(&vi::Census::precise_retired),
#endif
    };
    const native::detail::Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
    native::detail::Exports exports;
    const native::detail::Owner<LLVMModuleRef, LLVMDisposeModule> module(
        native::detail::parse(context.get(), {.bitcode = bitcode, .exports = {}}, exports));
    for (std::size_t index = 0; index < names.size(); ++index) {
      auto &name = names_[index];
      name = names[index];
      auto *function = LLVMGetNamedFunction(module.get(), name.c_str());
      if (function == nullptr) {
        // Literal assembler labels bypass the target's symbol prefix.
        name.insert(name.begin(), '\1');
        function = LLVMGetNamedFunction(module.get(), name.c_str());
      }
      if (function == nullptr || LLVMIsDeclaration(function) == 0) {
        throw std::invalid_argument(std::string("DSP binding lacks native channel: ") +
                                    names[index]);
      }
      signatures_[index] = native::detail::abi(function);
      entries_[index] = {.name = name, .abi = signatures_[index], .address = addresses[index]};
    }
  }

  [[nodiscard]] std::span<const native::Import> entries() const noexcept { return entries_; }

private:
  static constexpr std::array names{
      "blofeld_dsp_shi_read",
      "blofeld_dsp_shi_write",
      "blofeld_dsp_peer_sample",
      "blofeld_dsp_peer_data",
      "blofeld_dsp_peer_write",
      "blofeld_dsp_peer_ready",
      "blofeld_dsp_peer_interrupt",
#if defined(BLOFELD_CENSUS)
      "blofeld_census_unit_entry",
      "blofeld_census_unit_retired",
      "blofeld_census_discarded_write",
      "blofeld_census_precise_retired",
#endif
  };
  std::array<std::string, names.size()> names_, signatures_;
  std::array<native::Import, names.size()> entries_;
};
}

dsp56300::NativeImage prepare_dsp(const dsp56300::CodeImage image,
                                  const std::span<const dsp56300::VariantLayout> memory,
                                  const std::span<const dsp56300::InterruptRequest> interrupts,
                                  const std::span<const dsp56300::CodeTransition> transitions,
                                  const std::filesystem::path &cache,
                                  const dsp56300::CompileOptions::MemoryMode mode) {
  using dsp56300::CallbackBinding;
  using dsp56300::CallbackRole;
  constexpr std::array callbacks{
      CallbackBinding{.role = CallbackRole::read, .symbol = "blofeld_dsp_read"},
      CallbackBinding{.role = CallbackRole::write, .symbol = "blofeld_dsp_write"},
      CallbackBinding{.role = CallbackRole::bit, .symbol = "blofeld_dsp_bit"},
      CallbackBinding{.role = CallbackRole::reset, .symbol = "blofeld_dsp_reset"},
      CallbackBinding{.role = CallbackRole::pending, .symbol = "blofeld_dsp_pending"},
      CallbackBinding{.role = CallbackRole::revision, .symbol = "blofeld_dsp_revision"},
#if defined(BLOFELD_CENSUS)
      CallbackBinding{.role = CallbackRole::retired, .symbol = "blofeld_dsp_retired"},
#endif
  };
  constexpr std::array region_modes{dsp56300::Word{0}, dsp56300::Word{1} << 11};
  const auto layouts = mode == dsp56300::CompileOptions::MemoryMode::direct
                           ? std::span<const dsp56300::VariantLayout>{}
                           : memory;
  const dsp56300::CompileOptions options{.memory = layouts,
                                         .interrupts = interrupts,
                                         .transitions = transitions,
                                         .callbacks = callbacks,
                                         .bindings = {},
                                         .memory_mode = mode,
                                         .region_modes = region_modes};
  const auto semantic = detail::prepare_semantic<dsp56300::Core>(image, options, cache);
  std::vector<native::Export> exports;
  for (const auto symbol : semantic.exports()) {
    exports.push_back({.name = symbol.name, .abi = symbol.abi});
  }
  const std::array modules{native::Module{.bitcode = semantic.bitcode(), .exports = exports},
                           native::Module{.bitcode = detail::dsp_bindings, .exports = {}}};
  const Channels channels(detail::dsp_bindings);
#if defined(__APPLE__) && defined(__aarch64__)
  constexpr auto executable_memory = native::Code::ExecutableMemory::jit;
#else
  constexpr auto executable_memory = native::Code::ExecutableMemory::standard;
#endif
  const auto code =
      std::make_shared<native::Code>(modules, channels.entries(), cache, executable_memory);
  std::vector<dsp56300::NativeSymbol> symbols;
  for (const auto symbol : semantic.exports()) {
    symbols.push_back({.name = symbol.name, .address = code->address(symbol.name)});
  }
  const auto source = code->fault_map();
  std::vector<dsp56300::NativeSymbols::TextRange> text;
  std::vector<dsp56300::NativeSymbols::Site> sites;
  text.reserve(source.text_ranges.size());
  sites.reserve(source.sites.size());
  for (const auto &range : source.text_ranges) {
    text.push_back({.address = range.address, .size = range.size});
  }
  for (const auto &site : source.sites) {
    sites.push_back({.address = site.address,
                     .size = site.size,
                     .variant = site.variant,
                     .guest_pc = site.guest_pc});
  }
  return dsp56300::Core::bind(semantic,
                              {.owner = code,
                               .symbols = symbols,
                               .identity = semantic.identity(),
                               .native_bytes = code->native_bytes(),
                               .fault_map = {.text_ranges = text, .sites = sites}});
}

vi::dsp::Program prepare_dsp_program(const dsp56300::CodeImage image,
                                     const std::span<const vi::dsp::ProgramChoice> choices,
                                     const std::span<const dsp56300::VariantLayout> memory,
                                     const std::span<const dsp56300::InterruptRequest> interrupts,
                                     const std::span<const dsp56300::CodeTransition> transitions,
                                     const std::filesystem::path &cache,
                                     const dsp56300::CompileOptions::MemoryMode mode) {
  return {prepare_dsp(image, memory, interrupts, transitions, cache, mode),
          image,
          choices,
          transitions,
          mode == dsp56300::CompileOptions::MemoryMode::direct
              ? std::span<const dsp56300::VariantLayout>{}
              : memory};
}

}
