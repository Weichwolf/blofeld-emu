// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"

#include "aot/code.hpp"
#include "aot/llvm.hpp"
#include "dsp/channels.hpp"
#include "dsp_bindings.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Types.h"

#include <array>
#include <cstddef>
#include <dsp56300/core.hpp>
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
    constexpr std::array names{"blofeld_dsp_shi_read",
                               "blofeld_dsp_shi_write",
                               "blofeld_dsp_peer_sample",
                               "blofeld_dsp_peer_data",
                               "blofeld_dsp_peer_write",
                               "blofeld_dsp_peer_ready",
                               "blofeld_dsp_peer_interrupt"};
    const std::array addresses{reinterpret_cast<const void *>(&blofeld_dsp_shi_read),
                               reinterpret_cast<const void *>(&blofeld_dsp_shi_write),
                               reinterpret_cast<const void *>(&blofeld_dsp_peer_sample),
                               reinterpret_cast<const void *>(&blofeld_dsp_peer_data),
                               reinterpret_cast<const void *>(&blofeld_dsp_peer_write),
                               reinterpret_cast<const void *>(&blofeld_dsp_peer_ready),
                               reinterpret_cast<const void *>(&blofeld_dsp_peer_interrupt)};
    const native::detail::Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
    native::detail::Exports exports;
    const native::detail::Owner<LLVMModuleRef, LLVMDisposeModule> module(
        native::detail::parse(context.get(), {.bitcode = bitcode, .exports = {}}, exports));
    for (std::size_t index = 0; index < names.size(); ++index) {
      auto *function = LLVMGetNamedFunction(module.get(), names[index]);
      if (function == nullptr || LLVMIsDeclaration(function) == 0) {
        throw std::invalid_argument(std::string("DSP binding lacks native channel: ") +
                                    names[index]);
      }
      signatures_[index] = native::detail::abi(function);
      entries_[index] = {
          .name = names[index], .abi = signatures_[index], .address = addresses[index]};
    }
  }

  [[nodiscard]] std::span<const native::Import> entries() const noexcept { return entries_; }

private:
  std::array<std::string, 7> signatures_;
  std::array<native::Import, 7> entries_;
};
}

dsp56300::NativeImage prepare_dsp(const dsp56300::CodeImage image,
                                  const std::span<const dsp56300::VariantLayout> memory,
                                  const std::span<const dsp56300::InterruptRequest> interrupts,
                                  const std::span<const dsp56300::CodeTransition> transitions) {
  using dsp56300::CallbackBinding;
  using dsp56300::CallbackRole;
  constexpr std::array callbacks{
      CallbackBinding{.role = CallbackRole::read, .symbol = "blofeld_dsp_read"},
      CallbackBinding{.role = CallbackRole::write, .symbol = "blofeld_dsp_write"},
      CallbackBinding{.role = CallbackRole::bit, .symbol = "blofeld_dsp_bit"},
      CallbackBinding{.role = CallbackRole::reset, .symbol = "blofeld_dsp_reset"},
      CallbackBinding{.role = CallbackRole::pending, .symbol = "blofeld_dsp_pending"},
      CallbackBinding{.role = CallbackRole::revision, .symbol = "blofeld_dsp_revision"}};
  const auto semantic = dsp56300::Core::lower(image,
                                              {.memory = memory,
                                               .interrupts = interrupts,
                                               .transitions = transitions,
                                               .callbacks = callbacks,
                                               .bindings = {}});
  std::vector<native::Export> exports;
  for (const auto symbol : semantic.exports()) {
    exports.push_back({.name = symbol.name, .abi = symbol.abi});
  }
  const std::array modules{native::Module{.bitcode = semantic.bitcode(), .exports = exports},
                           native::Module{.bitcode = detail::dsp_bindings, .exports = {}}};
  const Channels channels(detail::dsp_bindings);
  const auto code = std::make_shared<native::Code>(modules, channels.entries());
  std::vector<dsp56300::NativeSymbol> symbols;
  for (const auto symbol : semantic.exports()) {
    symbols.push_back({.name = symbol.name, .address = code->address(symbol.name)});
  }
  return dsp56300::Core::bind(semantic,
                              {.owner = code,
                               .symbols = symbols,
                               .identity = semantic.identity(),
                               .native_bytes = code->native_bytes()});
}
}
