// SPDX-License-Identifier: GPL-3.0-only
#include "aot/mcu.hpp"

#include "aot/code.hpp"
#include "aot/llvm.hpp"
#include "mcu/channels.hpp"
#include "mcu/program.hpp"
#include "mcu_bindings.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Types.h"

#include <array>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace blofeld::app::aot {
McuBinding::McuBinding() {
  using namespace vi::mcu;
  constexpr std::array names{"blofeld_mcu_shi_select",
                             "blofeld_mcu_peer_interrupt",
                             "blofeld_mcu_peer_data",
                             "blofeld_mcu_peer_read",
                             "blofeld_mcu_peer_hold",
                             "blofeld_mcu_peer_publish",
                             "blofeld_mcu_peer_strobe",
                             "blofeld_mcu_time_publish",
                             "blofeld_mcu_time_enter",
                             "blofeld_mcu_time_leave",
                             "blofeld_mcu_time_hold",
                             "blofeld_mcu_midi_limit",
                             "blofeld_mcu_midi_peek",
                             "blofeld_mcu_midi_pop",
                             "blofeld_mcu_midi_write",
                             "blofeld_mcu_midi_await",
                             "blofeld_mcu_reset_state",
                             "blofeld_mcu_reset_assert"};
  const std::array addresses{reinterpret_cast<const void *>(&blofeld_mcu_shi_select),
                             reinterpret_cast<const void *>(&blofeld_mcu_peer_interrupt),
                             reinterpret_cast<const void *>(&blofeld_mcu_peer_data),
                             reinterpret_cast<const void *>(&blofeld_mcu_peer_read),
                             reinterpret_cast<const void *>(&blofeld_mcu_peer_hold),
                             reinterpret_cast<const void *>(&blofeld_mcu_peer_publish),
                             reinterpret_cast<const void *>(&blofeld_mcu_peer_strobe),
                             reinterpret_cast<const void *>(&blofeld_mcu_time_publish),
                             reinterpret_cast<const void *>(&blofeld_mcu_time_enter),
                             reinterpret_cast<const void *>(&blofeld_mcu_time_leave),
                             reinterpret_cast<const void *>(&blofeld_mcu_time_hold),
                             reinterpret_cast<const void *>(&blofeld_mcu_midi_limit),
                             reinterpret_cast<const void *>(&blofeld_mcu_midi_peek),
                             reinterpret_cast<const void *>(&blofeld_mcu_midi_pop),
                             reinterpret_cast<const void *>(&blofeld_mcu_midi_write),
                             reinterpret_cast<const void *>(&blofeld_mcu_midi_await),
                             reinterpret_cast<const void *>(&blofeld_mcu_reset_state),
                             reinterpret_cast<const void *>(&blofeld_mcu_reset_assert)};
  namespace native = host::aot;
  const native::detail::Owner<LLVMContextRef, LLVMContextDispose> context(LLVMContextCreate());
  native::detail::Exports exports;
  const native::detail::Owner<LLVMModuleRef, LLVMDisposeModule> module(
      native::detail::parse(context.get(), {.bitcode = bitcode(), .exports = {}}, exports));
  for (std::size_t index = 0; index < names.size(); ++index) {
    auto *function = LLVMGetNamedFunction(module.get(), names[index]);
    if (function == nullptr || LLVMIsDeclaration(function) == 0) {
      throw std::invalid_argument(std::string("MCU binding lacks native channel: ") + names[index]);
    }
    signatures_[index] = native::detail::abi(function);
    imports_[index] = {
        .name = names[index], .abi = signatures_[index], .address = addresses[index]};
  }
}

std::span<const std::byte> McuBinding::bitcode() noexcept {
  return detail::mcu_bindings;
}

cpu12::NativeImage prepare_mcu(const cpu12::CodeImage image,
                               const std::span<const cpu12::CodeTransition> transitions) {
  using cpu12::CallbackBinding;
  using cpu12::CallbackRole;
  constexpr std::array callbacks{
      CallbackBinding{.role = CallbackRole::read, .symbol = "blofeld_mcu_read"},
      CallbackBinding{.role = CallbackRole::write, .symbol = "blofeld_mcu_write"},
      CallbackBinding{.role = CallbackRole::read_word, .symbol = "blofeld_mcu_read_word"},
      CallbackBinding{.role = CallbackRole::write_word, .symbol = "blofeld_mcu_write_word"},
      CallbackBinding{.role = CallbackRole::read_masked, .symbol = "blofeld_mcu_read_masked"}};
  constexpr cpu12::Address xirq_vector = 0xfff4;
  std::vector<cpu12::InterruptRequest> interrupts{
      {.vector = xirq_vector, .kind = cpu12::InterruptKind::xirq}};
  constexpr std::array<cpu12::Address, 22> vectors{
      0xffee, 0xffec, 0xffea, 0xffe8, 0xffe6, 0xffe4, 0xffe2, 0xffe0, 0xffde, 0xfff0, 0xffc6,
      0xffd6, 0xffd4, 0xffd2, 0xffd0, 0xffbe, 0xffce, 0xffcc, 0xffca, 0xffc8, 0xffc4, 0xffc2};
  interrupts.reserve(vectors.size() + 1);
  for (const auto vector : vectors) { interrupts.push_back({.vector = vector}); }
  const auto semantic = cpu12::Core::lower(image,
                                           {.memory = {},
                                            .interrupts = interrupts,
                                            .transitions = transitions,
                                            .callbacks = callbacks,
                                            .bindings = {},
                                            .topology = cpu12::MemoryTopology::dynamic});
  namespace native = host::aot;
  const McuBinding binding;
  std::vector<native::Export> exports;
  exports.reserve(semantic.exports().size());
  for (const auto symbol : semantic.exports()) {
    exports.push_back({.name = symbol.name, .abi = symbol.abi});
  }
  const std::array modules{native::Module{.bitcode = semantic.bitcode(), .exports = exports},
                           native::Module{.bitcode = McuBinding::bitcode(), .exports = {}}};
  const auto code = std::make_shared<native::Code>(modules, binding.imports());
  std::vector<cpu12::NativeSymbol> symbols;
  symbols.reserve(semantic.exports().size());
  for (const auto symbol : semantic.exports()) {
    symbols.push_back({.name = symbol.name, .address = code->address(symbol.name)});
  }
  return cpu12::Core::bind(semantic,
                           {.owner = code,
                            .symbols = symbols,
                            .identity = semantic.identity(),
                            .native_bytes = code->native_bytes()});
}

vi::mcu::Program prepare_mcu_program(const cpu12::CodeImage image,
                                     const std::span<const cpu12::CodeTransition> transitions) {
  return {prepare_mcu(image, transitions), image};
}

}
