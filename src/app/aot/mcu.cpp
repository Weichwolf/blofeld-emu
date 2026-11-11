// SPDX-License-Identifier: GPL-3.0-only
#include "aot/mcu.hpp"

#include "aot/llvm.hpp"
#include "mcu/channels.hpp"
#include "mcu_bindings.hpp"

#include "llvm-c/Core.h"
#include "llvm-c/Types.h"

#include <array>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>

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
}
