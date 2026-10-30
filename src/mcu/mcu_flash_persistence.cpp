// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/mcu_flash.hpp"
#include "mcu/mcu_flash_config.hpp"
#include "mcu/mcu_flash_timing_state.hpp"
#include "mcu/mcu_memory.hpp"
#include "storage/flash_image.hpp"
#include "storage/virtual_device_state.hpp"

#include <memory>
#include <type_traits>
#include <utility>

namespace blofeld {
namespace {
bool same_image(const McuFlashImage &a, const McuFlashImage &b) {
  return a.bytes == b.bytes && a.known == b.known && a.fresh == b.fresh;
}
}

void McuFlash::require_storage() const {
  if (store_) { static_cast<void>(store_->image()); }
}

bool McuFlash::persistent() const {
  return static_cast<bool>(store_);
}

void McuFlash::attach_store(std::shared_ptr<VirtualDeviceState> store) {
  require_running();
  if (!store || store_ || active_ || buffer_state_ != BufferState::empty) {
    throw Fault("MCU flash store requires one inactive, unbound controller");
  }
  if (!same_image(store->image(), memory_.flash_image())) {
    throw Fault("MCU flash store image differs from the native controller's array/history");
  }
  auto writer = store->claim_writer();
  store_ = std::move(store);
  store_writer_ = std::move(writer);
}

void McuMemory::commit_flash(const McuMemory &source) noexcept {
  flash_ = source.flash_;
  flash_known_ = source.flash_known_;
  flash_fresh_ = source.flash_fresh_;
}

void McuFlash::commit_state(const McuFlash &source) noexcept {
  static_assert(std::is_nothrow_copy_assignable_v<McuFlashConfig>);
  static_assert(std::is_nothrow_copy_assignable_v<FlashTimingState>);
  config_ = source.config_;
  buffered_ = source.buffered_;
  active_ = source.active_;
  buffer_state_ = source.buffer_state_;
  control_ = source.control_;
  command_ = source.command_;
  errors_ = source.errors_;
  blank_ = source.blank_;
  key_words_ = source.key_words_;
  key_locked_ = source.key_locked_;
  clock_ = source.clock_;
  stopped_ = source.stopped_;
  command_known_ = source.command_known_;
  timing_ = source.timing_;
}

void McuFlash::transact(const Transition transition) {
  if (!store_) {
    (this->*transition)();
    return;
  }
  require_storage();
  auto memory = memory_;
  McuFlash next(memory, *this);
  (next.*transition)();
  const auto cut = next.power_off_image();
  if (!same_image(cut, store_->image())) { store_->save(cut, store_writer_); }
  // Publication and directory fsync precede any native flag/array acknowledgement.
  // Only flash cells/history commit; unrelated RAM and peripheral inputs are untouched.
  memory_.commit_flash(memory);
  commit_state(next);
}

void McuFlash::transfer() {
  transact(&McuFlash::transfer_state);
}

void McuFlash::complete() {
  transact(&McuFlash::complete_state);
}
}
