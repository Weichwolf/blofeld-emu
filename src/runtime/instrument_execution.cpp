// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "mcu/hc12.hpp"
#include "mcu/mcu_machine.hpp"
#include "mcu/mcu_memory.hpp"
#include "mcu/mcu_reset.hpp"
#include "runtime/instrument.hpp"
#include "runtime/mcu_dsp_runtime.hpp"
#include "runtime/nand_peer.hpp"
#include "usb/usbip_runtime.hpp"
#include "usb/usbip_tcp.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>

namespace blofeld::runtime {
class Instrument::Busy {
public:
  explicit Busy(Instrument &board) : board_(board) {
    board.require_idle();
    board_.active_ = true;
  }

  ~Busy() { board_.active_ = false; }

  Busy(const Busy &) = delete;
  Busy &operator=(const Busy &) = delete;

private:
  Instrument &board_;
};

void Instrument::require_idle() const {
  if (active_) { throw Fault("Instrument control requires an inactive operation boundary"); }
}

void Instrument::require_healthy() const {
  require_idle();
  if (timing_faulted()) { throw Fault("Instrument failure requires a fresh device instance"); }
}

void Instrument::initialize() {
  require_healthy();
  if (control_.started) { return; }
  const Busy guard(*this);
  try {
    check_pcm_output();
    boundary();
    control_.started = true;
  } catch (...) {
    mark_fault();
    throw;
  }
}

void Instrument::advance() {
  require_healthy();
  initialize();
  const Busy guard(*this);
  try {
    check_pcm_output();
    execute_until_event();
    if (execution_.trace != nullptr) {
      *execution_.trace << "MCU end=" << ticks();
      const auto reset = native_->mcu.system_reset_state();
      if (reset && reset->phase != McuResetPhase::complete) {
        *execution_.trace << " held-reset=1 reset-edges=" << reset->edges;
      } else {
        *execution_.trace << " pc=" << hex(native_->mcu.cpu().state().pc);
      }
      *execution_.trace << " instructions=" << control_.instructions
                        << " idle=" << native_->mcu.idle_bus_cycles() << '\n';
    }
    boundary();
  } catch (...) {
    mark_fault();
    throw;
  }
}

void Instrument::poll_usb(UsbipRuntime &runtime) {
  require_healthy();
  if (!control_.started) { throw Fault("USB/IP input requires an initialized instrument"); }
  if (native_->mcu.system_reset_active()) { return; }
  const Busy guard(*this);
  const auto cpu = native_->mcu.cpu().state();
  const auto before_started = runtime.started();
  const auto before_published = runtime.published();
  try {
    const auto events = runtime.poll(native_->mcu.bus(),
                                     {.pc = cpu.pc, .ccr = cpu.ccr},
                                     {.owner = identity_, .restore_revision = 0, .ticks = ticks()});
    if (execution_.trace != nullptr) {
      if (!before_started && runtime.started()) {
        *execution_.trace << "USBIP-START actual=" << ticks() << '\n';
      }
      if (!before_published && runtime.published()) {
        *execution_.trace << "USBIP-PUBLISH actual=" << ticks() << '\n';
      }
      for (const auto &event : events) {
        *execution_.trace << "USBIP-PEER actual=" << ticks() << " connection=" << event.connection
                          << " error=" << event.error << " reason=" << event.reason << '\n';
      }
    }
    if (!before_published && runtime.published()) {
      std::cout << "USBIP descriptors=ready tick=" << ticks() << '\n' << std::flush;
    }
    for (const auto &event : events) {
      if (event.error) { std::cerr << "USB/IP peer: " << event.reason << '\n'; }
    }
  } catch (...) {
    mark_fault();
    runtime.retire();
    throw;
  }
}

Status Instrument::status() const {
  require_idle();
  const bool known = !native_->mcu.system_reset_active();
  return {.mcu = known ? native_->mcu.cpu().state() : CpuState{},
          .dsp = native_->dsp.cpu().registers(),
          .mcu_state_known = known,
          .common_ticks = ticks(),
          .mcu_cycles = native_->mcu.run_clock().bus_cycles(),
          .dsp_cycles = native_->runtime->completed_cycles(),
          .dsp_steps = dsp_steps(),
          .instructions = control_.instructions,
          .interrupts = control_.interrupts,
          .waiting = known && native_->mcu.cpu().waiting(),
          .held_reset = dsp_reset_active(),
          .faulted = timing_faulted()};
}

McuMemoryByte Instrument::inspect_mcu(const std::uint16_t address) const {
  require_idle();
  return native_->mcu.bus().inspect_memory(address);
}

std::optional<std::uint32_t> Instrument::inspect_dsp(const DspAddress address) const {
  require_idle();
  return native_->dsp.bus().inspect_memory(address);
}

}
