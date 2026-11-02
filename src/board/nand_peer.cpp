// SPDX-License-Identifier: GPL-3.0-only
#include "board/nand_peer.hpp"

#include "board/gpio_pads.hpp"
#include "board/nand_bus_decode.hpp"
#include "board/nand_peer_medium.hpp"
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_interval.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_medium.hpp"
#include "storage/nand_write_store.hpp"
#include "storage/nand_write_timing.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace blofeld {

namespace {
constexpr unsigned control_mask = 0xf00;
constexpr unsigned re_pin = 0x100;
constexpr unsigned we_pin = 0x200;
constexpr unsigned cle_pin = 0x400;
constexpr unsigned ale_pin = 0x800;
constexpr unsigned data_mask = 0xff;
constexpr unsigned port_f_mask = 0x7ff;
constexpr unsigned ready_timer = 1;

std::uint64_t power_deadline(const std::uint64_t timebase) {
  constexpr unsigned microseconds = 100;
  constexpr unsigned per_second = 1000000;
  const auto ticks = nand_interval_ticks<per_second>(timebase, microseconds);
  if (!ticks || *ticks == 0) { throw Fault("Invalid NAND peer power-on timebase"); }
  return *ticks;
}

struct PeerState {
  PeerState(NandPeerMedium clock, const std::uint64_t timebase, const NandBusContract contract)
      : device(std::move(clock)),
        decoder(contract),
        write(timebase),
        setup(timebase),
        output(timebase),
        timebase_hz(timebase),
        power_ready_tick(power_deadline(timebase)) {}

  bool power_ready = false;
  bool faulted = false;
  NandPeerMedium device;
  NandBusDecode decoder;
  NandWriteTiming write;
  NandReadSetup setup;
  NandReadTiming output;
  DspGpioDrive drive;
  std::uint64_t timebase_hz;
  std::uint64_t power_ready_tick;
  DspRunContext context;
  std::optional<unsigned> previous_known;
  std::optional<bool> previous_re;
  bool connected = false;
  bool trace_access = false;
};
}

struct NandPeer::Impl {
  Impl(OwnDspMachine &machine, NandPeerMedium medium, const NandBusContract contract)
      : machine_(machine),
        state_(std::make_unique<PeerState>(
            std::move(medium), machine.clock().timebase_hz(), contract)) {
    std::cerr << "NAND_BOARD_MODEL CE=held-selected power=100us-reference "
                 "part=explicit-device-contract GPIO=logical sample=period-completion "
                 "board-qualified=0 contract="
              << static_cast<unsigned>(contract) << '\n';
    drive(0);
  }

  Impl(OwnDspMachine &target,
       const PeerState &state,
       std::shared_ptr<const PeripheralEventIdentity> identity)
      : machine_(target),
        state_(std::make_unique<PeerState>(state)),
        identity_(std::move(identity)) {}

  [[nodiscard]] NandPeerDetailed detailed() const {
    healthy();
    auto output = state_->output;
    return {.base = status(),
            .page = state_->device.device().page(),
            .column = state_->device.device().column(),
            .output = output.output(state_->device.ticks()),
            .power_ready = state_->power_ready,
            .known_controls = state_->previous_known,
            .previous_re = state_->previous_re,
            .connected = state_->connected,
            .last_pc = state_->context.pc,
            .last_omr = state_->context.omr,
            .trace_access = state_->trace_access,
            .bus_contract = state_->decoder.contract(),
            .neutral_bus = state_->decoder.neutral()};
  }

  void bind(const std::weak_ptr<const bool> &owner) {
    if (!needs_binding_) { return; }
    machine_.bus().gpio(DspPort::f).attach_pad_source([this, owner](const std::uint32_t mask) {
      if (owner.expired()) { throw Fault("NAND pad read lost its live board owner"); }
      return pads(mask);
    });
    needs_binding_ = false;
  }

  [[nodiscard]] GpioPadSample pads(const std::uint32_t mask) const {
    healthy();
    if (needs_binding_) { fail("pad read requires its restored native binding"); }
    const auto output = machine_.bus().gpio(DspPort::f).outputs();
    const auto drive = state_->drive;
    return resolve_gpio_net(mask,
                            {.active = (output.mask | output.peripheral) & mask,
                             .known = output.known & mask,
                             .levels = output.levels & mask},
                            {.active = (drive.mask | drive.unknown) & mask,
                             .known = drive.mask & mask,
                             .levels = drive.levels & mask,
                             .pull_down = drive.floating & mask});
  }

  void before(const std::uint64_t ticks, const DspRunContext context) {
    healthy();
    state_->context = context;
    // External events precede the period's completion. Its nominal start may
    // therefore precede a real event already published by the scheduler.
    if (ticks >= state_->device.ticks()) { advance(ticks); }
    drive(state_->device.ticks());
  }

  [[nodiscard]] std::optional<std::uint64_t> next_event() const {
    healthy();
    const auto operation = state_->device.next_event();
    std::optional<std::uint64_t> next;
    if (!state_->power_ready) { next = state_->power_ready_tick; }
    if (operation && (!next || operation->ticks < *next)) { next = operation->ticks; }
    const auto output = state_->output.next_transition();
    if (output && (!next || *output < *next)) { next = output; }
    return next;
  }

  void dispatch(const std::uint64_t ticks) {
    if (next_event() != ticks) { fail("event requires its exact offered deadline"); }
    const bool power = !state_->power_ready && ticks == state_->power_ready_tick;
    const bool output = state_->output.next_transition() == ticks;
    const auto pending = state_->device.next_event();
    const bool operation = pending && pending->ticks == ticks;
    state_->faulted = true;
    try {
      const auto completed = state_->device.advance(ticks);
      if (power) {
        state_->device.observe_power_ready();
        state_->power_ready = true;
        std::cerr << "NAND_AUTONOMOUS_POWER ticks=" << ticks << " timebase=" << state_->timebase_hz
                  << " reference-observation=100us fitted-latency=unknown\n";
      }
      if (operation) {
        if (!completed || completed->ticks != ticks) { fail("operation completion mismatch"); }
        std::cerr << "NAND_AUTONOMOUS_COMPLETION ticks=" << ticks
                  << " operation=" << static_cast<unsigned>(completed->operation) << '\n';
      } else if (completed) {
        fail("unoffered operation completed");
      }
      if (output) {
        std::cerr << "NAND_OUTPUT_BOUND ticks=" << ticks
                  << " last-native-pc=" << hex(state_->context.pc, 6)
                  << " omr=" << hex(state_->context.omr, 6) << " physical-edge=unobserved\n";
      }
      publish();
      drive(ticks);
      if (operation && state_->device.device().status_mode() && state_->previous_re == false) {
        const auto status = state_->device.device().status();
        std::cerr << "NAND_STATUS_REFRESH ticks=" << ticks << " value=" << hex(status.value, 2)
                  << " known=" << hex(status.known, 2)
                  << " controller-boundary-projection=1 physical-pad-delay=unknown\n";
      }
      state_->faulted = false;
    } catch (const Fault &cause) { fail(std::string("dispatch: ") + cause.what()); }
  }

  [[nodiscard]] NandPeerStatus status() const {
    healthy();
    return {.ticks = state_->device.ticks(),
            .ready = state_->device.device().ready(),
            .pending = state_->device.device().pending(),
            .next = next_event(),
            .cell = state_->device.device().cell({.page = 0, .column = 0})};
  }

  void after(const std::uint64_t ticks) {
    healthy();
    const auto page = state_->device.device().page();
    const auto column = state_->device.device().column();
    try {
      advance(ticks);
      observe(ticks);
      drive(ticks);
    } catch (const Fault &error) {
      state_->faulted = true;
      throw Fault("Joined NAND pins at X:0xffffbd/Y:0xfffff5 pc=" + hex(state_->context.pc, 6) +
                  " omr=" + hex(state_->context.omr, 6) + " ticks=" + std::to_string(ticks) +
                  " page-before=" + hex(page, 6) + " column-before=" + hex(column, 3) + ": " +
                  error.what());
    }
  }

  [[noreturn]] void fail(const std::string &reason) const {
    throw Fault("NAND output peer operation=event at=X:0xffffbd/Y:0xfffff5 last-native-pc=" +
                hex(state_->context.pc, 6) + " omr=" + hex(state_->context.omr, 6) +
                " native-ticks=" + std::to_string(machine_.clock().elapsed_ticks()) + ": " +
                reason);
  }

  void healthy() const {
    if (state_->faulted || state_->device.faulted()) {
      fail("quarantined after incomplete NAND operation or persistence failure");
    }
  }

  void advance(const std::uint64_t ticks) {
    const auto next = next_event();
    if (next && ticks >= *next) {
      throw Fault("NAND deadline was not dispatched by the independent board scheduler");
    }
    if (state_->device.advance(ticks)) { throw Fault("Unexpected undispatched NAND completion"); }
    publish();
  }

  void publish() {
    auto &timers = machine_.bus().timers();
    const auto ready = state_->device.device().ready();
    timers.set_tio_drive(ready_timer, ready);
    // Functional board sampling at the published NAND boundary. Only the
    // device's actual known R/B level qualifies Timer1's capture input.
    if (ready) { timers.observe_synchronized_tio(ready_timer, *ready); }
  }

  void drive(const std::uint64_t ticks) const {
    if (state_->device.device().status_mode() && state_->previous_re == false) {
      auto status = state_->device.device().status();
      status.known &= Nand::status_bits;
      status.value &= status.known;
      state_->output.refresh_status(ticks, status, NandStatusTiming::reference_controller_boundary);
    }
    const auto output = state_->output.output(ticks);
    DspGpioDrive drive{.floating = port_f_mask};
    if (!output.driven.has_value()) {
      drive.floating &= ~data_mask;
      drive.unknown = data_mask;
    } else if (*output.driven) {
      drive.floating &= ~data_mask;
      drive.mask = output.data.known;
      drive.levels = output.data.value;
      drive.unknown = data_mask & ~static_cast<unsigned>(output.data.known);
    }
    state_->drive = drive;
  }

  void latch(const NandWriteLatch input, const std::uint64_t ticks) const {
    std::cerr << "NAND_LATCH ticks=" << ticks << " pc=" << hex(state_->context.pc, 6)
              << " kind=" << static_cast<unsigned>(input.kind) << " value=" << hex(input.value, 2)
              << '\n';
    switch (input.kind) {
      case NandWriteKind::command: {
        state_->device.command(input.value);
        break;
      }
      case NandWriteKind::address: state_->device.address(input.value); break;
      case NandWriteKind::data: state_->device.write(input.value); break;
      default: throw Fault("Invalid NAND latch kind");
    }
  }

  void observe(const std::uint64_t ticks) {
    const auto controls = machine_.bus().gpio(DspPort::c).observe_pads(control_mask);
    if (controls.known != control_mask) {
      if (state_->connected) {
        throw Fault("NAND control knowledge lost after observation-window entry");
      }
      if (controls.known != state_->previous_known) {
        std::cerr << "UNQUALIFIED_NAND_STARTUP pc=" << hex(state_->context.pc, 6)
                  << " ticks=" << ticks
                  << " unknown-control-mask=" << hex(control_mask & ~controls.known, 3)
                  << " no-pre-window-pin-semantics=1\n";
        state_->previous_known = controls.known;
      }
      return;
    }
    if (!state_->connected) {
      std::cerr << "UNQUALIFIED_NAND_WINDOW pc=" << hex(state_->context.pc, 6) << " ticks=" << ticks
                << " controls=" << hex(controls.levels, 3) << " CE=held-selected-lab\n";
      state_->connected = true;
    }
    const bool was_neutral = state_->decoder.neutral();
    const auto logical = state_->decoder.observe(controls.levels);
    if (was_neutral != state_->decoder.neutral()) {
      std::cerr << "NAND_FUNCTIONAL_BUS ticks=" << ticks << " pc=" << hex(state_->context.pc, 6)
                << " raw=" << hex(controls.levels, 3) << " logical=" << hex(logical, 3)
                << " neutral=" << state_->decoder.neutral() << '\n';
    }
    const bool re_n = (logical & re_pin) != 0;
    const bool we_n = (logical & we_pin) != 0;
    const bool cle = (logical & cle_pin) != 0;
    const bool ale = (logical & ale_pin) != 0;
    const auto data = machine_.bus().gpio(DspPort::f).observe_pads(data_mask);
    const auto input = state_->write.observe(
        ticks,
        {.ce_n = false,
         .re_n = re_n,
         .we_n = we_n,
         .cle = cle,
         .ale = ale,
         .data = {.value = static_cast<std::uint8_t>(data.levels & data_mask),
                  .known = static_cast<std::uint8_t>(data.known & data_mask)}});
    if (input) { latch(*input, ticks); }
    const bool admitted = state_->setup.observe(
        ticks,
        {.ce_n = false,
         .re_n = re_n,
         .we_n = we_n,
         .ale = ale,
         .cle = cle,
         .ready = state_->device.device().ready()},
        state_->device.device().status_mode() ? NandReadKind::status70 : NandReadKind::array);
    state_->output.observe(ticks, {.ce_n = false, .re_n = re_n}, [&] {
      if (!admitted) { throw Fault("NAND protocol read without timing admission"); }
      const bool status = state_->device.device().status_mode();
      const auto mask =
          status ? (state_->device.device().status().known & Nand::status_bits) : Nand::byte_mask;
      const auto value = state_->device.read(static_cast<std::uint8_t>(mask));
      state_->trace_access = true;
      std::cerr << "NAND_BYTE ticks=" << ticks << " pc=" << hex(state_->context.pc, 6)
                << " value=" << hex(value, 2) << " next-column=" << state_->device.device().column()
                << '\n';
      return NandByte{.value = value, .known = static_cast<std::uint8_t>(mask)};
    });
    if (state_->previous_re == false && re_n) {
      const auto before_page = state_->device.device().page();
      const auto before_column = state_->device.device().column();
      state_->device.observe_read_end(NandReadEnd::re_high);
      if (before_column == Nand::page_bytes) {
        const auto ready = state_->device.device().ready();
        if (!ready) { throw Fault("P-part read-end lost Ready knowledge"); }
        std::cerr << "NAND_END ticks=" << ticks << " pc=" << hex(state_->context.pc, 6)
                  << " row=" << before_page << " next-row=" << state_->device.device().page()
                  << " ready=" << *ready << " reference-tRB=0ns\n";
      }
    }
    state_->previous_re = re_n;
    publish();
  }

  OwnDspMachine &machine_;
  std::unique_ptr<PeerState> state_;
  bool needs_binding_ = true;
  const std::shared_ptr<const PeripheralEventIdentity> identity_ =
      std::make_shared<const PeripheralEventIdentity>();
};

struct NandPeerSnapshot::Storage {
  PeerState state;
  const NandPeer *owner;
  const OwnDspMachine *machine;
  std::weak_ptr<const bool> lifetime;
  std::shared_ptr<const PeripheralEventIdentity> identity;
};

NandPeerSnapshot::NandPeerSnapshot(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

NandPeerSnapshot::~NandPeerSnapshot() = default;

NandPeer::NandPeer(OwnDspMachine &machine, Nand device)
    : NandPeer(machine, std::move(device), NandCellProfile{}) {}

NandPeer::NandPeer(OwnDspMachine &machine, Nand device, NandCellProfile profile)
    : NandPeer(machine, std::move(device), std::move(profile), NandBusContract::ordered_pins) {}

NandPeer::NandPeer(OwnDspMachine &machine,
                   Nand device,
                   NandCellProfile profile,
                   const NandBusContract contract)
    : impl_(std::make_unique<Impl>(machine,
                                   NandPeerMedium(NandWriteMedium(std::move(device),
                                                                  machine.clock().timebase_hz(),
                                                                  std::move(profile))),
                                   contract)) {
  impl_->bind(lifetime_);
  impl_->publish();
}

NandPeer::NandPeer(OwnDspMachine &machine,
                   std::unique_ptr<NandWriteStore> store,
                   const NandBusContract contract)
    : impl_(std::make_unique<Impl>(
          machine, NandPeerMedium(std::move(store), machine.clock().timebase_hz()), contract)) {
  impl_->bind(lifetime_);
  impl_->publish();
}

NandPeer::NandPeer(const NandPeerSnapshot &snapshot, OwnDspMachine &target) {
  if (!snapshot.storage_ ||
      (snapshot.storage_->lifetime.lock() && snapshot.storage_->machine == &target)) {
    throw Fault("NAND clone requires a distinct native owner and trusted payload");
  }
  const auto &saved = *snapshot.storage_;
  if (saved.state.timebase_hz != target.clock().timebase_hz()) {
    throw Fault("NAND clone requires matching device timebase");
  }
  impl_ = std::make_unique<Impl>(target, saved.state, saved.identity);
}

NandPeer::~NandPeer() = default;

NandPeerEvents NandPeer::events() {
  const std::weak_ptr<const bool> owner = lifetime_;
  return {.identity = impl_->identity_,
          .next_ticks = [this, owner]() -> std::optional<std::uint64_t> {
            if (owner.expired()) { throw Fault("NAND deadline query lost its live board owner"); }
            const auto next = impl_->next_event();
            return next;
          },
          .observe =
              [this, owner](const std::uint64_t ticks) {
                if (owner.expired()) {
                  throw Fault("NAND event dispatch lost its live board owner");
                }
                impl_->bind(owner);
                impl_->dispatch(ticks);
              }};
}

void NandPeer::before_native(const std::uint64_t common_ticks) {
  impl_->bind(lifetime_);
  const auto cpu = impl_->machine_.cpu().registers();
  impl_->before(common_ticks, {.pc = cpu.pc, .omr = cpu.omr});
}

void NandPeer::after_native(const std::uint64_t common_ticks,
                            const std::optional<std::uint32_t> native_pc) {
  impl_->bind(lifetime_);
  if (native_pc) { impl_->state_->context.pc = *native_pc; }
  impl_->after(common_ticks);
}

NandPeerDetailed NandPeer::detailed() const {
  return impl_->detailed();
}

Nand NandPeer::protocol_copy() const {
  impl_->healthy();
  return impl_->state_->device.device();
}

bool NandPeer::faulted() const {
  return impl_->state_->faulted || impl_->state_->device.faulted();
}

bool NandPeer::durable() const {
  return impl_->state_->device.durable();
}

NandPeerSnapshot NandPeer::checkpoint() const {
  impl_->healthy();
  return NandPeerSnapshot(std::make_shared<const NandPeerSnapshot::Storage>(
      NandPeerSnapshot::Storage{.state = *impl_->state_,
                                .owner = this,
                                .machine = &impl_->machine_,
                                .lifetime = lifetime_,
                                .identity = impl_->identity_}));
}

void NandPeer::restore(const NandPeerSnapshot &snapshot) {
  if (durable()) { throw Fault("A live NAND store cannot be restored from RAM"); }
  if (!snapshot.storage_ || snapshot.storage_->owner != this ||
      snapshot.storage_->lifetime.lock() != lifetime_ ||
      snapshot.storage_->identity != impl_->identity_) {
    throw Fault("NAND payload restore requires its original live board owner");
  }
  impl_->state_ = std::make_unique<PeerState>(snapshot.storage_->state);
  impl_->needs_binding_ = true;
}
}
