// SPDX-License-Identifier: GPL-3.0-only
#include "nand_native_peer.hpp"

#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_schedule.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/dsp_scheduled_run.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_read_setup.hpp"
#include "storage/nand_read_timing.hpp"
#include "storage/nand_write_medium.hpp"
#include "storage/nand_write_timing.hpp"

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace blofeld::test {
using namespace blofeld;

namespace {
constexpr unsigned control_mask = 0xf00;
constexpr unsigned re_pin = 0x100;
constexpr unsigned we_pin = 0x200;
constexpr unsigned cle_pin = 0x400;
constexpr unsigned ale_pin = 0x800;
constexpr unsigned data_mask = 0xff;
constexpr unsigned port_f_mask = 0x7ff;
constexpr unsigned ready_timer = 1;

struct PeerState {
  explicit PeerState(NandWriteMedium clock) : device(std::move(clock)) {}

  bool power_ready = false;
  bool faulted = false;
  NandWriteMedium device;
  NandWriteTiming write{nand_timebase_hz};
  NandReadSetup setup{nand_timebase_hz};
  NandReadTiming output{nand_timebase_hz};
  DspRunContext context;
  std::optional<unsigned> previous_known;
  std::optional<bool> previous_re;
  bool connected = false;
  bool trace_access = false;
};
}

struct NandPeer::Impl {
  Impl(DspMachine &machine, Nand device, NandCellProfile profile)
      : machine_(machine),
        state_(std::make_unique<PeerState>(
            NandWriteMedium(std::move(device), nand_timebase_hz, std::move(profile)))) {
    std::cerr << "UNQUALIFIED_OWNED_NAND CE=held-selected-lab power=100us-reference "
                 "part=P-sequential GPIO=canonical-own-program sample=period-completion "
                 "board-qualified=0\n";
    publish();
    drive(0);
  }

  Impl(DspMachine &target,
       const PeerState &state,
       std::shared_ptr<const PeripheralEventIdentity> identity)
      : machine_(target),
        state_(std::make_unique<PeerState>(state)),
        identity_(std::move(identity)) {}

  [[nodiscard]] PeerDetailed detailed() const {
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
            .trace_access = state_->trace_access};
  }

  void record_restart(const std::uint8_t value) const {
    const auto ready = state_->device.device().ready();
    if (!ready || !*ready) { throw Fault("Random-read restart requires actual known Ready high"); }
    std::cerr << "UNQUALIFIED_RANDOM_RESTART operation=command at=X:0xffffbd/Y:0xfffff5"
              << " pc=" << hex(state_->context.pc, 6) << " omr=" << hex(state_->context.omr, 6)
              << " value=" << hex(value, 2) << " ticks=" << state_->device.ticks()
              << " row=" << state_->device.device().page()
              << " column=" << state_->device.device().column() << " ready=" << *ready
              << " hypothesis=fresh-read-command-terminates-sequential-state CE-edge-injected=0\n";
  }

  void record(const DspNativeAccess &access) const {
    if (!state_->trace_access || access.stage != DspNativeAccessStage::after || !access.value ||
        (access.kind != DspNativeAccessKind::read && access.kind != DspNativeAccessKind::write &&
         access.kind != DspNativeAccessKind::bit)) {
      return;
    }
    std::cerr << "NAND_ACCESS pc=" << hex(access.pc, 6)
              << " kind=" << static_cast<unsigned>(access.kind)
              << " space=" << static_cast<unsigned>(access.target.space)
              << " address=" << hex(access.target.address, 6) << " value=" << hex(*access.value, 6)
              << " ordinal=" << access.ordinal << '\n';
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
    if (!state_->power_ready) { next = power_ready_tick; }
    if (operation && (!next || operation->ticks < *next)) { next = operation->ticks; }
    const auto output = state_->output.next_transition();
    if (output && (!next || *output < *next)) { next = output; }
    return next;
  }

  void dispatch(const std::uint64_t ticks) {
    if (next_event() != ticks) { fail("event requires its exact offered deadline"); }
    const bool power = !state_->power_ready && ticks == power_ready_tick;
    const bool output = state_->output.next_transition() == ticks;
    const auto pending = state_->device.next_event();
    const bool operation = pending && pending->ticks == ticks;
    state_->faulted = true;
    try {
      const auto completed = state_->device.advance(ticks);
      if (power) {
        state_->device.observe_power_ready();
        state_->power_ready = true;
        std::cerr << "NAND_AUTONOMOUS_POWER ticks=" << ticks
                  << " timebase=1080000000 reference-observation=100us fitted-latency=unknown\n";
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

  void reattach() {
    healthy();
    publish();
    drive(state_->device.ticks());
    std::cerr << "NAND_LIFETIME_REATTACH ticks=" << state_->device.ticks()
              << " pending=" << static_cast<unsigned>(state_->device.device().pending())
              << " power-ready=" << state_->power_ready << " protocol-and-cells-preserved=1\n";
  }

  [[nodiscard]] PeerStatus status() const {
    healthy();
    return {.ticks = state_->device.ticks(),
            .ready = state_->device.device().ready(),
            .pending = state_->device.device().pending(),
            .next = next_event(),
            .cell = state_->device.device().cell({.page = 0, .column = 0})};
  }

  void after(const std::uint64_t ticks) {
    try {
      advance(ticks);
      observe(ticks);
      drive(ticks);
    } catch (const Fault &error) {
      state_->faulted = true;
      throw Fault("Joined NAND pins at X:0xffffbd/Y:0xfffff5 pc=" + hex(state_->context.pc, 6) +
                  " omr=" + hex(state_->context.omr, 6) + " ticks=" + std::to_string(ticks) +
                  " page=" + hex(state_->device.device().page(), 6) +
                  " column=" + hex(state_->device.device().column(), 3) + ": " + error.what());
    }
  }

  [[noreturn]] void fail(const std::string &reason) const {
    throw Fault("NAND output peer operation=event at=X:0xffffbd/Y:0xfffff5 last-native-pc=" +
                hex(state_->context.pc, 6) + " omr=" + hex(state_->context.omr, 6) +
                " ticks=" + std::to_string(state_->device.ticks()) + ": " + reason);
  }

  void healthy() const {
    if (state_->faulted) { fail("quarantined after incomplete output event"); }
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
    machine_.bus().timers().set_tio_drive(ready_timer, state_->device.device().ready());
  }

  void drive(const std::uint64_t ticks) {
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
    machine_.bus().gpio(DspPort::f).set_drive(drive);
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
    const bool re_n = (controls.levels & re_pin) != 0;
    const bool we_n = (controls.levels & we_pin) != 0;
    const bool cle = (controls.levels & cle_pin) != 0;
    const bool ale = (controls.levels & ale_pin) != 0;
    const auto data = machine_.bus().gpio(DspPort::f).outputs();
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
    machine_.bus().timers().set_tio_drive(ready_timer, state_->device.device().ready());
  }

  static constexpr std::uint64_t power_ready_tick = 108000;
  DspMachine &machine_;
  std::unique_ptr<PeerState> state_;
  const std::shared_ptr<const PeripheralEventIdentity> identity_ =
      std::make_shared<const PeripheralEventIdentity>();
};

struct NandPeerSnapshot::Storage {
  PeerState state;
  const NandPeer *owner;
  const DspMachine *machine;
  std::weak_ptr<const bool> lifetime;
  std::shared_ptr<const PeripheralEventIdentity> identity;
};

NandPeerSnapshot::NandPeerSnapshot(std::shared_ptr<const Storage> storage)
    : storage_(std::move(storage)) {}

NandPeerSnapshot::~NandPeerSnapshot() = default;

NandPeer::NandPeer(DspMachine &machine, Nand device)
    : NandPeer(machine, std::move(device), NandCellProfile{}) {}

NandPeer::NandPeer(DspMachine &machine, Nand device, NandCellProfile profile)
    : impl_(std::make_unique<Impl>(machine, std::move(device), std::move(profile))) {}

NandPeer::NandPeer(const NandPeerSnapshot &snapshot, DspMachine &target) {
  if (!snapshot.storage_ ||
      (snapshot.storage_->lifetime.lock() && snapshot.storage_->machine == &target)) {
    throw Fault("NAND clone requires a distinct native owner and trusted payload");
  }
  const auto &saved = *snapshot.storage_;
  impl_ = std::make_unique<Impl>(target, saved.state, saved.identity);
}

NandPeer::~NandPeer() = default;

DspFetchPeripheralEvents NandPeer::events() {
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
                impl_->dispatch(ticks);
              },
          .order = DspExternalOrder::external_before_dsp,
          .maximum_events = 4};
}

void NandPeer::before_native(const std::uint64_t common_ticks) {
  const auto cpu = impl_->machine_.cpu().state();
  impl_->before(common_ticks, {.pc = cpu.pc, .omr = cpu.omr});
}

void NandPeer::after_native(const std::uint64_t common_ticks) {
  impl_->after(common_ticks);
}

PeerDetailed NandPeer::detailed() const {
  return impl_->detailed();
}

Nand NandPeer::protocol_copy() const {
  impl_->healthy();
  return impl_->state_->device.device();
}

bool NandPeer::faulted() const {
  return impl_->state_->faulted;
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
  if (!snapshot.storage_ || snapshot.storage_->owner != this ||
      snapshot.storage_->lifetime.lock() != lifetime_ ||
      snapshot.storage_->identity != impl_->identity_) {
    throw Fault("NAND payload restore requires its original live board owner");
  }
  impl_->state_ = std::make_unique<PeerState>(snapshot.storage_->state);
}
}
