// SPDX-License-Identifier: GPL-3.0-only
#include "vi/io/memory_controller.hpp"

#include "common/bus.hpp"
#include "common/telemetry.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_peripheral_calendar.hpp"
#include "runtime/gpio_pads.hpp"
#include "runtime/nand_peer_medium.hpp"
#include "storage/nand.hpp"
#include "storage/nand_cell_model.hpp"
#include "storage/nand_write_medium.hpp"
#include "storage/nand_write_store.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

namespace blofeld {
namespace {
constexpr unsigned power_intervals_per_second = 10000;
constexpr unsigned reserved_status_bits = 0x3e;
constexpr unsigned held_status_controls = 0x600;
constexpr unsigned data_mask = 0xff, controls_mask = 0xf00;
constexpr unsigned re_n = 0x100, we_n = 0x200, cle = 0x400, ale = 0x800;

// Decode firmware register transactions while the shared byte port changes owner.
class TransactionDecoder {
public:
  unsigned observe(const unsigned controls, const bool host_outputs) {
    constexpr unsigned strobes = re_n | we_n;
    if ((controls & ~controls_mask) != 0) { throw Fault("Invalid NAND bus control mask"); }
    const auto current = controls & strobes;
    const auto previous = previous_.value_or(strobes) & strobes;
    const bool active = current != 0 && current != strobes;
    if (host_outputs || host_owned_) {
      if (!host_owned_ && previous == re_n) {
        throw Fault("NAND functional bus operation=decode at=X:0xffffbd previous=" +
                    hex(previous_.value_or(strobes), 3) + " controls=" + hex(controls, 3) +
                    ": host ownership interrupts an unfinished NAND write");
      }
      host_owned_ = host_outputs || current != strobes;
      previous_ = controls;
      return host_owned_ ? strobes : controls;
    }
    if ((current == 0 && previous == re_n) || (previous == 0 && active)) {
      throw Fault("NAND functional bus operation=decode at=X:0xffffbd previous=" +
                  hex(previous_.value_or(strobes), 3) + " controls=" + hex(controls, 3) +
                  ": neutral phase requires a completed write and inactive transfer entry");
    }
    previous_ = controls;
    return current == 0 ? controls | strobes : controls;
  }

  [[nodiscard]] bool neutral() const {
    return host_owned_ || (previous_ && (*previous_ & (re_n | we_n)) == 0);
  }

private:
  std::optional<unsigned> previous_;
  bool host_owned_ = false;
};
}

struct MemoryController::Impl {
  Impl(DspMachine &owner, NandPeerMedium storage)
      : machine(owner),
        medium(std::move(storage)),
        power_at(owner.clock().timebase_hz() / power_intervals_per_second) {
    machine.bus()
        .gpio(DspPort::f)
        .attach_pad_source([this, lease = std::weak_ptr(lifetime)](const unsigned mask) {
          if (lease.expired()) { throw Fault("NAND register source has expired"); }
          return data(mask);
        });
    publish();
  }

  [[noreturn]] void fail(const std::string &reason) const {
    throw Fault("NAND operation=transaction at=X:0xffffbd/Y:0xfffff5 pc=" + hex(pc, 6) +
                " ticks=" + std::to_string(medium.ticks()) + ": " + reason);
  }

  void healthy() const {
    if (failed || medium.faulted()) { fail("reload required after incomplete transaction"); }
  }

  std::optional<std::uint64_t> next() const {
    healthy();
    if (!powered) { return power_at; }
    const auto event = medium.next_event();
    return event ? std::optional{event->ticks} : std::nullopt;
  }

  void publish() {
    auto &timers = machine.bus().timers();
    const auto ready = medium.device().ready();
    timers.set_tio_drive(1, ready);
    if (ready) { timers.observe_synchronized_tio(1, *ready); }
  }

  void advance(const std::uint64_t ticks, const bool dispatch) {
    healthy();
    const auto event = next();
    if (event && ticks >= *event && (!dispatch || ticks != *event)) {
      fail("operation completion must be delivered at its event boundary");
    }
    static_cast<void>(medium.advance(ticks));
    if (!powered && ticks == power_at) {
      medium.observe_power_ready();
      powered = true;
    }
    publish();
    refresh_status();
  }

  void refresh_status() {
    if (reading && medium.device().status_mode()) {
      output = medium.device().status();
      // Only C1 contains NAND status; the remaining register bits are reserved zero.
      output.known |= reserved_status_bits;
    }
  }

  GpioPadSample data(const unsigned mask) const {
    healthy();
    const auto local = machine.bus().gpio(DspPort::f).outputs();
    const auto driven = (local.mask | local.peripheral) & mask;
    const auto input = mask & ~driven;
    const auto known =
        (local.known & driven) | (input & (reading ? (output.known | ~data_mask) : mask));
    return {.known = known,
            .levels = ((local.levels & driven) | (reading ? output.value & input : 0U)) & known};
  }

  void latch(const unsigned controls) {
    const auto source = machine.bus().gpio(DspPort::f).observe_pads(data_mask);
    if (source.known != data_mask) { fail("unknown NAND write byte"); }
    const auto byte = static_cast<std::uint8_t>(source.levels);
    if ((controls & (cle | ale)) == (cle | ale)) { fail("command and address selected together"); }
    if ((controls & cle) != 0) {
      medium.command(byte);
    } else if ((controls & ale) != 0) {
      medium.address(byte);
    } else {
      medium.write(byte);
    }
    telemetry.emit([&](std::ostream &out) {
      out << "NAND-WRITE ticks=" << medium.ticks() << " pc=" << hex(pc, 6)
          << " controls=" << hex(controls, 3) << " byte=" << hex(byte, 2) << '\n';
    });
  }

  void observe() {
    const auto controls = machine.bus().gpio(DspPort::c).observe_pads(controls_mask);
    if (controls.known != controls_mask) {
      if (previous) { fail("NAND controls became unknown"); }
      return;
    }
    const auto host = machine.bus().gpio(DspPort::c).outputs().mask & data_mask;
    if (host != 0 && host != data_mask) { fail("partial shared data-register ownership"); }
    auto current = decoder.observe(controls.levels, host == data_mask);
    if (medium.device().status_mode() && current == held_status_controls) { current = we_n; }
    if (!previous && (current & (re_n | we_n)) != (re_n | we_n)) {
      fail("first transaction requires inactive read and write strobes");
    }
    const auto before = previous.value_or(re_n | we_n);
    if ((before & we_n) == 0 && (current & we_n) != 0) { latch(current); }
    if ((before & re_n) != 0 && (current & re_n) == 0) {
      if ((current & we_n) == 0) { fail("overlapping read and write transactions"); }
      const auto mask = medium.device().status_mode() ? Nand::status_bits : Nand::byte_mask;
      output = {.value = medium.read(static_cast<std::uint8_t>(mask)),
                .known = static_cast<std::uint8_t>(mask)};
      reading = true;
    }
    if ((before & re_n) == 0 && (current & re_n) != 0) {
      medium.observe_read_end(decoder.neutral() && !medium.device().status_mode()
                                  ? NandReadEnd::ce_high
                                  : NandReadEnd::re_high);
      reading = false;
    }
    previous = current;
    refresh_status();
    publish();
  }

  const std::shared_ptr<const bool> lifetime = std::make_shared<const bool>(true);
  DspMachine &machine;
  NandPeerMedium medium;
  const std::uint64_t power_at;
  TransactionDecoder decoder;
  std::shared_ptr<const PeripheralEventIdentity> identity =
      std::make_shared<const PeripheralEventIdentity>();
  std::optional<unsigned> previous;
  NandByte output;
  Telemetry telemetry;
  unsigned pc = 0;
  bool powered = false;
  bool reading = false;
  bool failed = false;
};

MemoryController::MemoryController(DspMachine &machine, Nand device, NandCellProfile profile)
    : impl_(std::make_unique<Impl>(machine,
                                   NandPeerMedium(NandWriteMedium(std::move(device),
                                                                  machine.clock().timebase_hz(),
                                                                  std::move(profile))))) {}

MemoryController::MemoryController(DspMachine &machine, std::unique_ptr<NandWriteStore> store)
    : impl_(std::make_unique<Impl>(
          machine, NandPeerMedium(std::move(store), machine.clock().timebase_hz()))) {}

MemoryController::~MemoryController() = default;

void MemoryController::set_telemetry(std::ostream *output) {
  impl_->telemetry = Telemetry(output);
}

MemoryControllerEvents MemoryController::events() {
  const std::weak_ptr lease(impl_->lifetime);
  return {.identity = impl_->identity,
          .next_ticks =
              [this, lease] {
                if (lease.expired()) { throw Fault("NAND event owner has expired"); }
                return impl_->next();
              },
          .observe =
              [this, lease](const auto ticks) {
                if (lease.expired()) { throw Fault("NAND event owner has expired"); }
                if (impl_->next() != ticks) {
                  throw Fault("NAND dispatch requires its exact deadline");
                }
                impl_->advance(ticks, true);
              }};
}

void MemoryController::before_native(const std::uint64_t ticks) {
  impl_->advance(ticks, false);
}

void MemoryController::after_native(const std::uint64_t ticks,
                                    const std::optional<std::uint32_t> pc) {
  if (pc) { impl_->pc = *pc; }
  try {
    impl_->advance(ticks, false);
    impl_->observe();
  } catch (...) {
    impl_->failed = true;
    throw;
  }
}

Nand MemoryController::protocol_copy() const {
  impl_->healthy();
  return impl_->medium.device();
}

bool MemoryController::faulted() const {
  return impl_->failed || impl_->medium.faulted();
}

bool MemoryController::durable() const {
  return impl_->medium.durable();
}
}
