// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/execute.hpp"

#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/mcu_flash.hpp"
#include "io/sci.hpp"
#include "io/shi.hpp"
#include "io/usb.hpp"
#include "mcu/clock.hpp"
#include "mcu/memory.hpp"
#include "mcu/ports.hpp"
#include "mcu/timer.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t reset_vector = 0xfffe;
constexpr std::uint16_t mapping_first = 0x10;
constexpr std::uint16_t mapping_last = 0x13;
constexpr std::uint8_t all_bits = 0xff;
// An enabled but masked IRQ still needs native CCR/RTI effect boundaries.
// This unreachable horizon selects that Core path without an execution quantum.
constexpr auto no_deadline = std::numeric_limits<Timer::Cycle>::max() / 2;

struct Devices {
  Timer timer;
  Clock clock;
  Sci sci;
  Usb usb;
  Timer::Cycle idle = 0;

  bool advance(const cpu12::Core &core) noexcept {
    const auto cycles = core.progress().estimated_cycles;
    return idle <= std::numeric_limits<Timer::Cycle>::max() - cycles &&
           timer.advance(cycles + idle) && clock.advance(cycles + idle) &&
           sci.advance(cycles + idle);
  }

  [[nodiscard]] std::optional<std::uint16_t> pending() const noexcept {
    return std::max({timer.pending(), clock.pending(), sci.pending(), usb.pending()});
  }

  [[nodiscard]] std::optional<Timer::Cycle> deadline() const noexcept {
    auto next = timer.deadline();
    const auto offer = [&next](const std::optional<Timer::Cycle> candidate) {
      if (candidate && (!next || *candidate < *next)) { next = candidate; }
    };
    offer(clock.deadline());
    offer(sci.deadline());
    return next;
  }

  [[nodiscard]] bool interrupts_enabled() const noexcept {
    return timer.interrupts_enabled() || clock.interrupts_enabled() || sci.interrupts_enabled() ||
           usb.interrupts_enabled();
  }

  void activity(const cpu12::ExecutionState state) {
    switch (state) {
      case cpu12::ExecutionState::running:
        clock.activity(Clock::Activity::running);
        timer.activity(Timer::Activity::running);
        break;
      case cpu12::ExecutionState::waiting:
        clock.activity(Clock::Activity::waiting);
        timer.activity(clock.bus_running() ? Timer::Activity::waiting : Timer::Activity::stopped);
        break;
      case cpu12::ExecutionState::stopped:
        clock.activity(Clock::Activity::stopped);
        timer.activity(Timer::Activity::stopped);
        break;
      default: throw std::logic_error("Unknown MCU execution state");
    }
    sci.activity(
        {.waiting = state == cpu12::ExecutionState::waiting, .clocked = clock.bus_running()});
  }
};

class Io {
public:
  Io(Memory &memory,
     const FlashImage &image,
     Devices &devices,
     Shi &shi,
     DspReset &reset,
     const std::stop_token &stop)
      : memory_(memory), flash_(image), ports_(shi), devices_(devices), reset_(reset), stop_(stop) {
    // B3/B5..B7 are passive controls, with no input producer in the firmware
    // contract. Use an inactive sample while they are inputs, not a device reply.
    constexpr std::uint8_t passive_controls = 0xe8;
    // DSP56371UM table 2-7: SS, SCK and MOSI have internal pull-ups. The
    // application releases B1/B2/B4 after boot; this is bias, not HREQ/data.
    constexpr std::uint8_t spi_pullups = 0x16;
    constexpr std::uint8_t attached = passive_controls | spi_pullups;
    ports_.input(Port::b, {.mask = attached, .known = attached, .value = spi_pullups});
    // DSP56371 RESET has an internal pull-up. U0 drives it low or releases it.
    ports_.input(Port::u, {.mask = 1, .known = 1, .value = 1});
  }

  void bind(cpu12::Core &core) noexcept { core_ = &core; }

  cpu12::Io callbacks() noexcept;

  bool complete_events() {
    if (release_reset_) {
      if (!reset_.release(stop_)) { return false; }
      release_reset_ = false;
    }
    return ports_.complete_boot_transfer(stop_);
  }

private:
  template <typename Action> bool access_devices(const Action &action) noexcept {
    const auto pending = devices_.pending();
    const auto deadline = devices_.deadline();
    const auto enabled = devices_.interrupts_enabled();
    if (!devices_.advance(*core_) || !action()) { return false; }
    if (pending != devices_.pending() || deadline != devices_.deadline() ||
        enabled != devices_.interrupts_enabled()) {
      core_->request_yield();
    }
    return true;
  }

  bool interrupted() noexcept {
    if (!stop_.stop_requested()) { return false; }
    core_->request_yield();
    return true;
  }

  bool read(const std::uint16_t address,
            std::uint8_t &value,
            const std::uint8_t mask = all_bits) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(address);
    if (at.space == Space::registers && McuFlash::contains(at.offset)) {
      return flash_.read(at.offset, value);
    }
    if (at.space == Space::registers && Usb::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.usb.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Sci::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.sci.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Clock::contains(at.offset)) {
      return access_devices([&] { return devices_.clock.read(at.offset, value); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted =
          access_devices([&] { return devices_.timer.read({.offset = at.offset}, word); });
      value = static_cast<std::uint8_t>(word);
      return accepted;
    }
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      return ports_.read(at.offset, value, mask);
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      value = memory_.read_mapping(at.offset);
      return true;
    }
    return memory_.read(address, value);
  }

  bool write_usb(const Usb::Write access) noexcept {
    return access_devices([&] {
      if (!devices_.usb.write(access, devices_.clock.usb_module_control())) { return false; }
      if (devices_.usb.mapping_pending()) { core_->request_yield(); }
      return true;
    });
  }

  bool write_port(const Ports::Write access) noexcept {
    constexpr std::uint16_t data_u = 0x278;
    constexpr std::uint16_t direction_u = 0x27a;
    if (!ports_.write(access)) { return false; }
    if (ports_.boot_pending()) { core_->request_yield(); }
    if (access.offset != data_u && access.offset != direction_u) { return true; }
    const auto output = ports_.output(Port::u);
    if (!output || ((output->direction & 1) != 0 && (output->known & 1) == 0)) { return false; }
    const bool held = (output->direction & 1) != 0 && (output->value & 1) == 0;
    const bool previous = reset_.state().held;
    if (held && !previous) {
      ports_.reset_boot_transfer();
      if (!reset_.assert_reset()) { return false; }
    } else if (!held && previous) {
      release_reset_ = true;
      core_->request_yield();
    }
    ports_.input(Port::u,
                 {.mask = 1, .known = 1, .value = held ? std::uint8_t{0} : std::uint8_t{1}});
    return true;
  }

  bool write(const cpu12::ByteWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    if (at.space == Space::flash) {
      return flash_.write_array({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && McuFlash::contains(at.offset)) {
      return flash_.write({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && Usb::contains(at.offset)) {
      return write_usb({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && Sci::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.sci.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Clock::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.clock.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      return access_devices(
          [&] { return devices_.timer.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      return write_port({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && at.offset >= mapping_first && at.offset <= mapping_last) {
      if (memory_.write_mapping({.address = at.offset, .value = access.value})) {
        core_->request_yield();
      }
      return true;
    }
    return memory_.write(access);
  }

  bool read_word(const std::uint16_t address, std::uint16_t &value) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(address);
    if (at.space == Space::registers && Usb::contains(at.offset) &&
        Usb::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.usb.read({.offset = at.offset, .bytes = 2}, value); });
    }
    if (at.space == Space::registers && Sci::contains(at.offset) &&
        Sci::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.sci.read({.offset = at.offset, .bytes = 2}, value); });
    }
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices(
          [&] { return devices_.timer.read({.offset = at.offset, .bytes = 2}, value); });
    }
    std::uint8_t high = 0;
    std::uint8_t low = 0;
    if (!read(address, high) || !read(static_cast<std::uint16_t>(address + 1), low)) {
      return false;
    }
    value = static_cast<std::uint16_t>((unsigned{high} << 8U) | low);
    return true;
  }

  bool write_word(const cpu12::WordWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    const auto next = memory_.locate(static_cast<std::uint16_t>(access.address + 1));
    if (at.space == Space::flash && next.space == Space::flash && next.offset == at.offset + 1) {
      return flash_.write_array({.offset = at.offset, .value = access.value, .bytes = 2});
    }
    if (at.space == Space::registers && Usb::contains(at.offset) &&
        Usb::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return write_usb({.offset = at.offset, .value = access.value, .bytes = 2});
    }
    if (at.space == Space::registers && Sci::contains(at.offset) &&
        Sci::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.sci.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    if (at.space == Space::registers && Clock::contains(at.offset) &&
        Clock::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.clock.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_devices([&] {
        return devices_.timer.write({.offset = at.offset, .value = access.value, .bytes = 2});
      });
    }
    return write({.address = access.address,
                  .value = static_cast<std::uint8_t>(access.value >> 8U)}) &&
           write({.address = static_cast<std::uint16_t>(access.address + 1),
                  .value = static_cast<std::uint8_t>(access.value & all_bits)});
  }

  Memory &memory_;
  McuFlash flash_;
  Ports ports_;
  Devices &devices_;
  DspReset &reset_;
  bool release_reset_ = false;
  const std::stop_token &stop_;
  cpu12::Core *core_ = nullptr;
};

cpu12::Io Io::callbacks() noexcept {
  return {.owner = this,
          .read =
              [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(address, value);
              },
          .write =
              [](void *owner, const cpu12::ByteWrite access) noexcept {
                return static_cast<Io *>(owner)->write(access);
              },
          .read_word =
              [](void *owner, const std::uint16_t address, std::uint16_t &value) noexcept {
                return static_cast<Io *>(owner)->read_word(address, value);
              },
          .write_word =
              [](void *owner, const cpu12::WordWrite access) noexcept {
                return static_cast<Io *>(owner)->write_word(access);
              },
          .fetch =
              [](void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(address, value);
              },
          .read_masked =
              [](void *owner, const cpu12::MaskedByteRead access, std::uint8_t &value) noexcept {
                return static_cast<Io *>(owner)->read(access.address, value, access.mask);
              }};
}

Access operation(const cpu12::Fault fault) {
  switch (fault) {
    case cpu12::Fault::memory_read: return Access::read;
    case cpu12::Fault::memory_write: return Access::write;
    case cpu12::Fault::instruction_fetch: return Access::fetch;
    case cpu12::Fault::none:
    case cpu12::Fault::invalid_input:
    case cpu12::Fault::unsupported_instruction:
    case cpu12::Fault::unsupported_host:
    case cpu12::Fault::reentrant:
    case cpu12::Fault::unsupported_state:
    default: return Access::execute;
  }
}

bool interrupt(cpu12::Core &core, const Devices &devices, ExecutionReport &report) {
  const auto vector = devices.pending();
  if (!vector || !core.interrupt_ready(cpu12::InterruptKind::maskable)) { return false; }
  const auto entered = core.interrupt({.vector = *vector});
  if (entered.fault != cpu12::Fault::none) {
    report.failure = Failure{.operation = operation(entered.fault),
                             .pc = entered.pc,
                             .address = entered.address,
                             .value = entered.value,
                             .code = static_cast<std::uint32_t>(entered.fault),
                             .reason = "MCU device interrupt entry failed"};
    return false;
  }
  if (!entered.accepted) { throw std::logic_error("Eligible MCU device interrupt was refused"); }
  return true;
}

void advance(const cpu12::Core &core, Devices &devices) {
  if (!devices.advance(core)) {
    if (devices.sci.error() != nullptr) { throw std::runtime_error(devices.sci.error()); }
    throw std::overflow_error("MCU device time is not representable or monotonic");
  }
}

std::vector<cpu12::Ram> regions(Memory &memory, Usb &usb) {
  auto result = memory.regions();
  constexpr std::uint16_t register_mapping = 0x11;
  const auto base =
      static_cast<std::uint16_t>(unsigned{memory.read_mapping(register_mapping)} << 8U);
  for (const auto window : usb.regions(base)) {
    if (!window.bytes.empty()) { result.push_back(window); }
  }
  return result;
}

enum class Dispatch : std::uint8_t { run, again, done };

Dispatch prepare(cpu12::Core &core, Devices &devices, ExecutionReport &report) {
  advance(core, devices);
  const auto state = core.state().execution;
  if (state == cpu12::ExecutionState::stopped) { return Dispatch::done; }
  if (interrupt(core, devices, report)) {
    devices.activity(cpu12::ExecutionState::running);
    return Dispatch::again;
  }
  if (report.failure) { return Dispatch::done; }
  if (core.state().returned_frame) { core.close_return_window(); }
  if (state == cpu12::ExecutionState::waiting) {
    const auto next = devices.deadline();
    if (!next) { return Dispatch::done; }
    devices.idle = *next - core.progress().estimated_cycles;
    return Dispatch::again;
  }
  return Dispatch::run;
}

}

ExecutionReport execute(FlashImage &image, Shi &shi, DspReset &reset, const std::stop_token &stop) {
  Memory memory(image);
  Devices devices;
  auto grants = regions(memory, devices.usb);
  Io io(memory, image, devices, shi, reset, stop);
  auto callbacks = io.callbacks();
  cpu12::Core core(cpu12::Memory::mapped(grants), &callbacks);
  io.bind(core);
  ExecutionReport report;
  const auto initial = core.reset(reset_vector);
  if (!initial.accepted) {
    report.failure = Failure{.operation = Access::reset,
                             .pc = initial.pc,
                             .address = initial.address,
                             .value = initial.value,
                             .code = static_cast<std::uint32_t>(initial.fault),
                             .reason = "MCU reset vector access failed"};
    return report;
  }
  while (!stop.stop_requested()) {
    const auto dispatch = prepare(core, devices, report);
    if (dispatch == Dispatch::done) { break; }
    if (dispatch == Dispatch::again) { continue; }
    const auto deadline = devices.deadline();
    const auto result = deadline || devices.interrupts_enabled()
                            ? core.run_until(deadline ? *deadline - devices.idle : no_deadline)
                            : core.run();
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (result.fault != cpu12::Fault::none) {
      report.failure = Failure{.operation = operation(result.fault),
                               .pc = result.pc,
                               .address = result.address,
                               .value = result.value,
                               .code = static_cast<std::uint32_t>(result.fault),
                               .reason = devices.sci.error() != nullptr
                                             ? devices.sci.error()
                                             : "MCU access or instruction is not implemented"};
      break;
    }
    if (!io.complete_events()) { break; }
    const auto memory_changed = memory.commit_mapping();
    const auto buffers_changed = devices.usb.commit_mapping();
    if (memory_changed || buffers_changed) {
      auto next_grants = regions(memory, devices.usb);
      core.set_memory(cpu12::Memory::mapped(next_grants));
      grants.swap(next_grants);
    }
    advance(core, devices);
    devices.activity(result.state);
  }

  return report;
}
}
