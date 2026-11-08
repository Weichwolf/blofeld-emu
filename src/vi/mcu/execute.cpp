// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/execute.hpp"

#include "execution.hpp"
#include "io/mcu_flash.hpp"
#include "io/shi.hpp"
#include "mcu/memory.hpp"
#include "mcu/ports.hpp"
#include "mcu/timer.hpp"
#include "storage/flash_image.hpp"

#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t reset_vector = 0xfffe;
constexpr std::uint16_t mapping_first = 0x10;
constexpr std::uint16_t mapping_last = 0x13;
constexpr std::uint8_t all_bits = 0xff;
// An enabled but masked IRQ still needs native CCR/RTI effect boundaries.
// This unreachable horizon selects that Core path without an execution quantum.
constexpr auto no_deadline = std::numeric_limits<Timer::Cycle>::max() / 2;

class Io {
public:
  Io(Memory &memory,
     const FlashImage &image,
     Timer &timer,
     const Timer::Cycle &idle,
     Shi &shi,
     const std::stop_token &stop)
      : memory_(memory), flash_(image), ports_(shi), timer_(timer), idle_(idle), stop_(stop) {}

  void bind(cpu12::Core &core) noexcept { core_ = &core; }

  cpu12::Io callbacks() noexcept;

private:
  template <typename Action> bool access_timer(const Action &action) noexcept {
    const auto pending = timer_.pending();
    const auto deadline = timer_.deadline();
    const auto enabled = timer_.interrupts_enabled();
    if (!timer_.advance(core_->progress().estimated_cycles + idle_) || !action()) { return false; }
    if (pending != timer_.pending() || deadline != timer_.deadline() ||
        enabled != timer_.interrupts_enabled()) {
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
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      std::uint16_t word = 0;
      const auto accepted = access_timer([&] { return timer_.read({.offset = at.offset}, word); });
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

  bool write(const cpu12::ByteWrite access) noexcept {
    if (interrupted()) { return false; }
    const auto at = memory_.locate(access.address);
    if (at.space == Space::flash) {
      return flash_.write_array({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && McuFlash::contains(at.offset)) {
      return flash_.write({.offset = at.offset, .value = access.value});
    }
    if (at.space == Space::registers && Timer::contains(at.offset)) {
      return access_timer(
          [&] { return timer_.write({.offset = at.offset, .value = access.value}); });
    }
    if (at.space == Space::registers && Ports::contains(at.offset)) {
      return ports_.write({.offset = at.offset, .value = access.value});
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
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_timer([&] { return timer_.read({.offset = at.offset, .bytes = 2}, value); });
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
    if (at.space == Space::registers && Timer::contains(at.offset) &&
        Timer::contains(static_cast<std::uint16_t>(at.offset + 1))) {
      return access_timer(
          [&] { return timer_.write({.offset = at.offset, .value = access.value, .bytes = 2}); });
    }
    return write({.address = access.address,
                  .value = static_cast<std::uint8_t>(access.value >> 8U)}) &&
           write({.address = static_cast<std::uint16_t>(access.address + 1),
                  .value = static_cast<std::uint8_t>(access.value & all_bits)});
  }

  Memory &memory_;
  McuFlash flash_;
  Ports ports_;
  Timer &timer_;
  const Timer::Cycle &idle_;
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

bool interrupt(cpu12::Core &core, const Timer &timer, ExecutionReport &report) {
  const auto vector = timer.pending();
  if (!vector || !core.interrupt_ready(cpu12::InterruptKind::maskable)) { return false; }
  const auto entered = core.interrupt({.vector = *vector});
  if (entered.fault != cpu12::Fault::none) {
    report.failure = Failure{.operation = operation(entered.fault),
                             .pc = entered.pc,
                             .address = entered.address,
                             .value = entered.value,
                             .code = static_cast<std::uint32_t>(entered.fault),
                             .reason = "MCU timer interrupt entry failed"};
    return false;
  }
  if (!entered.accepted) { throw std::logic_error("Eligible MCU timer interrupt was refused"); }
  return true;
}

void advance(const cpu12::Core &core, Timer &timer, const Timer::Cycle idle) {
  const auto cycles = core.progress().estimated_cycles;
  if (idle > std::numeric_limits<Timer::Cycle>::max() - cycles || !timer.advance(cycles + idle)) {
    throw std::overflow_error("MCU device time is not representable or monotonic");
  }
}

enum class Dispatch : std::uint8_t { run, again, done };

Dispatch prepare(cpu12::Core &core, Timer &timer, Timer::Cycle &idle, ExecutionReport &report) {
  advance(core, timer, idle);
  const auto state = core.state().execution;
  if (state == cpu12::ExecutionState::stopped) { return Dispatch::done; }
  if (interrupt(core, timer, report)) {
    timer.activity(Timer::Activity::running);
    return Dispatch::again;
  }
  if (report.failure) { return Dispatch::done; }
  if (core.state().returned_frame) { core.close_return_window(); }
  if (state == cpu12::ExecutionState::waiting) {
    const auto next = timer.deadline();
    if (!next) { return Dispatch::done; }
    idle = *next - core.progress().estimated_cycles;
    return Dispatch::again;
  }
  return Dispatch::run;
}

Timer::Activity activity(const cpu12::ExecutionState state) {
  switch (state) {
    case cpu12::ExecutionState::running: return Timer::Activity::running;
    case cpu12::ExecutionState::waiting: return Timer::Activity::waiting;
    case cpu12::ExecutionState::stopped: return Timer::Activity::stopped;
    default: throw std::logic_error("Unknown MCU execution state");
  }
}
}

ExecutionReport execute(FlashImage &image, Shi &shi, const std::stop_token &stop) {
  Memory memory(image);
  Timer timer;
  Timer::Cycle idle = 0;
  auto grants = memory.regions();
  Io io(memory, image, timer, idle, shi, stop);
  auto callbacks = io.callbacks();
  cpu12::Core core(cpu12::Memory::mapped(grants), &callbacks);
  io.bind(core);
  ExecutionReport report;
  const auto reset = core.reset(reset_vector);
  if (!reset.accepted) {
    report.failure = Failure{.operation = Access::reset,
                             .pc = reset.pc,
                             .address = reset.address,
                             .value = reset.value,
                             .code = static_cast<std::uint32_t>(reset.fault),
                             .reason = "MCU reset vector access failed"};
    return report;
  }
  while (!stop.stop_requested()) {
    const auto dispatch = prepare(core, timer, idle, report);
    if (dispatch == Dispatch::done) { break; }
    if (dispatch == Dispatch::again) { continue; }
    const auto deadline = timer.deadline();
    const auto result =
        timer.interrupts_enabled() ? core.run_until(deadline.value_or(no_deadline)) : core.run();
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (result.fault != cpu12::Fault::none) {
      report.failure = Failure{.operation = operation(result.fault),
                               .pc = result.pc,
                               .address = result.address,
                               .value = result.value,
                               .code = static_cast<std::uint32_t>(result.fault),
                               .reason = "MCU access or instruction is not implemented"};
      break;
    }
    if (memory.commit_mapping()) {
      grants = memory.regions();
      core.set_memory(cpu12::Memory::mapped(grants));
    }
    advance(core, timer, idle);
    timer.activity(activity(result.state));
  }

  return report;
}
}
