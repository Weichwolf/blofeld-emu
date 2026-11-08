// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "common/masked_bus.hpp"
#include "cpu12/core.hpp"
#include "cpu12/memory.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_registers.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld {
namespace {
class ExecutionGuard {
public:
  explicit ExecutionGuard(bool &active) : active_(active) {
    if (active_) { throw Fault("Recursive HC12 execution"); }
    active_ = true;
  }

  ~ExecutionGuard() { active_ = false; }

  ExecutionGuard(const ExecutionGuard &) = delete;
  ExecutionGuard &operator=(const ExecutionGuard &) = delete;

private:
  bool &active_;
};

cpu12::InterruptKind native_kind(const InterruptKind kind) {
  switch (kind) {
    case InterruptKind::maskable: return cpu12::InterruptKind::maskable;
    case InterruptKind::xirq: return cpu12::InterruptKind::xirq;
    default: throw Fault("Unknown CPU interrupt kind");
  }
}

std::string_view native_fault(const cpu12::Fault fault) {
  switch (fault) {
    case cpu12::Fault::none: return "none";
    case cpu12::Fault::invalid_input: return "invalid-input";
    case cpu12::Fault::unsupported_instruction: return "unsupported-instruction";
    case cpu12::Fault::unsupported_host: return "unsupported-host";
    case cpu12::Fault::memory_read: return "memory-read";
    case cpu12::Fault::memory_write: return "memory-write";
    case cpu12::Fault::reentrant: return "reentrant";
    case cpu12::Fault::unsupported_state: return "unsupported-state";
    case cpu12::Fault::instruction_fetch: return "instruction-fetch";
    default: return "unknown-fault";
  }
}

std::string event_failure(const cpu12::EventResult result) {
  return "Native CPU12 event pc=" + hex(result.pc) +
         " fault=" + std::string(native_fault(result.fault)) + " address=" + hex(result.address) +
         " size=" + std::to_string(result.access_size) + " value=" + hex(result.value);
}
}

class Hc12::Impl {
public:
  explicit Impl(Bus &target)
      : bus(target),
        io{.owner = this,
           .read = read,
           .write = write,
           .read_word = read_word,
           .write_word = write_word,
           .fetch = fetch,
           .read_masked = read_masked},
        core(cpu12::Memory::external(), &io) {}

  template <bool boundary = true, typename Operation>
  bool callback(const Operation &operation, const MemoryRead access = {}) noexcept {
    if (failure) { return false; }
    try {
      const auto revision = boundary ? std::uint64_t{0} : bus.revision();
      synchronize();
      const auto registers = core.registers();
      bus.begin_instruction({.pc = registers.pc, .ccr = registers.ccr});
      operation();
      if (boundary || bus.revision() != revision || bus.read_requires_boundary(access) ||
          core.state().irq_delay != 0 || bus.interrupt_request(core.registers().ccr)) {
        core.request_yield();
      }
      return true;
    } catch (...) {
      failure = std::current_exception();
      return false;
    }
  }

  static bool read(void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback<false>(
        [&] {
          const auto result = self.bus.read_memory({.address = address});
          if (result > std::numeric_limits<std::uint8_t>::max()) {
            throw Fault("Native CPU12 byte read exceeds eight bits at " + hex(address));
          }
          value = static_cast<std::uint8_t>(result);
        },
        {.address = address});
  }

  static bool read_word(void *owner, const std::uint16_t address, std::uint16_t &value) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback<false>(
        [&] {
          const auto result = self.bus.read_memory({.address = address, .size = 2});
          if (result > std::numeric_limits<std::uint16_t>::max()) {
            throw Fault("Native CPU12 word read exceeds sixteen bits at " + hex(address));
          }
          value = static_cast<std::uint16_t>(result);
        },
        {.address = address, .size = 2});
  }

  static bool write(void *owner, const cpu12::ByteWrite access) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback(
        [&] { self.bus.write_memory({.address = access.address, .value = access.value}); });
  }

  static bool write_word(void *owner, const cpu12::WordWrite access) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback([&] {
      self.bus.write_memory({.address = access.address, .size = 2, .value = access.value});
    });
  }

  static bool
  read_masked(void *owner, const cpu12::MaskedByteRead access, std::uint8_t &value) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback<false>(
        [&] {
          value = observe_masked_byte(self.bus, {.address = access.address, .mask = access.mask});
        },
        {.address = access.address});
  }

  static bool fetch(void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback([&] {
      self.bus.check_execute({.address = address, .length = 1});
      value = self.bus.peek(address);
      self.bus.note_code_byte(address);
    });
  }

  void synchronize(const bool boundary = false) const {
    if (clock) {
      const auto registers = core.registers();
      clock(core.progress().estimated_cycles, {.pc = registers.pc, .ccr = registers.ccr}, boundary);
    }
  }

  void rethrow_callback() const {
    if (failure) { std::rethrow_exception(failure); }
  }

  Bus &bus;
  cpu12::Io io;
  cpu12::Core core;
  std::exception_ptr failure;
  std::function<void(std::uint64_t, ExecutionContext, bool)> clock;
  bool active = false;
  bool boundary_healthy = false;
  bool wait_clock_failed = false;
};

Hc12::Hc12(Bus &bus, const std::filesystem::path & /*specification*/)
    : impl_(std::make_unique<Impl>(bus)) {}

Hc12::~Hc12() = default;

void Hc12::reset(const Hc12ResetSource source) {
  if (source != Hc12ResetSource::external && source != Hc12ResetSource::cop) {
    throw Fault("Invalid HC12 reset source");
  }
  const ExecutionGuard guard(impl_->active);
  impl_->boundary_healthy = false;
  impl_->failure = nullptr;
  impl_->core.set_registers({.sp = hc12::reset_sp, .ccr = hc12::reset_ccr});
  impl_->bus.begin_reset({.ccr = hc12::reset_ccr});
  const auto vector = source == Hc12ResetSource::cop ? hc12::cop_reset_vector : hc12::reset_vector;
  const auto result = impl_->core.reset(vector);
  impl_->rethrow_callback();
  if (!result.accepted || result.fault != cpu12::Fault::none) {
    throw Fault(event_failure(result));
  }
  impl_->bus.end_reset();
  impl_->synchronize(true);
  impl_->wait_clock_failed = false;
  impl_->boundary_healthy = true;
}

void Hc12::set_state(const CpuState &state) {
  const ExecutionGuard guard(impl_->active);
  impl_->boundary_healthy = false;
  impl_->failure = nullptr;
  impl_->core.set_registers(
      {.pc = state.pc, .d = state.d, .x = state.x, .y = state.y, .sp = state.sp, .ccr = state.ccr});
  impl_->wait_clock_failed = false;
  impl_->boundary_healthy = true;
}

CpuState Hc12::state() const {
  const auto state = impl_->core.registers();
  return {
      .pc = state.pc, .d = state.d, .x = state.x, .y = state.y, .sp = state.sp, .ccr = state.ccr};
}

bool Hc12::waiting() const {
  return impl_->core.state().execution == cpu12::ExecutionState::waiting;
}

bool Hc12::executing() const {
  return impl_->active;
}

void Hc12::begin_wait_clock() {
  if (impl_->active || !impl_->boundary_healthy || impl_->wait_clock_failed || !waiting()) {
    throw Fault("HC12 wait clocks require a healthy completed WAI boundary");
  }
  impl_->active = true;
  impl_->boundary_healthy = false;
}

void Hc12::end_wait_clock(const bool healthy) {
  impl_->active = false;
  impl_->boundary_healthy = healthy;
  impl_->wait_clock_failed = !healthy;
}

std::string Hc12::disassemble() const {
  return "CPU12 opcode " + hex(impl_->bus.peek(state().pc), 2);
}

std::uint64_t Hc12::cycles() const {
  return impl_->core.progress().estimated_cycles;
}

void Hc12::bind_memory(const cpu12::Memory memory) {
  impl_->core.set_memory(memory);
}

void Hc12::set_clock_observer(std::function<void(std::uint64_t, ExecutionContext, bool)> observer) {
  if (impl_->active) { throw Fault("Cannot replace an executing MCU clock owner"); }
  impl_->clock = std::move(observer);
}

std::optional<InterruptRequest> Hc12::pending_interrupt() const {
  auto ccr = state().ccr;
  if (!impl_->core.interrupt_ready(cpu12::InterruptKind::maskable)) { ccr |= hc12::interrupt_mask; }
  if (!impl_->core.interrupt_ready(cpu12::InterruptKind::xirq)) { ccr |= hc12::xirq_mask; }
  if ((ccr & (hc12::interrupt_mask | hc12::xirq_mask)) ==
      (hc12::interrupt_mask | hc12::xirq_mask)) {
    return {};
  }
  const auto request = impl_->bus.interrupt_request(ccr);
  if (!request || !impl_->core.interrupt_ready(native_kind(request->kind))) { return {}; }
  constexpr unsigned first_maskable = 0xff00;
  constexpr unsigned last_maskable = 0xfff2;
  if (request->kind == InterruptKind::maskable &&
      ((request->vector & 1U) != 0 || request->vector < first_maskable ||
       request->vector > last_maskable)) {
    throw Fault("Invalid CPU maskable interrupt vector " + hex(request->vector));
  }
  if (request->kind == InterruptKind::xirq && request->vector != hc12::xirq_vector) {
    throw Fault("Invalid CPU XIRQ vector");
  }
  return request;
}

void Hc12::enter_interrupt(const InterruptRequest request,
                           const std::optional<std::uint16_t> reuse_stack) {
  if (impl_->core.interrupt_frame_available() != reuse_stack.has_value()) {
    throw Fault("Native CPU12 frame reuse disagrees with board interrupt sequence");
  }
  impl_->bus.begin_interrupt(request, reuse_stack.has_value());
  const auto result =
      impl_->core.interrupt({.vector = request.vector, .kind = native_kind(request.kind)});
  impl_->rethrow_callback();
  if (!result.accepted || result.fault != cpu12::Fault::none) {
    throw Fault(event_failure(result));
  }
  impl_->bus.acknowledge_interrupt(request.vector);
}

CpuStep Hc12::step() {
  return execute(std::nullopt);
}

CpuStep Hc12::run_until(const std::uint64_t cycle_deadline) {
  return execute(cycle_deadline);
}

CpuStep Hc12::execute(const std::optional<std::uint64_t> deadline) {
  if (impl_->wait_clock_failed) {
    throw Fault("HC12 wait clock failure requires coordinated recovery");
  }
  const ExecutionGuard guard(impl_->active);
  if (!impl_->boundary_healthy) {
    throw Fault("HC12 incomplete execution at " + hex(state().pc) +
                " requires coordinated recovery before stepping");
  }
  impl_->boundary_healthy = false;
  const auto before = state();
  impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
  if (const auto request = pending_interrupt()) {
    const auto reuse = impl_->core.interrupt_frame_available();
    enter_interrupt(*request, reuse ? std::optional(before.sp) : std::nullopt);
    impl_->bus.retire_instruction();
    impl_->synchronize(true);
    impl_->boundary_healthy = true;
    return {
        .instruction_executed = false, .interrupt_vector = request->vector, .stack_reused = reuse};
  }
  if (waiting()) {
    impl_->boundary_healthy = true;
    return {.instruction_executed = false};
  }
  if (impl_->core.state().execution != cpu12::ExecutionState::running) {
    throw Fault("Unqualified board STOP clock policy");
  }
  const auto result = deadline ? impl_->core.run_until(*deadline) : impl_->core.step();
  impl_->rethrow_callback();
  if (result.fault != cpu12::Fault::none) {
    impl_->synchronize();
    throw Fault("Native CPU12 instruction pc=" + hex(result.pc) + " opcode=" +
                hex(result.opcode, 2) + " fault=" + std::string(native_fault(result.fault)) +
                " address=" + hex(result.address) + " size=" + std::to_string(result.access_size) +
                " value=" + hex(result.value) + " mask=" + hex(result.read_mask, 2));
  }
  if (result.state == cpu12::ExecutionState::stopped) {
    throw Fault("Unqualified board STOP clock policy");
  }
  CpuStep outcome{.retired = result.retired, .instruction_executed = result.retired != 0};
  constexpr unsigned swi_opcode = 0x3f;
  if (result.opcode == swi_opcode) { outcome.interrupt_vector = hc12::swi_vector; }
  if (impl_->core.state().returned_frame) {
    if (const auto request = pending_interrupt()) {
      enter_interrupt(*request, before.sp);
      outcome.interrupt_vector = request->vector;
      outcome.stack_reused = true;
    }
    impl_->core.close_return_window();
  }
  impl_->bus.retire_instruction();
  impl_->synchronize(true);
  impl_->boundary_healthy = true;
  return outcome;
}
}
