// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "common/masked_bus.hpp"
#include "cpu12/core.hpp"
#include "cpu12/memory.hpp"
#include "mcu/hc12.hpp"
#include "mcu/hc12_cycles.hpp"
#include "mcu/hc12_registers.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace blofeld {
namespace {
std::uint64_t inspection_identity() {
  static std::atomic<std::uint64_t> next{0};
  auto current = next.load(std::memory_order_relaxed);
  for (;;) {
    if (current == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("HC12 inspection identity exhausted");
    }
    if (next.compare_exchange_weak(current, current + 1, std::memory_order_relaxed)) {
      return current + 1;
    }
  }
}

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

std::string event_failure(const cpu12::EventResult result) {
  return "Native CPU12 event pc=" + hex(result.pc) +
         " fault=" + std::to_string(static_cast<unsigned>(result.fault)) +
         " address=" + hex(result.address) + " size=" + std::to_string(result.access_size) +
         " value=" + hex(result.value);
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

  void mark_mutation() {
    if (inspection.revision == std::numeric_limits<std::uint64_t>::max()) {
      throw Fault("HC12 inspection revision exhausted");
    }
    ++inspection.revision;
  }

  template <typename Operation> bool callback(const Operation &operation) noexcept {
    if (failure) { return false; }
    try {
      operation();
      return true;
    } catch (...) {
      failure = std::current_exception();
      return false;
    }
  }

  static bool read(void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback([&] {
      const auto result = self.bus.read_memory({.address = address});
      if (result > std::numeric_limits<std::uint8_t>::max()) {
        throw Fault("Native CPU12 byte read exceeds eight bits at " + hex(address));
      }
      value = static_cast<std::uint8_t>(result);
    });
  }

  static bool read_word(void *owner, const std::uint16_t address, std::uint16_t &value) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback([&] {
      const auto result = self.bus.read_memory({.address = address, .size = 2});
      if (result > std::numeric_limits<std::uint16_t>::max()) {
        throw Fault("Native CPU12 word read exceeds sixteen bits at " + hex(address));
      }
      value = static_cast<std::uint16_t>(result);
      if (self.ack_vector && address == *self.ack_vector) {
        self.bus.acknowledge_interrupt(address);
        self.ack_vector.reset();
      }
    });
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
    return self.callback([&] {
      value = observe_masked_byte(self.bus, {.address = access.address, .mask = access.mask});
    });
  }

  static bool fetch(void *owner, const std::uint16_t address, std::uint8_t &value) noexcept {
    auto &self = *static_cast<Impl *>(owner);
    return self.callback([&] {
      const auto expected = static_cast<std::uint16_t>(self.capture.address + self.fetched);
      if (self.fetched >= self.capture.length || address != expected) {
        throw Fault("Native CPU12 fetch disagrees with board plan at " + hex(address));
      }
      self.bus.check_execute({.address = address, .length = 1});
      value = self.bus.peek(address);
      self.bus.note_code_byte(address);
      ++self.fetched;
    });
  }

  void rethrow_callback() const {
    if (failure) { std::rethrow_exception(failure); }
  }

  Bus &bus;
  cpu12::Io io;
  cpu12::Core core;
  CpuInspectionStamp inspection{.owner = inspection_identity()};
  std::exception_ptr failure;
  std::optional<std::uint16_t> ack_vector;
  InstructionFetch capture;
  unsigned fetched = 0;
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
  impl_->mark_mutation();
  impl_->boundary_healthy = false;
  impl_->failure = nullptr;
  impl_->ack_vector.reset();
  impl_->core.set_registers({.sp = hc12::reset_sp, .ccr = hc12::reset_ccr});
  impl_->bus.begin_reset({.ccr = hc12::reset_ccr});
  const auto vector = source == Hc12ResetSource::cop ? hc12::cop_reset_vector : hc12::reset_vector;
  const auto result = impl_->core.reset(vector);
  impl_->rethrow_callback();
  if (!result.accepted || result.fault != cpu12::Fault::none) {
    throw Fault(event_failure(result));
  }
  impl_->bus.end_reset();
  impl_->wait_clock_failed = false;
  impl_->boundary_healthy = true;
}

void Hc12::set_state(const CpuState &state) {
  const ExecutionGuard guard(impl_->active);
  impl_->mark_mutation();
  impl_->boundary_healthy = false;
  impl_->failure = nullptr;
  impl_->ack_vector.reset();
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

CpuInspectionStamp Hc12::inspection_stamp() const noexcept {
  return impl_->inspection;
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
  impl_->mark_mutation();
  impl_->active = true;
  impl_->boundary_healthy = false;
}

void Hc12::end_wait_clock(const bool healthy) {
  impl_->mark_mutation();
  impl_->active = false;
  impl_->boundary_healthy = healthy;
  impl_->wait_clock_failed = !healthy;
}

std::string Hc12::disassemble() const {
  const auto before = state();
  const auto plan = hc12_cycle_plan(impl_->bus, {.pc = before.pc, .ccr = before.ccr});
  std::string result = "CPU12 bytes";
  for (unsigned offset = 0; offset < plan.bytes; ++offset) {
    result += " " + hex(impl_->bus.peek(static_cast<std::uint16_t>(before.pc + offset)), 2);
  }
  return result;
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
  impl_->ack_vector = request.vector;
  const auto result =
      impl_->core.interrupt({.vector = request.vector, .kind = native_kind(request.kind)});
  impl_->rethrow_callback();
  if (!result.accepted || result.fault != cpu12::Fault::none || impl_->ack_vector) {
    throw Fault(event_failure(result));
  }
}

CpuStep Hc12::step() {
  if (impl_->wait_clock_failed) {
    throw Fault("HC12 wait clock failure requires coordinated recovery");
  }
  const ExecutionGuard guard(impl_->active);
  if (!impl_->boundary_healthy) {
    throw Fault("HC12 incomplete execution at " + hex(state().pc) +
                " requires coordinated recovery before stepping");
  }
  impl_->mark_mutation();
  impl_->boundary_healthy = false;
  const auto before = state();
  impl_->bus.begin_instruction({.pc = before.pc, .ccr = before.ccr});
  if (const auto request = pending_interrupt()) {
    const auto reuse = impl_->core.interrupt_frame_available();
    enter_interrupt(*request, reuse ? std::optional(before.sp) : std::nullopt);
    impl_->bus.retire_instruction();
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
  impl_->bus.check_execute({.address = before.pc, .length = 1});
  const auto plan = hc12_cycle_plan(impl_->bus, {.pc = before.pc, .ccr = before.ccr});
  impl_->capture = {.address = before.pc, .length = plan.bytes};
  impl_->fetched = 0;
  impl_->bus.check_execute(impl_->capture);
  impl_->bus.begin_execution(impl_->capture);
  const auto result = impl_->core.step();
  impl_->rethrow_callback();
  if (result.fault != cpu12::Fault::none || result.retired != 1 || impl_->fetched != plan.bytes) {
    throw Fault("Native CPU12 instruction pc=" + hex(result.pc) +
                " opcode=" + hex(result.opcode, 2) +
                " fault=" + std::to_string(static_cast<unsigned>(result.fault)) +
                " address=" + hex(result.address) + " size=" + std::to_string(result.access_size) +
                " value=" + hex(result.value) + " mask=" + hex(result.read_mask, 2));
  }
  if (result.state == cpu12::ExecutionState::stopped) {
    throw Fault("Unqualified board STOP clock policy");
  }
  CpuStep outcome;
  constexpr unsigned swi_opcode = 0x3f;
  if (result.opcode == swi_opcode) { outcome.interrupt_vector = hc12::swi_vector; }
  constexpr unsigned rti_opcode = 0x0b;
  if (result.opcode == rti_opcode) {
    if (const auto request = pending_interrupt()) {
      enter_interrupt(*request, before.sp);
      outcome.interrupt_vector = request->vector;
      outcome.stack_reused = true;
    }
    impl_->core.close_return_window();
  }
  impl_->bus.retire_instruction();
  impl_->boundary_healthy = true;
  return outcome;
}
}
