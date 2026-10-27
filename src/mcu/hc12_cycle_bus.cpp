// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "common/masked_bus.hpp"
#include "mcu/hc12_cycles.hpp"

#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace blofeld {
namespace {
bool operand(const char kind) {
  return std::string_view("rRwWsSuUIV").find(kind) != std::string_view::npos;
}

bool writes(const char kind) {
  return std::string_view("wWsS").find(kind) != std::string_view::npos;
}

unsigned width(const char kind) {
  return std::string_view("RWSUIV").find(kind) != std::string_view::npos ? 2U : 1U;
}
}

Hc12CycleBus::Hc12CycleBus(Bus &bus, Clock clock, ReadTiming read_timing)
    : bus_(bus), clock_(std::move(clock)), read_timing_(std::move(read_timing)) {
  if (!clock_) { throw Fault("HC12 timed bus requires an explicit cycle clock/memory contract"); }
}

std::uint64_t Hc12CycleBus::checkpoint_cycles() const {
  healthy();
  if (active_) { throw Fault("HC12 cycle checkpoint requires a completed sequence"); }
  return cycles_;
}

void Hc12CycleBus::restore_boundary(const std::uint64_t cycles, const ExecutionContext context) {
  healthy();
  if (active_ || cycles_ != 0) { throw Fault("HC12 cycle restore requires a fresh adapter"); }
  cycles_ = cycles;
  context_ = context;
}

void Hc12CycleBus::healthy() const {
  if (faulted_) {
    throw Fault("HC12 timed bus requires coordinated recovery after a failed cycle");
  }
}

[[noreturn]] void Hc12CycleBus::fail(const std::string_view reason) const {
  faulted_ = true;
  throw Fault("HC12 cycle execution pc=" + hex(context_.pc) + " ccr=" + hex(context_.ccr, 2) +
              " completed-cycles=" + std::to_string(cycles_) + ": " + std::string(reason));
}

void Hc12CycleBus::tick(Hc12BusCycle cycle) {
  healthy();
  if (cycles_ == std::numeric_limits<std::uint64_t>::max()) { fail("Clock counter exhausted"); }
  try {
    cycle.context = context_;
    clock_(cycle);
  } catch (const std::exception &error) {
    fail("Clock/operand " + std::string(1, cycle.kind) + " address=" +
         (cycle.address ? hex(*cycle.address) : std::string("unreconstructed-prefetch/internal")) +
         ": " + error.what());
  }
  ++cycles_;
}

void Hc12CycleBus::start(const std::string_view slots) {
  healthy();
  if (active_) { fail("Previous cycle sequence is incomplete"); }
  slots_ = slots;
  next_ = 0;
  active_ = true;
}

char Hc12CycleBus::access(const MemoryRead transfer, const bool write) {
  healthy();
  if (!active_) { fail("Operand access outside a timed instruction/reset/interrupt"); }
  while (next_ < slots_.size() && !operand(slots_[next_])) { tick({.kind = slots_[next_++]}); }
  if (next_ == slots_.size() || writes(slots_[next_]) != write ||
      width(slots_[next_]) != transfer.size) {
    fail(std::string(write ? "Write" : "Read") + " address=" + hex(transfer.address) +
         " size=" + std::to_string(transfer.size) + " disagrees with native bus-cycle sequence");
  }
  return slots_[next_++];
}

std::uint64_t Hc12CycleBus::read_byte(const Hc12BusCycle cycle) {
  if (!cycle.address) { fail("Expanded byte read requires an address"); }
  tick(cycle);
  try {
    const auto value = bus_.read_memory({.address = *cycle.address});
    constexpr unsigned byte_mask = 0xff;
    if (value > byte_mask) {
      throw Fault("Expanded byte read at " + hex(*cycle.address) +
                  " returned more than eight bits");
    }
    return value;
  } catch (const std::exception &error) { fail(error.what()); }
}

std::uint64_t Hc12CycleBus::read_word_bytes(const MemoryRead transfer, const char kind) {
  constexpr unsigned last_address = 0xffff;
  if (transfer.size != 2 || transfer.address == last_address) {
    fail("Expanded word read at " + hex(transfer.address) +
         " requires two bytes within the address space");
  }
  const auto high = read_byte(
      {.kind = kind, .address = transfer.address, .size = 1, .word_part = Hc12WordPart::high});
  const auto low = read_byte({.kind = kind,
                              .address = static_cast<std::uint16_t>(transfer.address + 1U),
                              .size = 1,
                              .word_part = Hc12WordPart::low});
  return (high << 8U) | low;
}

std::uint64_t Hc12CycleBus::read_memory(const MemoryRead transfer) {
  return read_operand(transfer, std::nullopt);
}

std::uint8_t Hc12CycleBus::read_masked_byte(const MaskedByteRead access) {
  return static_cast<std::uint8_t>(read_operand({.address = access.address}, access.mask));
}

std::uint64_t Hc12CycleBus::read_operand(const MemoryRead transfer,
                                         const std::optional<std::uint8_t> mask) {
  const auto kind = access(transfer, false);
  auto timing = Hc12ReadTiming::single_cycle;
  if (read_timing_) {
    try {
      timing = read_timing_(transfer);
    } catch (const std::exception &error) {
      fail("Read timing at " + hex(transfer.address) + ": " + error.what());
    }
  }
  switch (timing) {
    case Hc12ReadTiming::byte_cycles: return read_word_bytes(transfer, kind);
    case Hc12ReadTiming::single_cycle: break;
    case Hc12ReadTiming::unqualified:
    default: fail("Unqualified operand read timing at " + hex(transfer.address));
  }
  tick({.kind = kind, .address = transfer.address, .size = transfer.size});
  try {
    return mask ? observe_masked_byte(bus_, {.address = transfer.address, .mask = *mask})
                : bus_.read_memory(transfer);
  } catch (const std::exception &error) { fail(error.what()); }
}

void Hc12CycleBus::write_memory(const MemoryWrite transfer) {
  const auto kind = access({.address = transfer.address, .size = transfer.size}, true);
  tick({.kind = kind, .address = transfer.address, .size = transfer.size});
  try {
    bus_.write_memory(transfer);
  } catch (const std::exception &error) { fail(error.what()); }
}

std::uint8_t Hc12CycleBus::read(const std::uint16_t address) {
  return static_cast<std::uint8_t>(read_memory({.address = address}));
}

void Hc12CycleBus::write(const std::uint16_t address, const std::uint8_t value) {
  write_memory({.address = address, .value = value});
}

std::uint8_t Hc12CycleBus::peek(const std::uint16_t address) const {
  return bus_.peek(address);
}

std::uint64_t Hc12CycleBus::revision() const {
  return bus_.revision();
}

std::uint64_t Hc12CycleBus::code_revision() const {
  return bus_.code_revision();
}

void Hc12CycleBus::note_code_byte(const std::uint16_t address) const {
  bus_.note_code_byte(address);
}

void Hc12CycleBus::check_execute(const InstructionFetch fetch) const {
  bus_.check_execute(fetch);
}

void Hc12CycleBus::begin_instruction(const ExecutionContext context) {
  healthy();
  if (active_) { fail("Instruction began before the preceding sequence completed"); }
  context_ = context;
  bus_.begin_instruction(context);
}

void Hc12CycleBus::begin_execution(const InstructionFetch fetch) {
  healthy();
  try {
    const auto plan = hc12_cycle_plan(bus_, context_);
    if (fetch.address != context_.pc || fetch.length != plan.bytes) {
      fail("Native instruction length disagrees with cycle plan at address=" + hex(fetch.address));
    }
    start(plan.slots);
    bus_.begin_execution(fetch);
  } catch (const std::exception &error) { fail(error.what()); }
}

void Hc12CycleBus::begin_reset(const ExecutionContext context) {
  healthy();
  if (active_) { fail("Reset requires a completed or reconstructed timed bus"); }
  context_ = context;
  bus_.begin_instruction(context);
  bus_.begin_reset(context);
  start("VfPPP");
}

void Hc12CycleBus::finish() {
  healthy();
  if (!active_) { fail("No active cycle sequence to finish"); }
  while (next_ < slots_.size()) {
    if (operand(slots_[next_])) { fail("Native execution omitted a required operand cycle"); }
    tick({.kind = slots_[next_++]});
  }
  active_ = false;
}

void Hc12CycleBus::end_reset() {
  finish();
  bus_.end_reset();
}

void Hc12CycleBus::begin_interrupt(const InterruptRequest request, const bool reuse_stack) {
  if (active_) {
    if (!reuse_stack || slots_ != "uUUUUPPP" || next_ != 5) {
      fail("Interrupt began during an incomplete instruction");
    }
    active_ = false;
  }
  start(reuse_stack ? "fVfPPP" : "VSPSSPSsP");
  bus_.begin_interrupt(request, reuse_stack);
}

void Hc12CycleBus::retire_instruction() {
  finish();
  bus_.retire_instruction();
}

std::optional<InterruptRequest> Hc12CycleBus::interrupt_request(const std::uint8_t ccr) const {
  healthy();
  try {
    return bus_.interrupt_request(ccr);
  } catch (const std::exception &error) {
    fail("Interrupt selection: " + std::string(error.what()));
  }
}

void Hc12CycleBus::acknowledge_interrupt(const std::uint16_t vector) {
  healthy();
  bus_.acknowledge_interrupt(vector);
}

void Hc12CycleBus::advance_idle(const unsigned cycles) {
  healthy();
  if (active_) { fail("Idle clocks supplied during an executing instruction"); }
  constexpr unsigned maximum_cycles = 1048576;
  if (cycles > maximum_cycles) { fail("Idle clock slice exceeds cycle limit"); }
  for (unsigned cycle = 0; cycle < cycles; ++cycle) { tick({}); }
}
}
