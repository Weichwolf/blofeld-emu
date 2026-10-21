// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace {
using namespace blofeld;
constexpr unsigned entry = 0x8000;
constexpr unsigned stack = 0x2000;
constexpr unsigned returned = 0x9000;
constexpr unsigned xhandler = 0x9100;
constexpr unsigned xvector = 0xfff4;
constexpr unsigned x_mask = 0x40;
constexpr unsigned interrupt_masks = 0x50;
constexpr unsigned byte_values = 256;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned high_byte_mask = 0xff00;
constexpr unsigned stack_frame_bytes = 9;
constexpr unsigned reset_vector = 0xfffe;
constexpr unsigned nop = 0xa7;
constexpr CpuState initial{.pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = stack};
constexpr CpuState restored_state{
    .pc = returned, .d = 0x4219, .x = 0x1234, .y = 0x5678, .sp = stack + stack_frame_bytes};
using Registers = std::array<unsigned, 6>;

void require(const bool condition, const std::string_view message) {
  if (!condition) { throw Fault(std::string(message)); }
}

Registers registers(const CpuState state) {
  return {state.pc, state.d, state.x, state.y, state.sp, state.ccr};
}

class TestBus final : public Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  unsigned writes = 0;
  bool arm_on_stack_read = false;
  bool xirq_pending = false;

  std::uint8_t read(const std::uint16_t address) override {
    if (arm_on_stack_read && address == stack) {
      xirq_pending = true;
      arm_on_stack_read = false;
    }
    return bytes[address];
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    ++writes;
    bytes[address] = value;
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return writes; }

  void check_execute(const InstructionFetch /*fetch*/) const override {}

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    if (xirq_pending) { return InterruptRequest{.vector = xvector, .kind = InterruptKind::xirq}; }
    return {};
  }

  void acknowledge_interrupt(const std::uint16_t vector) override {
    require(xirq_pending && vector == xvector,
            "Only the actual XIRQ entry acknowledges its source");
    xirq_pending = false;
  }

  void program(const std::span<const std::uint8_t> code) {
    std::ranges::copy(code, bytes.begin() + entry);
    bytes[entry + code.size()] = nop;
    bytes[xvector] = xhandler >> 8U;
    bytes[xvector + 1] = xhandler & byte_mask;
    bytes[reset_vector] = entry >> 8U;
    bytes[reset_vector + 1] = entry & byte_mask;
  }
};

enum class Form : std::uint8_t { pulc, rti, tap, transfer_b, exchange_a, exchange_from_ccr };

struct Instruction {
  std::string_view name;
  Form form;
  std::array<std::uint8_t, 2> code;
  unsigned bytes = 1;
};

CpuState expected_state(TestBus &bus,
                        CpuState &before,
                        const Instruction &instruction,
                        const unsigned loaded) {
  auto expected = before;
  expected.pc += static_cast<std::uint16_t>(instruction.bytes);
  // Independent NXP S12CPUV2 PULC/RTI/TFR/EXG rule: only X's 0->1 is forbidden.
  auto flags = loaded;
  if ((before.ccr & x_mask) == 0) { flags &= byte_mask - x_mask; }
  expected.ccr = static_cast<std::uint8_t>(flags);
  switch (instruction.form) {
    case Form::rti: {
      constexpr std::array<std::uint8_t, 8> frame{0x19, 0x42, 0x12, 0x34, 0x56, 0x78, 0x90, 0};
      std::ranges::copy(frame, bus.bytes.begin() + stack + 1);
      expected = restored_state;
      expected.ccr = static_cast<std::uint8_t>(flags);
      break;
    }
    case Form::pulc: expected.sp = stack + 1; break;
    case Form::transfer_b:
      before.d = static_cast<std::uint16_t>((before.d & high_byte_mask) | loaded);
      expected.d = before.d;
      break;
    case Form::tap:
    case Form::exchange_a:
    case Form::exchange_from_ccr:
      before.d = static_cast<std::uint16_t>((loaded << 8U) | (before.d & byte_mask));
      expected.d = before.d;
      if (instruction.form != Form::tap) {
        expected.d = static_cast<std::uint16_t>((static_cast<unsigned>(before.ccr) << 8U) |
                                                (before.d & byte_mask));
      }
      break;
    default: throw Fault("Unsupported CCR test instruction form");
  }
  bus.bytes[stack] = static_cast<std::uint8_t>(loaded);
  return expected;
}

unsigned matrix(const Instruction instruction) {
  TestBus bus;
  bus.program(std::span(instruction.code).first(instruction.bytes));
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  unsigned mismatches = 0;
  for (unsigned old = 0; old < byte_values; ++old) {
    for (unsigned loaded = 0; loaded < byte_values; ++loaded) {
      auto before = initial;
      before.ccr = static_cast<std::uint8_t>(old);
      const auto expected = expected_state(bus, before, instruction, loaded);
      cpu.set_state(before);
      const auto step = cpu.step();
      require(step.instruction_executed && !step.interrupt_vector && bus.writes == 0,
              "A native CCR instruction completes without memory writes or invented interrupts");
      const auto actual = cpu.state();
      if (registers(actual) != registers(expected)) {
        if (mismatches == 0) {
          std::cout << instruction.name << " first old=" << hex(old, 2)
                    << " loaded=" << hex(loaded, 2) << " actual-ccr=" << hex(actual.ccr, 2)
                    << " expected-ccr=" << hex(expected.ccr, 2) << '\n';
        }
        ++mismatches;
      }
    }
  }
  std::cout << instruction.name << " cases=" << byte_values * byte_values
            << " mismatches=" << mismatches << '\n';
  return mismatches;
}

void check_xirq_after_pull(const bool rti) {
  TestBus bus;
  constexpr std::uint8_t pulc_opcode = 0x38;
  constexpr std::uint8_t rti_opcode = 0x0b;
  const Instruction instruction{.name = "IRQ",
                                .form = rti ? Form::rti : Form::pulc,
                                .code = {rti ? rti_opcode : pulc_opcode, 0}};
  bus.program(std::span(instruction.code).first(instruction.bytes));
  auto before = initial;
  const auto expected = expected_state(bus, before, instruction, x_mask);
  Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(before);
  bus.arm_on_stack_read = true;
  const auto pulled = cpu.step();
  const auto entered = rti ? pulled : cpu.step();
  require(entered.interrupt_vector == xvector && entered.stack_reused == rti &&
              entered.instruction_executed == rti && cpu.state().pc == xhandler &&
              cpu.state().ccr == interrupt_masks && !bus.xirq_pending,
          "PULC permits next-boundary XIRQ; RTI tail-chains it, and hardware entry can set X");
  const auto saved_stack = rti ? stack : stack + 1 - stack_frame_bytes;
  require(cpu.state().sp == saved_stack && bus.bytes[saved_stack] == (rti ? x_mask : expected.ccr),
          "RTI preserves its existing frame; a new entry stacks the effective unmasked CCR");
  cpu.set_state(initial);
  cpu.reset();
  constexpr unsigned reset_ccr = 0xd0;
  require(cpu.state().ccr == reset_ccr, "Hardware reset can set X again after software cleared it");
}
}

int main() {
  try {
    constexpr std::array<Instruction, 6> instructions{{
        {.name = "PULC", .form = Form::pulc, .code = {0x38, 0}},
        {.name = "RTI", .form = Form::rti, .code = {0x0b, 0}},
        {.name = "TAP", .form = Form::tap, .code = {0xb7, 2}, .bytes = 2},
        {.name = "TFR B,CCR", .form = Form::transfer_b, .code = {0xb7, 0x12}, .bytes = 2},
        {.name = "EXG A,CCR", .form = Form::exchange_a, .code = {0xb7, 0x82}, .bytes = 2},
        {.name = "EXG CCR,A", .form = Form::exchange_from_ccr, .code = {0xb7, 0xa0}, .bytes = 2},
    }};
    unsigned mismatches = 0;
    for (const auto instruction : instructions) { mismatches += matrix(instruction); }
    require(mismatches == 0, "Native CCR instructions must match every independent flag pair");
    check_xirq_after_pull(false);
    check_xirq_after_pull(true);
    std::cout << "Verified 393216 independent CCR cases and actual XIRQ/reset exceptions.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
