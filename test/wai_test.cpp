// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "mcu/hc12.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace {
constexpr unsigned entry = 0x8000;
constexpr unsigned handler = 0x9000;
constexpr unsigned xhandler = 0x9100;
constexpr unsigned sci_vector = 0xffd6;
constexpr unsigned xirq_vector = 0xfff4;
constexpr unsigned reset_vector = 0xfffe;
constexpr unsigned stack = 0x4000;
constexpr unsigned frame = stack - 9;
constexpr unsigned wai = 0x3e;
constexpr unsigned rti = 0x0b;
constexpr unsigned nop = 0xa7;
constexpr unsigned initial_flags = 0xe3;
constexpr unsigned masked_flags = 0xd0;
constexpr unsigned i_masked_flags = 0x90;
constexpr unsigned byte_mask = 0xff;

void require(const bool condition, const char *message) {
  if (!condition) { throw blofeld::Fault(message); }
}

void reject(const auto &operation) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault("Expected a visible WAI stack/vector fault");
}

struct Access {
  bool write = false;
  std::uint16_t address = 0;
};

class WaitBus final : public blofeld::Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  std::vector<Access> accesses;
  std::vector<blofeld::MemoryWrite> operands;
  std::optional<blofeld::InterruptRequest> pending;
  std::optional<std::uint16_t> fail_read;
  std::optional<std::uint16_t> fail_write;
  mutable unsigned peeks = 0;
  mutable unsigned fetches = 0;
  unsigned acknowledgements = 0;
  unsigned retirements = 0;
  std::uint64_t generation = 0;
  bool reject_fetch = false;

  WaitBus() {
    bytes[entry] = wai;
    bytes[entry + 1] = nop;
    bytes[handler] = rti;
    bytes[xhandler] = rti;
    bytes[sci_vector] = handler >> 8U;
    bytes[xirq_vector] = xhandler >> 8U;
    bytes[reset_vector] = entry >> 8U;
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (fail_read == address) { throw blofeld::Fault("Injected vector read failure"); }
    accesses.push_back({.address = address});
    return bytes[address];
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (fail_write == address) { throw blofeld::Fault("Injected WAI stack write failure"); }
    accesses.push_back({.write = true, .address = address});
    bytes[address] = value;
    ++generation;
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    ++peeks;
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return generation; }

  void write_memory(const blofeld::MemoryWrite access) override {
    operands.push_back(access);
    Bus::write_memory(access);
  }

  void check_execute(blofeld::InstructionFetch /*fetch*/) const override {
    ++fetches;
    if (reject_fetch) { throw blofeld::Fault("Injected code fetch failure"); }
  }

  [[nodiscard]] std::optional<blofeld::InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    return pending;
  }

  void acknowledge_interrupt(std::uint16_t /*vector*/) override {
    ++acknowledgements;
    pending.reset();
  }

  void retire_instruction() override { ++retirements; }
};

blofeld::CpuState initial() {
  constexpr blofeld::CpuState state{
      .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = stack, .ccr = initial_flags};
  return state;
}

std::array<std::uint64_t, 7> activity(const WaitBus &bus) {
  return {bus.accesses.size(),
          bus.operands.size(),
          bus.peeks,
          bus.fetches,
          bus.acknowledgements,
          bus.retirements,
          bus.generation};
}

void require_recovery(blofeld::Hc12 &cpu, const WaitBus &bus) {
  const auto before = cpu.state();
  const auto waiting = cpu.waiting();
  const auto saved_bytes = bus.bytes;
  const auto saved_activity = activity(bus);
  bool stopped = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &fault) {
    stopped = std::string_view(fault.what()).contains("requires coordinated recovery");
  }
  const auto after = cpu.state();
  require(stopped && before.pc == after.pc && before.d == after.d && before.x == after.x &&
              before.y == after.y && before.sp == after.sp && before.ccr == after.ccr &&
              cpu.waiting() == waiting && bus.bytes == saved_bytes &&
              activity(bus) == saved_activity,
          "A failed WAI operation refuses further stepping before any CPU or bus effect");
}

void check_restored(const blofeld::CpuState &state, const unsigned next = entry + 1) {
  const auto expected = initial();
  require(state.pc == next && state.d == expected.d && state.x == expected.x &&
              state.y == expected.y && state.sp == stack && state.ccr == expected.ccr,
          "RTI restores all original registers and resumes after WAI");
}

void test_frame_quiescence_and_wake() {
  WaitBus bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial());
  const auto entered = cpu.step();
  require(entered.instruction_executed && !entered.interrupt_vector && cpu.waiting() &&
              cpu.state().pc == entry + 1 && cpu.state().sp == frame &&
              cpu.state().ccr == initial_flags,
          "WAI executes once, preserves flags and saves its next PC before waiting");
  constexpr std::array<std::uint8_t, 9> golden{0xe3, 0x34, 0x12, 0x56, 0x78, 0x9a, 0xbc, 0x80, 1};
  require(std::ranges::equal(golden, std::span(bus.bytes).subspan(frame, golden.size())),
          "Literal independent CCR,B,A,X,Y,next-PC frame matches the NXP WAI definition");
  require(bus.operands.size() == 5 && bus.operands[0].size == 2 && bus.operands[1].size == 2 &&
              bus.operands[2].size == 2 && bus.operands[3].size == 2 && bus.operands[4].size == 1,
          "NXP WAI uses four word operands and one CCR byte, including one B:A transfer");
  constexpr std::array<unsigned, 9> order{
      0x3ffe, 0x3fff, 0x3ffc, 0x3ffd, 0x3ffa, 0x3ffb, 0x3ff8, 0x3ff9, 0x3ff7};
  require(bus.accesses.size() == order.size(), "WAI makes nine stack writes and no data reads");
  for (unsigned index = 0; index < order.size(); ++index) {
    require(bus.accesses[index].write && bus.accesses[index].address == order[index],
            "Native WAI stack bus order is independently specified");
  }
  const auto peeks = bus.peeks;
  const auto fetches = bus.fetches;
  const auto retirements = bus.retirements;
  bus.reject_fetch = true;
  for (unsigned index = 0; index < 8; ++index) {
    const auto idle = cpu.step();
    require(!idle.instruction_executed && !idle.interrupt_vector && cpu.waiting(),
            "Repeated wait observations execute no opcode");
  }
  require(bus.accesses.size() == order.size() && bus.peeks == peeks && bus.fetches == fetches &&
              bus.retirements == retirements,
          "Waiting performs no data traffic, lookahead, code fetch or instruction retirement");
  bus.accesses.clear();
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  const auto wake = cpu.step();
  require(!wake.instruction_executed && wake.interrupt_vector == sci_vector && wake.stack_reused &&
              !cpu.waiting() && cpu.state().pc == handler && cpu.state().sp == frame,
          "An eligible interrupt wakes using the already saved WAI frame");
  require(bus.accesses.size() == 2 && !bus.accesses[0].write && !bus.accesses[1].write &&
              bus.accesses[0].address == sci_vector && bus.accesses[1].address == sci_vector + 1 &&
              bus.acknowledgements == 1,
          "Wake reads only the vector; it neither fetches code nor writes a duplicate frame");
  bus.reject_fetch = false;
  cpu.step();
  check_restored(cpu.state());
}

void test_masks_and_xirq() {
  WaitBus bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = initial();
  state.ccr = i_masked_flags;
  cpu.set_state(state);
  cpu.step();
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  require(!cpu.step().instruction_executed && cpu.waiting() && bus.acknowledgements == 0,
          "A masked peripheral interrupt cannot release WAI");
  bus.pending =
      blofeld::InterruptRequest{.vector = xirq_vector, .kind = blofeld::InterruptKind::xirq};
  const auto wake = cpu.step();
  require(wake.stack_reused && wake.interrupt_vector == xirq_vector &&
              cpu.state().ccr == masked_flags && bus.bytes[frame] == i_masked_flags,
          "Unmasked XIRQ ignores I and sets X/I after reusing the original flags/frame");
  cpu.step();
  require(cpu.state().pc == entry + 1 && cpu.state().sp == stack &&
              cpu.state().ccr == i_masked_flags,
          "XIRQ RTI restores the pre-wait masks and following PC");
}

void test_cli_and_tap_protect_wai() {
  for (const bool tap : {false, true}) {
    WaitBus bus;
    constexpr std::array<std::uint8_t, 3> cli_code{0x10, 0xef, 0x3e};
    constexpr std::array<std::uint8_t, 3> tap_code{0xb7, 0x02, 0x3e};
    std::ranges::copy(tap ? tap_code : cli_code, bus.bytes.begin() + entry);
    blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    auto state = initial();
    constexpr unsigned tap_data = 0x4034;
    state.d = tap_data;
    state.ccr = masked_flags;
    cpu.set_state(state);
    bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
    require(cpu.step().instruction_executed && !cpu.pending_interrupt(),
            "The documented I-clear delay defers the pending maskable request");
    require(cpu.step().instruction_executed && cpu.waiting() && bus.acknowledgements == 0,
            "CLI/TAP lets the following real WAI stack its context first");
    const auto wake = cpu.step();
    require(wake.stack_reused && wake.interrupt_vector == sci_vector && bus.bytes[stack - 1] == 3,
            "Wake reuses the CLI/TAP-WAI frame whose return PC follows both instructions");
    cpu.step();
    constexpr unsigned after_cli = 0xc0;
    constexpr unsigned after_tap = 0x40;
    require(cpu.state().pc == entry + 3 && cpu.state().sp == stack &&
                cpu.state().ccr == (tap ? after_tap : after_cli),
            "RTI returns after the protected WAI with the actual post-unmask CCR");
  }
}

void test_eligible_request_before_wai_and_during_delay() {
  WaitBus bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial());
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  const auto before = cpu.step();
  require(!before.instruction_executed && !before.stack_reused && !cpu.waiting() &&
              bus.bytes[stack - 1] == 0,
          "A request already eligible at the boundary enters before an unexecuted WAI");
  cpu.step();
  check_restored(cpu.state(), entry);
  cpu.step();
  require(cpu.waiting(), "The resumed WAI now executes without the consumed request");
  WaitBus xbus;
  constexpr std::array<std::uint8_t, 3> code{0x10, 0xef, 0x3e};
  std::ranges::copy(code, xbus.bytes.begin() + entry);
  blofeld::Hc12 other(xbus, BLOFELD_DEFAULT_SLA);
  auto state = initial();
  state.ccr = i_masked_flags;
  other.set_state(state);
  other.step();
  xbus.pending =
      blofeld::InterruptRequest{.vector = xirq_vector, .kind = blofeld::InterruptKind::xirq};
  const auto xirq = other.step();
  require(!xirq.instruction_executed && !xirq.stack_reused && !other.waiting() &&
              other.state().pc == xhandler && xbus.bytes[stack - 1] == 2,
          "The maskable unmask delay cannot suppress an independently unmasked XIRQ");
}

void test_faults_and_vector_retry() {
  WaitBus broken;
  blofeld::Hc12 bad(broken, BLOFELD_DEFAULT_SLA);
  bad.set_state(initial());
  broken.fail_write = stack - 4;
  reject([&bad] { bad.step(); });
  require(!bad.waiting() && bad.state().sp == stack - 4 && broken.accesses.size() == 2,
          "A stack failure remains visible before wait entry and preserves partial bus effects");
  require_recovery(bad, broken);
  WaitBus bus;
  const auto initial_bus = bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial());
  cpu.step();
  const auto saved = bus.bytes;
  bus.accesses.clear();
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  bus.fail_read = sci_vector + 1;
  reject([&cpu] { cpu.step(); });
  require(cpu.waiting() && cpu.state().sp == frame && bus.acknowledgements == 0 &&
              bus.bytes == saved,
          "A failed wake vector leaves WAI/frame intact and does not acknowledge its source");
  require_recovery(cpu, bus);
  // The fixture owner restores its whole bus and re-executes the genuine WAI;
  // replacing registers alone cannot reconstruct the CPU's private wait latch.
  bus = initial_bus;
  cpu.set_state(initial());
  require(cpu.step().instruction_executed && cpu.waiting() && bus.bytes == saved,
          "Explicit owner recovery re-executes WAI and recreates the independent frame");
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  require(cpu.step().stack_reused && !cpu.waiting(),
          "A valid vector wakes the explicitly recovered WAI frame");
  cpu.step();
  check_restored(cpu.state());
}

void test_stack_wrap_reset_and_state_replacement() {
  WaitBus bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = initial();
  state.sp = 4;
  cpu.set_state(state);
  cpu.step();
  constexpr unsigned wrapped_frame = 0xfffb;
  require(cpu.waiting() && cpu.state().sp == wrapped_frame &&
              bus.bytes[wrapped_frame] == initial_flags && bus.bytes[3] == 1,
          "WAI stack arithmetic wraps at the 16-bit boundary with its real following PC");
  cpu.set_state(initial());
  require(!cpu.waiting(), "Explicit state replacement cancels wait state");
  cpu.step();
  require(cpu.waiting(), "A new WAI is independent of the abandoned frame");
  bus.bytes[reset_vector] = entry >> 8U;
  bus.bytes[reset_vector + 1] = 0;
  cpu.reset();
  require(!cpu.waiting() && cpu.state().pc == entry && cpu.state().sp == byte_mask,
          "Reset discards the wait latch and uses its actual reset vector/model");
}

void test_pc_wrap_fetch_fault_and_masked_xirq() {
  WaitBus bus;
  const auto initial_bus = bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(initial());
  bus.reject_fetch = true;
  reject([&cpu] { cpu.step(); });
  require(!cpu.waiting() && cpu.state().sp == stack && bus.accesses.empty(),
          "A rejected WAI opcode fetch cannot write a frame or enter wait");
  require_recovery(cpu, bus);
  bus = initial_bus;
  cpu.set_state(initial());
  cpu.step();
  bus.pending =
      blofeld::InterruptRequest{.vector = xirq_vector, .kind = blofeld::InterruptKind::xirq};
  require(!cpu.step().instruction_executed && cpu.waiting() && bus.acknowledgements == 0,
          "A masked XIRQ cannot release WAI even when I is clear");
  WaitBus wrapped;
  constexpr unsigned last_address = 0xffff;
  wrapped.bytes[last_address] = wai;
  blofeld::Hc12 other(wrapped, BLOFELD_DEFAULT_SLA);
  auto state = initial();
  state.pc = last_address;
  other.set_state(state);
  other.step();
  require(other.waiting() && other.state().pc == 0 && wrapped.bytes[stack - 2] == 0 &&
              wrapped.bytes[stack - 1] == 0,
          "WAI at FFFF saves and exposes the wrapped following PC 0000");
  wrapped.pending = blofeld::InterruptRequest{.vector = sci_vector};
  require(other.step().stack_reused, "Wrapped-PC wake still reuses the same frame");
  other.step();
  check_restored(other.state(), 0);
}

void test_unmask_transition_and_debug_override() {
  constexpr std::array<std::uint8_t, 3> code{0x10, 0xef, 0xa7};
  for (const bool override : {false, true}) {
    WaitBus bus;
    std::ranges::copy(code, bus.bytes.begin() + entry);
    blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    auto state = initial();
    if (override) { state.ccr = masked_flags; }
    cpu.set_state(state);
    cpu.step();
    if (override) { cpu.set_state(cpu.state()); }
    bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
    const auto accepted = cpu.step();
    require(!accepted.instruction_executed && accepted.interrupt_vector == sci_vector &&
                bus.bytes[stack - 1] == 2,
            "No delay follows an unchanged I bit or an explicit debug context replacement");
  }
}
}

int main() {
  try {
    test_frame_quiescence_and_wake();
    test_masks_and_xirq();
    test_cli_and_tap_protect_wai();
    test_eligible_request_before_wai_and_during_delay();
    test_faults_and_vector_retry();
    test_stack_wrap_reset_and_state_replacement();
    test_pc_wrap_fetch_fault_and_masked_xirq();
    test_unmask_transition_and_debug_override();
    std::cout << "Native WAI frame, idle bus, wake/RTI, I-clear protection, faults/reset checks "
                 "passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
