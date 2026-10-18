// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hc12.hpp"
#include "blofeld/mcu_bus.hpp"
#include "blofeld/mcu_gpio.hpp"
#include "blofeld/mcu_interrupts.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace {
// Independent expectations: S12CPUV2 sections 7.5/7.8; S12INTV1 HPRIO;
// S12MEBIV3 IRQCR; 9S12UF32DGV1 table 5-1. All program bytes are synthetic.
constexpr unsigned nop = 0xa7;
constexpr unsigned swi = 0x3f;
constexpr unsigned prefix18 = 0x18;
constexpr unsigned stop = 0x3e;
constexpr unsigned entry = 0x8000;
constexpr unsigned handler = 0x9000;
constexpr unsigned xhandler = 0x9100;
constexpr unsigned shandler = 0x9200;
constexpr unsigned sci_vector = 0xffd6;
constexpr unsigned timer_vector = 0xffee;
constexpr unsigned irq_vector = 0xfff2;
constexpr unsigned xirq_vector = 0xfff4;
constexpr unsigned swi_vector = 0xfff6;
constexpr unsigned stack_top = 0x4000;
constexpr unsigned stack_frame = 0x3ff7;
constexpr unsigned original_flags = 0xe3;
constexpr unsigned handler_flags = 0xf3;
constexpr unsigned all_masked = 0xd0;
constexpr unsigned x_only_masked = 0x40;
constexpr unsigned byte_mask = 0xff;
constexpr unsigned interrupt_priority = 0x1f;
constexpr unsigned irq_control = 0x1e;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::uint16_t requested_vector(const std::optional<blofeld::InterruptRequest> request) {
  if (!request) { throw blofeld::Fault("Expected an interrupt request"); }
  return request->vector;
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

struct Access {
  bool write = false;
  std::uint16_t address = 0;
  std::uint8_t value = 0;
};

class InterruptBus final : public blofeld::Bus {
public:
  std::array<std::uint8_t, 65536> bytes{};
  std::vector<Access> accesses;
  std::optional<blofeld::InterruptRequest> pending;
  std::optional<std::uint16_t> fail_write;
  bool clear_on_ack = true;
  unsigned acknowledgements = 0;
  std::uint64_t generation = 0;

  InterruptBus() {
    bytes[entry] = nop;
    bytes[handler] = 0x0b;
    bytes[xhandler] = 0x0b;
    bytes[shandler] = 0x0b;
    bytes[sci_vector] = handler >> 8U;
    bytes[sci_vector + 1] = 0;
    bytes[xirq_vector] = xhandler >> 8U;
    bytes[xirq_vector + 1] = 0;
    bytes[swi_vector] = shandler >> 8U;
    bytes[swi_vector + 1] = 0;
  }

  std::uint8_t read(const std::uint16_t address) override {
    const auto value = bytes[address];
    accesses.push_back({.address = address, .value = value});
    return value;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    if (fail_write == address) { throw blofeld::Fault("Injected interrupt stack write failure"); }
    accesses.push_back({.write = true, .address = address, .value = value});
    bytes[address] = value;
    ++generation;
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return bytes[address];
  }

  [[nodiscard]] std::uint64_t revision() const override { return generation; }

  void check_execute(blofeld::InstructionFetch /*fetch*/) const override {}

  [[nodiscard]] std::optional<blofeld::InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    return pending;
  }

  void acknowledge_interrupt(std::uint16_t /*vector*/) override {
    ++acknowledgements;
    if (clear_on_ack) { pending.reset(); }
  }
};

blofeld::CpuState original_state() {
  constexpr blofeld::CpuState original{
      .pc = entry, .d = 0x1234, .x = 0x5678, .y = 0x9abc, .sp = stack_top, .ccr = original_flags};
  return original;
}

void verify_original_state(const blofeld::CpuState actual) {
  const auto original = original_state();
  require(actual.pc == original.pc && actual.d == original.d && actual.x == original.x &&
              actual.y == original.y && actual.sp == original.sp && actual.ccr == original.ccr,
          "RTI restores the complete interrupted CPU context");
}

void test_entry_frame_and_return() {
  InterruptBus bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(original_state());
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  require(requested_vector(cpu.pending_interrupt()) == sci_vector && bus.acknowledgements == 0,
          "Peeking at an interrupt does not acknowledge or consume it");
  const auto step = cpu.step();
  require(!step.instruction_executed && step.interrupt_vector == sci_vector && !step.stack_reused,
          "Interrupt entry is a distinct machine event, not an executed opcode");
  require(cpu.state().pc == handler && cpu.state().sp == stack_frame &&
              cpu.state().ccr == handler_flags && cpu.state().d == original_state().d,
          "Maskable entry updates PC/SP/I while preserving the data registers");
  constexpr std::array<std::uint8_t, 9> golden{0xe3, 0x34, 0x12, 0x56, 0x78, 0x9a, 0xbc, 0x80, 0};
  for (unsigned index = 0; index < golden.size(); ++index) {
    require(bus.bytes[stack_frame + index] == golden[index], "Literal CCR,B,A,X,Y,PC stack frame");
  }
  constexpr std::array<unsigned, 11> addresses{
      0xffd6, 0xffd7, 0x3ffe, 0x3fff, 0x3ffc, 0x3ffd, 0x3ffa, 0x3ffb, 0x3ff8, 0x3ff9, 0x3ff7};
  require(bus.accesses.size() == addresses.size(), "Vector reads and nine stack writes only");
  for (unsigned index = 0; index < addresses.size(); ++index) {
    require(bus.accesses[index].address == addresses[index] &&
                bus.accesses[index].write == (index >= 2),
            "Vector is fetched before stack traffic, with no synthetic prereads");
  }
  const auto returned = cpu.step();
  require(returned.instruction_executed && !returned.interrupt_vector,
          "RTI with no pending request returns normally");
  verify_original_state(cpu.state());
}

void test_masks_and_xirq() {
  InterruptBus bus;
  constexpr std::array<std::uint8_t, 3> code{0x10, 0xef, 0xa7}; // ANDCC #EF; NOP.
  std::ranges::copy(code, bus.bytes.begin() + entry);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = original_state();
  state.ccr = all_masked;
  cpu.set_state(state);
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  const auto unmask = cpu.step();
  require(unmask.instruction_executed && !unmask.interrupt_vector && bus.acknowledgements == 0,
          "Masked IRQ waits while the real ANDCC instruction executes");
  const auto accepted = cpu.step();
  require(!accepted.instruction_executed && cpu.state().pc == handler,
          "IRQ is recognized before the next unexecuted instruction");
  InterruptBus xbus;
  blofeld::Hc12 xcpu(xbus, BLOFELD_DEFAULT_SLA);
  state = original_state();
  constexpr unsigned x_initial_flags = 0x93;
  state.ccr = x_initial_flags;
  xcpu.set_state(state);
  xbus.pending =
      blofeld::InterruptRequest{.vector = xirq_vector, .kind = blofeld::InterruptKind::xirq};
  const auto xstep = xcpu.step();
  constexpr unsigned x_handler_flags = 0xd3;
  require(!xstep.instruction_executed && xcpu.state().pc == xhandler &&
              xcpu.state().ccr == x_handler_flags && xbus.bytes[stack_frame] == state.ccr,
          "XIRQ ignores I, stacks the old CCR, then sets both X and I");
  xcpu.step();
  require(xcpu.state().ccr == state.ccr && xcpu.state().pc == entry,
          "RTI restores XIRQ masks from the frame");
}

void test_tail_chain_and_vector_before_stack() {
  InterruptBus bus;
  bus.clear_on_ack = false;
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(original_state());
  cpu.step();
  bus.accesses.clear();
  const auto chained = cpu.step();
  require(chained.instruction_executed && chained.stack_reused &&
              chained.interrupt_vector == sci_vector && cpu.state().pc == handler &&
              cpu.state().sp == stack_frame,
          "RTI reuses its existing frame for a pending interrupt");
  constexpr unsigned read_count = 11;
  require(bus.accesses.size() == read_count &&
              std::ranges::none_of(bus.accesses, [](const Access access) { return access.write; }),
          "Tail chaining reads the restored frame and new vector without rewriting memory");
  bus.pending.reset();
  cpu.step();
  verify_original_state(cpu.state());
  InterruptBus overlap;
  blofeld::Hc12 other(overlap, BLOFELD_DEFAULT_SLA);
  auto state = original_state();
  state.sp = sci_vector + 2;
  other.set_state(state);
  overlap.pending = blofeld::InterruptRequest{.vector = sci_vector};
  other.step();
  require(other.state().pc == handler && overlap.bytes[sci_vector] == 0x80,
          "The vector is latched before an overlapping stack overwrites it");
}

void test_stack_fault_and_invalid_vector() {
  InterruptBus bus;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  cpu.set_state(original_state());
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector};
  constexpr unsigned failed_stack = 0x3ffa;
  bus.fail_write = failed_stack;
  reject([&cpu] { cpu.step(); }, "Stack write failures must remain visible");
  require(cpu.state().sp == failed_stack && cpu.state().pc == entry &&
              cpu.state().ccr == original_flags,
          "Failed entry retains its partially advanced SP without falsely entering the handler");
  bus.pending = blofeld::InterruptRequest{.vector = sci_vector + 1};
  reject([&cpu] { static_cast<void>(cpu.pending_interrupt()); }, "Odd interrupt vectors fail");
}

void test_priority_and_external_pins() {
  blofeld::McuInterrupts interrupts;
  constexpr std::array<std::uint16_t, 2> pending{0xffd6, 0xffee};
  require(!interrupts.select(pending, all_masked), "Global masks suppress all sources");
  reject([&interrupts] { static_cast<void>(interrupts.select({}, x_only_masked)); },
         "An unmodeled external IRQ level cannot be assumed inactive");
  interrupts.observe_pins({.known = 3, .levels = 3});
  require(requested_vector(interrupts.select(pending, x_only_masked)) == timer_vector,
          "Default priority follows vector address");
  constexpr unsigned promote_sci = 0xd6;
  interrupts.write({.address = interrupt_priority, .value = promote_sci}, all_masked);
  require(requested_vector(interrupts.select(pending, x_only_masked)) == sci_vector,
          "HPRIO promotes SCI above timer");
  constexpr unsigned promote_timer = 0xee;
  interrupts.write({.address = interrupt_priority, .value = promote_timer}, x_only_masked);
  require(interrupts.read(interrupt_priority) == promote_sci, "HPRIO writes require I=1");
  interrupts.write({.address = interrupt_priority, .value = byte_mask}, all_masked);
  interrupts.observe_pins({.known = 3, .levels = 1});
  require(requested_vector(interrupts.select(pending, x_only_masked)) == irq_vector,
          "Invalid HPRIO selects default IRQ priority");
  interrupts.observe_pins({.known = 3, .levels = 0});
  require(requested_vector(interrupts.select(pending, 0)) == xirq_vector,
          "XIRQ outranks maskable sources");
  interrupts.reset();
  interrupts.observe_pins({.known = 3, .levels = 3});
  constexpr unsigned edge_enabled = 0xc0;
  interrupts.write({.address = irq_control, .value = edge_enabled}, all_masked);
  interrupts.observe_pins({.known = 3, .levels = 1});
  interrupts.observe_pins({.known = 3, .levels = 3});
  require(requested_vector(interrupts.select({}, x_only_masked)) == irq_vector,
          "Falling IRQ edge remains latched after the pad rises");
  interrupts.acknowledge(irq_vector);
  require(!interrupts.select({}, x_only_masked), "Servicing IRQ clears only its edge latch");
  interrupts.write({.address = irq_control, .value = 0}, all_masked);
  require(interrupts.read(irq_control) == 0x80, "IRQE is write-once while IRQEN remains writable");
}

std::array<std::uint8_t, 32768> serial_firmware() {
  std::array<std::uint8_t, 32768> flash{};
  flash[0] = nop;
  constexpr unsigned vector_offset = sci_vector - entry;
  flash[vector_offset] = handler >> 8U;
  flash[vector_offset + 1] = 0;
  // Own ISR: LDAA SCISR1; LDAA SCIDRL; STAA $1200; RTI.
  constexpr std::array<std::uint8_t, 8> code{0x96, 0xcc, 0x96, 0xcf, 0x7a, 0x12, 0x00, 0x0b};
  std::ranges::copy(code, flash.begin() + (handler - entry));
  return flash;
}

void test_serial_waveform_to_cpu_isr() {
  const auto flash = serial_firmware();
  blofeld::McuBus bus(flash);
  bus.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
  constexpr unsigned baud_high = 0xc8;
  constexpr unsigned baud_low = 0xc9;
  constexpr unsigned control = 0xcb;
  constexpr unsigned data = 0xcf;
  constexpr unsigned receive_interrupt = 0x24;
  constexpr unsigned note_on = 0x90;
  constexpr unsigned mailbox = 0x1200;
  bus.write(baud_high, 0);
  bus.write(baud_low, 1);
  bus.write(control, receive_interrupt);
  bus.write(mailbox, 0);
  const auto line = [&bus](const bool high, const unsigned cycles) {
    bus.set_pin_drive(blofeld::McuPort::r,
                      {.mask = 4, .levels = static_cast<std::uint8_t>(high ? 4 : 0)});
    bus.advance_sci(cycles);
  };
  line(true, 3);
  line(false, 16);
  for (unsigned bit = 0; bit < 8; ++bit) { line(((note_on >> bit) & 1U) != 0, 16); }
  line(true, 16);
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto state = original_state();
  state.sp = 0x2000;
  state.ccr = x_only_masked;
  cpu.set_state(state);
  require(requested_vector(cpu.pending_interrupt()) == sci_vector,
          "A complete serial frame asserts the SCI vector");
  require(bus.read(data) == note_on && requested_vector(cpu.pending_interrupt()) == sci_vector,
          "Interrupt polling does not impersonate an SCISR1 read to acknowledge receive data");
  const auto accepted = cpu.step();
  require(!accepted.instruction_executed && accepted.interrupt_vector == sci_vector,
          "CPU services the peripheral request at an instruction boundary");
  cpu.step();
  cpu.step();
  cpu.step();
  const auto returned = cpu.step();
  require(!returned.interrupt_vector && bus.read(mailbox) == note_on && cpu.state().pc == entry &&
              cpu.state().sp == state.sp && cpu.state().d == state.d &&
              cpu.state().ccr == state.ccr,
          "Real ISR instructions acknowledge SCI, store the MIDI byte, and restore context through "
          "RTI");
  require(!cpu.pending_interrupt(), "The acknowledged SCI flag deasserts its interrupt line");
}

void test_software_interrupt_and_stop() {
  InterruptBus bus;
  bus.bytes[entry] = swi;
  blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
  auto masked = original_state();
  masked.ccr = all_masked;
  cpu.set_state(masked);
  const auto step = cpu.step();
  require(step.instruction_executed && step.interrupt_vector == swi_vector &&
              cpu.state().pc == shandler,
          "SWI reads the vector contents rather than executing the vector address");
  require(!bus.accesses.front().write && bus.accesses.front().address == swi_vector,
          "SWI fetches its exception vector before stacking registers");
  require(bus.bytes[stack_frame] == all_masked && bus.bytes[stack_top - 1] == 1,
          "SWI ignores global masks and saves its following instruction address");
  cpu.step();
  require(cpu.state().pc == entry + 1 && cpu.state().sp == stack_top &&
              cpu.state().ccr == all_masked && cpu.state().d == masked.d,
          "RTI restores the masked software-interrupt context");
  InterruptBus stopped;
  stopped.bytes[entry] = prefix18;
  stopped.bytes[entry + 1] = stop;
  blofeld::Hc12 other(stopped, BLOFELD_DEFAULT_SLA);
  other.set_state(original_state()); // S=1 disables STOP.
  other.step();
  require(other.state().pc == entry + 2 && other.state().sp == stack_top &&
              stopped.accesses.empty(),
          "Disabled STOP behaves as a two-byte NOP without stacking");
  auto state = original_state();
  state.ccr = 0;
  other.set_state(state);
  reject([&other] { other.step(); },
         "Enabled STOP remains visibly unsupported until clock/wakeup modeling");
  require(other.state().sp == stack_frame && stopped.accesses.size() == 9,
          "Enabled STOP stacks its real context before unsupported clock suspension fails");
}

void test_timer_vectors_and_isr_acknowledgment() {
  constexpr unsigned tios = 0x40;
  constexpr unsigned tscr1 = 0x46;
  constexpr unsigned tie = 0x4c;
  constexpr unsigned tflags1 = 0x4e;
  constexpr unsigned channel0 = 0x50;
  for (unsigned channel = 0; channel < 8; ++channel) {
    const auto bit = static_cast<std::uint8_t>(1U << channel);
    const auto vector = timer_vector - (channel * 2);
    auto flash = serial_firmware();
    flash[vector - entry] = handler >> 8U;
    flash[vector - entry + 1] = 0;
    // Own ISR: LDAA #channel-mask; STAA TFLG1; RTI.
    const std::array<std::uint8_t, 5> code{0x86, bit, 0x5a, tflags1, 0x0b};
    std::ranges::copy(code, flash.begin() + (handler - entry));
    blofeld::McuBus bus(flash);
    bus.set_pin_drive(blofeld::McuPort::e, {.mask = 3, .levels = 3});
    bus.write(tios, bit);
    bus.write(static_cast<std::uint16_t>(channel0 + (channel * 2)), 0);
    bus.write(static_cast<std::uint16_t>(channel0 + (channel * 2) + 1), 1);
    bus.write(tie, bit);
    bus.write(tscr1, 0x80);
    bus.advance_timer(1);
    blofeld::Hc12 cpu(bus, BLOFELD_DEFAULT_SLA);
    auto state = original_state();
    state.sp = 0x2000;
    state.ccr = x_only_masked;
    cpu.set_state(state);
    require(requested_vector(cpu.pending_interrupt()) == vector,
            "Each timer channel routes its literal device vector");
    const auto accepted = cpu.step();
    require(!accepted.instruction_executed && accepted.interrupt_vector == vector &&
                bus.read(tflags1) == bit,
            "CPU entry does not silently clear the timer channel flag");
    cpu.step();
    cpu.step();
    const auto returned = cpu.step();
    require(!returned.interrupt_vector && bus.read(tflags1) == 0 && cpu.state().pc == entry &&
                cpu.state().sp == state.sp && cpu.state().d == state.d,
            "Real timer ISR clears its source with W1C and RTI restores context");
  }
}
}

int main() {
  try {
    test_entry_frame_and_return();
    test_masks_and_xirq();
    test_tail_chain_and_vector_before_stack();
    test_stack_fault_and_invalid_vector();
    test_priority_and_external_pins();
    test_serial_waveform_to_cpu_isr();
    test_software_interrupt_and_stop();
    test_timer_vectors_and_isr_acknowledgment();
    std::cout << "CPU interrupt frames, priority, RTI chaining and SCI ISR checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
