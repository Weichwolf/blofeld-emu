// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_known_memory.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned vector = 0xffe0, handler = 0x9000, stack = 0x240, saved_pc = 0x1234;
constexpr Hcs08State foreground{.pc = saved_pc, .hx = 0xbc34, .sp = stack, .a = 0x5a, .ccr = 0x61};
constexpr unsigned handler_high = 0x90;
constexpr unsigned irq_mask = 8, first_vector = 0xffc0, last_vector = 0xfffa;
constexpr unsigned entry_flags = 0x69, saved_flags = 0x61, reset_known = 0x68;
constexpr unsigned high_mask = 0xff00, isr_high = 0x1200, partial_ccr = 0xef;
constexpr unsigned vector_stack_cases = 240;

class IrqMemory final : public MaskedMemory {
public:
  unsigned entries = 0, acknowledgements = 0, operation = 0;
  std::optional<unsigned> stop, return_stop;
  unsigned return_reads = 0;
  std::function<void()> on_entry, on_retire;
  mutable std::vector<std::uint8_t> queries;

  IrqMemory() {
    writable.set();
    constexpr std::array<std::uint8_t, 2> target{handler_high, 0};
    constexpr std::array<std::uint8_t, 1> code{0x80}; // Own RTI.
    install(vector, target);
    install(handler, code);
    interrupt = {.vector = vector, .kind = InterruptKind::maskable};
  }

  void event(const unsigned address, const char *name) {
    const auto index = operation++;
    if (stop == index) {
      throw Fault(std::string("Directed IRQ ") + name + " address=" + hex(address) +
                  " pc=" + hex(context.pc));
    }
  }

  void begin_interrupt(const InterruptRequest request, const bool reuse) override {
    event(request.vector, "begin");
    require(request.kind == InterruptKind::maskable && !reuse,
            "Normal IRQ is a new hardware frame, never a WAIT frame reuse");
    ++entries;
    if (on_entry) { on_entry(); }
  }

  void retire_instruction() override {
    if (on_retire) { on_retire(); }
    Memory::retire_instruction();
  }

  void acknowledge_interrupt(const std::uint16_t address) override {
    event(address, "acknowledge");
    require(interrupt && interrupt->vector == address, "Acknowledge actual selected vector");
    ++acknowledgements; // Actual source remains asserted until its ISR clears it.
  }

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(const std::uint8_t ccr) const override {
    queries.push_back(ccr);
    return interrupt;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    event(address, "write");
    Memory::write(address, value);
  }

  void write_known_byte(const ByteWrite access, const std::uint8_t known_bits) override {
    require(known_bits != byte_mask, "Partial frame uses its actual mask");
    event(access.address, "masked write");
    Memory::write_known_byte(access, known_bits);
  }

  Hcs08KnownByte read_known_byte(const std::uint16_t address) override {
    const auto index = return_reads++;
    if (return_stop == index) { fail(address, "directed RTI read"); }
    return Memory::read_known_byte(address);
  }

  std::uint8_t read(const std::uint16_t address) override {
    if (interrupt && (address == interrupt->vector || address == interrupt->vector + 1U)) {
      event(address, "vector read");
    }
    return Memory::read(address);
  }
};

std::vector<Access> frame(const Hcs08State s) {
  return {{.address = s.sp, .value = s.pc & byte_mask, .write = true},
          {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 1U),
           .value = static_cast<unsigned>(s.pc) >> 8U,
           .write = true},
          {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 2U),
           .value = s.hx & byte_mask,
           .write = true,
           .known = static_cast<std::uint8_t>(s.hx_known)},
          {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 3U),
           .value = s.a,
           .write = true,
           .known = s.a_known},
          {.address = static_cast<std::uint16_t>(s.sp + space_bytes - 4U),
           .value = s.ccr,
           .write = true,
           .known = s.ccr_known}};
}

void entered(const IrqMemory &memory, const Hcs08 &cpu, const Hcs08State before) {
  auto expected = before;
  expected.pc = handler;
  expected.sp = static_cast<std::uint16_t>(before.sp + space_bytes - 5U);
  expected.ccr |= irq_mask;
  auto accesses = frame(before);
  accesses.push_back({.address = vector, .value = handler_high, .write = false});
  accesses.push_back({.address = vector + 1, .value = 0, .write = false});
  require(cpu.state() == expected && memory.accesses == accesses && memory.entries == 1 &&
              memory.acknowledgements == 1 && memory.retired == 0 && !cpu.faulted(),
          "Independent PCL/PCH/X/A/CCR write order, set-I, vector reads and no retirement");
}

void vectors_and_wrap() {
  IrqMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  constexpr std::array<unsigned, 8> stacks{0, 1, 2, 3, 4, 5, stack, word_mask};
  for (unsigned address = first_vector; address <= last_vector; address += 2) {
    constexpr std::array<std::uint8_t, 2> target{handler_high, 0};
    memory.install(address, target);
    memory.interrupt = {.vector = static_cast<std::uint16_t>(address)};
    for (const auto sp : stacks) {
      memory.install(address, target);
      auto initial = foreground;
      initial.sp = static_cast<std::uint16_t>(sp);
      cpu.set_state(initial);
      memory.accesses.clear();
      const auto retired = memory.retired;
      require(cpu.pending_interrupt() == address && memory.accesses.empty(),
              "Pending query does not fetch a vector, push a frame or acknowledge");
      require(cpu.service_interrupt(), "Every even architecture hardware vector is admitted");
      auto expected = initial;
      // A wrapping RAM stack may really overwrite a RAM-backed vector. Do not
      // retain a stale target merely because it was installed before entry.
      const unsigned low = address == last_vector && sp == word_mask ? saved_flags : 0;
      expected.pc = static_cast<std::uint16_t>(handler | low);
      expected.sp = static_cast<std::uint16_t>(sp + space_bytes - 5U);
      expected.ccr = entry_flags;
      auto accesses = frame(initial);
      accesses.push_back({.address = address, .value = handler_high, .write = false});
      accesses.push_back({.address = address + 1, .value = low, .write = false});
      require(cpu.state() == expected && memory.accesses == accesses && memory.retired == retired,
              "All vector slots and16-bit stack wraps have independent actual access order");
      constexpr std::array<std::uint8_t, 1> rti{0x80};
      memory.install(expected.pc, rti);
      cpu.step();
      require(cpu.state() == initial && memory.retired == retired + 1,
              "Native RTI restores actual frame across all tested wrap positions");
    }
  }
  require(memory.entries == vector_stack_cases && memory.acknowledgements == vector_stack_cases,
          "Thirty architecture slots times eight stack positions, without source flag clearing");
}

struct SavedMasks {
  unsigned a, x, ccr;
};

void masked_case(const SavedMasks masks) {
  IrqMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  auto supplied = foreground;
  supplied.a_known = static_cast<std::uint8_t>(masks.a);
  supplied.hx_known = static_cast<std::uint16_t>(high_mask | masks.x);
  supplied.ccr_known = static_cast<std::uint8_t>(masks.ccr);
  cpu.set_state(supplied);
  const auto before = cpu.state();
  require(cpu.service_interrupt(), "Partial A/X/CCR still permit known-I hardware IRQ");
  entered(memory, cpu, before);
  const auto checkpoint = cpu.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = memory;
    Hcs08 other(restored, BLOFELD_HCS08_SLA);
    other.restore_checkpoint(checkpoint);
    constexpr std::array<std::uint8_t, 6> body{0xa6, 0x42, 0x45, 0x12, 0x34, 0x80};
    restored.install(handler, body); // Own LDA #42; LDHX #1234; RTI.
    auto expected = before;
    expected.hx = static_cast<std::uint16_t>(isr_high | (before.hx & byte_mask));
    bool observed = false;
    restored.on_retire = [&] {
      if (other.state().pc != saved_pc) { return; }
      require(other.state() == expected &&
                  rejected([&] { (void)other.checkpoint(); }).contains("completed native boundary"),
              "RTI retirement sees restored register values and all actual masks together");
      observed = true;
      if (replay == 1) { throw Fault("Own RTI retirement refusal"); }
    };
    other.step();
    other.step();
    if (replay == 0) {
      other.step();
    } else {
      require(rejected([&] { other.step(); }).contains("Own RTI retirement refusal") &&
                  other.faulted() &&
                  rejected([&] { other.step(); }).contains("coordinated recovery"),
              "Failing retirement hook retains restored masks and quarantines the owner");
    }
    require(observed && other.state() == expected && restored.retired == (replay == 0 ? 3U : 2U) &&
                restored.acknowledgements == 1,
            "Fresh joint RAM/CPU restore preserves frame masks; RTI restores A/X/CCR "
            "knowledge while H retains the actual ISR value");
  }
  cpu.step();
  require(cpu.state() == before, "Native RTI copies genuine partial bytes without defining bits");
}

void masked_frames() {
  constexpr std::array<unsigned, 3> a_masks{0, 0x55, 0xff};
  constexpr std::array<unsigned, 3> x_masks{0, 0xaa, 0xff};
  constexpr std::array<unsigned, 3> ccr_masks{reset_known, partial_ccr, 0xff};
  for (const auto a_mask : a_masks) {
    for (const auto x_mask : x_masks) {
      for (const auto ccr_mask : ccr_masks) {
        masked_case({.a = a_mask, .x = x_mask, .ccr = ccr_mask});
      }
    }
  }
}

void masking_and_invalid() {
  IrqMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  auto s = foreground;
  s.ccr |= irq_mask;
  cpu.set_state(s);
  require(!cpu.pending_interrupt() && !cpu.service_interrupt() && memory.accesses.empty() &&
              memory.entries == 0 && cpu.state() == s && memory.queries.back() == saved_flags,
          "Known I masks a real source without claiming or clearing it");
  s.ccr_known = flags_fixed;
  cpu.set_state(s);
  const auto before = cpu.state();
  require(rejected([&] { (void)cpu.service_interrupt(); }).contains("unknown CPU I") &&
              cpu.faulted() && cpu.state() == before && memory.accesses.empty(),
          "Pending source cannot consume an unknown global mask");
  memory.interrupt.reset();
  constexpr std::array<std::uint8_t, 1> code{0x9b}; // Own SEI defines I without consuming it.
  memory.install(saved_pc, code);
  cpu.set_state(s);
  require(!cpu.service_interrupt(), "No request does not require an invented I value");
  cpu.step();
  require(cpu.state().ccr_known == reset_known && cpu.state().ccr == reset_known &&
              memory.retired == 1,
          "Actual SEI can establish a previously unknown I bit");
  constexpr std::array<unsigned, 7> invalid_vectors{
      0, 0xffbf, 0xffc1, 0xfffb, 0xfffc, 0xfffe, 0xffff};
  for (const auto address : invalid_vectors) {
    memory.interrupt = {.vector = static_cast<std::uint16_t>(address)};
    cpu.set_state(foreground);
    require(rejected([&] { (void)cpu.service_interrupt(); }).contains("invalid maskable vector") &&
                memory.accesses.empty() && memory.entries == 0,
            "Odd, non-vector, SWI and reset locations fail before a stack or vector effect");
  }
  memory.interrupt = {.vector = vector, .kind = InterruptKind::xirq};
  cpu.set_state(foreground);
  require(rejected([&] { (void)cpu.service_interrupt(); }).contains("interrupt kind"),
          "HCS08 does not accept another architecture's nonmaskable XIRQ");
  memory.interrupt = {.vector = vector};
  auto absent_pc = foreground;
  absent_pc.pc_known = false;
  cpu.set_state(absent_pc);
  require(rejected([&] { (void)cpu.service_interrupt(); }).contains("unknown stacked CPU PC") &&
              memory.accesses.empty(),
          "An eligible IRQ cannot push an invented numerical return PC");
  cpu.set_state(foreground);
  require(rejected([&] { cpu.step(); }).contains("service required") && memory.accesses.empty(),
          "Instruction stepping cannot silently mischarge an IRQ as an ordinary opcode");
}

void failures() {
  for (unsigned cut = 0; cut < 9; ++cut) {
    IrqMemory memory;
    memory.stop = cut;
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    cpu.set_state(foreground);
    const auto error = rejected([&] { (void)cpu.service_interrupt(); });
    const auto pushes = cut == 0 ? 0 : std::min(cut - 1, 5U);
    const auto reads = cut <= 6 ? 0 : cut - 6;
    auto accesses = frame(foreground);
    accesses.resize(pushes);
    if (reads > 0) {
      accesses.push_back({.address = vector, .value = handler_high, .write = false});
    }
    if (reads > 1) { accesses.push_back({.address = vector + 1, .value = 0, .write = false}); }
    auto expected = foreground;
    expected.sp = static_cast<std::uint16_t>(stack - pushes);
    if (pushes == 5) { expected.ccr = entry_flags; }
    if (reads == 2) { expected.pc = handler; }
    require(error.contains("pc=0x1234") && error.contains("address=") && cpu.faulted() &&
                cpu.state() == expected && memory.accesses == accesses && memory.retired == 0 &&
                memory.acknowledgements == 0 && memory.entries == (cut == 0 ? 0U : 1U),
            "Every frame/vector/ACK failure retains the exact committed prefix and quarantines");
    require(rejected([&] { (void)cpu.service_interrupt(); }).contains("coordinated recovery") &&
                rejected([&] { cpu.step(); }).contains("coordinated recovery") &&
                memory.accesses == accesses,
            "Failed IRQ cannot retry partial stack effects implicitly");
  }
  IrqMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(foreground);
  memory.on_entry = [&] { (void)cpu.service_interrupt(); };
  require(rejected([&] { (void)cpu.service_interrupt(); }).contains("Re-entrant") &&
              cpu.faulted() && memory.accesses.empty(),
          "A source callback cannot recursively enter the same native CPU");
}

void return_failures() {
  constexpr std::array<std::uint8_t, 6> body{0xa6, 0x42, 0x45, 0x12, 0x34, 0x80};
  constexpr unsigned a_mask = 0x55;
  constexpr unsigned x_mask = 0xaa;
  for (unsigned cut = 0; cut < 5; ++cut) {
    IrqMemory memory;
    memory.install(handler, body);
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    auto supplied = foreground;
    supplied.ccr_known = reset_known;
    supplied.a_known = a_mask;
    supplied.hx_known = high_mask | x_mask;
    cpu.set_state(supplied);
    const auto saved = cpu.state();
    require(cpu.service_interrupt(), "Actual partial hardware frame precedes each RTI failure cut");
    cpu.step();
    cpu.step();
    auto expected = cpu.state();
    const auto retired = memory.retired;
    if (cut < 3) {
      memory.return_stop = cut;
    } else {
      memory.known.reset(stack - 1 + cut - 3);
    }
    expected.sp = static_cast<std::uint16_t>(stack - 5 + std::min(cut + 1, 4U));
    if (cut > 0) {
      expected.ccr = saved.ccr;
      expected.ccr_known = saved.ccr_known;
    }
    if (cut > 1) {
      expected.a = saved.a;
      expected.a_known = saved.a_known;
    }
    if (cut > 2) {
      expected.hx = static_cast<std::uint16_t>(isr_high | (saved.hx & byte_mask));
      expected.hx_known = high_mask | x_mask;
    }
    const auto error = rejected([&] { cpu.step(); });
    require(cpu.faulted() && error.contains("address=") && cpu.state() == expected &&
                memory.retired == retired,
            "RTI read cut=" + std::to_string(cut) +
                " must retain actual completed register masks; " +
                "ccr-known=" + hex(cpu.state().ccr_known) + " a-known=" + hex(cpu.state().a_known) +
                " hx-known=" + hex(cpu.state().hx_known));
    const auto accesses = memory.accesses;
    require(rejected([&] { cpu.step(); }).contains("coordinated recovery") &&
                memory.accesses == accesses,
            "A partial RTI never silently retries committed register/stack reads");
  }
}

void partial_load_and_strict_consumers() {
  IrqMemory memory;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  auto s = foreground;
  s.ccr_known = partial_ccr;
  cpu.set_state(s);
  require(cpu.service_interrupt(), "Actual IRQ stores a partial saved CCR");
  constexpr unsigned saved_ccr = stack - 4;
  require(!memory.known[saved_ccr] && memory.partial_known[saved_ccr] == partial_ccr &&
              rejected([&] { (void)memory.read(saved_ccr); }).contains("read") &&
              rejected([&] {
                memory.check_execute({.address = saved_ccr, .length = 1});
              }).contains("fetch"),
          "Ordinary data reads and fetches never treat unknown frame bits as known zeros");
  // Own LDA $023C; BEQ skips CMP only on zero; CMP #0 remains unqualified for partial A.
  constexpr std::array<std::uint8_t, 7> load{0xc6, 2, 0x3c, 0x27, 2, 0xa1, 0};
  memory.install(handler, load);
  cpu.step();
  auto expected = foreground;
  expected.pc = handler + 3;
  expected.sp = stack - 5;
  expected.a = saved_flags;
  expected.a_known = partial_ccr;
  expected.ccr = entry_flags;
  expected.ccr_known = partial_ccr;
  require(
      cpu.state() == expected && !cpu.faulted() && memory.return_reads == 1 &&
          memory.accesses.back() ==
              Access{
                  .address = saved_ccr, .value = saved_flags, .write = false, .known = partial_ccr},
      "Native LDA preserves the saved H-bit uncertainty in A; known nonzero and bit7 define Z/N");
  cpu.step();
  expected.pc = handler + 5;
  require(cpu.state() == expected && memory.retired == 2,
          "A real branch can consume known Z without consuming unknown A bits");
  require(rejected([&] { cpu.step(); }).contains("unknown operand") && cpu.faulted() &&
              memory.return_reads == 1 && memory.retired == 2,
          "Unqualified arithmetic cannot use the partial loaded byte or retire silently");
}

class PlainMemory final : public Bus {
public:
  Memory memory;
  unsigned entries = 0, acknowledgements = 0;

  void begin_interrupt(const InterruptRequest request, const bool reuse) override {
    require(request.vector == vector && request.kind == InterruptKind::maskable && !reuse,
            "Plain memory receives the actual hardware IRQ entry");
    ++entries;
  }

  void acknowledge_interrupt(const std::uint16_t address) override {
    require(address == vector, "Plain memory acknowledges the selected vector");
    ++acknowledgements;
  }

  void retire_instruction() override { memory.retire_instruction(); }

  std::uint8_t read(const std::uint16_t address) override { return memory.read(address); }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    memory.write(address, value);
  }

  [[nodiscard]] std::uint8_t peek(const std::uint16_t address) const override {
    return memory.peek(address);
  }

  [[nodiscard]] std::uint64_t revision() const override { return memory.revision(); }

  void check_execute(const InstructionFetch fetch) const override { memory.check_execute(fetch); }

  [[nodiscard]] std::optional<InterruptRequest>
  interrupt_request(std::uint8_t /*ccr*/) const override {
    return InterruptRequest{.vector = vector};
  }
};

void required_capability() {
  PlainMemory plain;
  plain.memory.writable.set();
  Hcs08 cpu(plain, BLOFELD_HCS08_SLA);
  auto s = foreground;
  s.ccr_known = partial_ccr;
  cpu.set_state(s);
  const auto before = cpu.state();
  const auto error = rejected([&] { (void)cpu.service_interrupt(); });
  require(error.contains("masked RAM storage") && error.contains("address=0x023c") &&
              cpu.state() == before && plain.memory.accesses.empty(),
          "Missing memory knowledge capability refuses before any hardware frame write");
  constexpr std::array<std::uint8_t, 2> target{handler_high, 0};
  constexpr std::array<std::uint8_t, 1> rti{0x80};
  plain.memory.install(vector, target);
  plain.memory.install(handler, rti);
  cpu.set_state(foreground);
  require(cpu.service_interrupt() && plain.entries == 1 && plain.acknowledgements == 1 &&
              plain.memory.retired == 0,
          "Fully known hardware frames require no optional masked-memory capability");
  cpu.step();
  require(cpu.state() == foreground && plain.memory.retired == 1,
          "Native RTI restores a real fully known frame using ordinary memory reads");
}

class NoBdcMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override { return false; }

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC write");
  }
};

class PolicyMemory final : public Memory {
public:
  unsigned entries = 0;

  void begin_interrupt(const InterruptRequest request, const bool reuse) override {
    require(request.vector == vector && request.kind == InterruptKind::maskable && !reuse,
            "Own restricted RAM test receives the actual architectural IRQ entry");
    ++entries;
  }

  void write(const std::uint16_t address, const std::uint8_t value) override {
    constexpr unsigned last_programmer_write = 0x84;
    if (address > last_programmer_write) { throw Fault("Own restricted RAM write denied"); }
    Memory::write(address, value);
  }
};

void policy_capability() {
  for (const auto mask : {partial_ccr, byte_mask}) {
    PolicyMemory memory;
    memory.writable.set();
    memory.interrupt = InterruptRequest{.vector = vector};
    const Bus &board = memory;
    require(dynamic_cast<const Hcs08KnownMemory *>(&board) == nullptr,
            "An ordinary RAM helper cannot implicitly grant bypasses of a derived write policy");
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    auto before = foreground;
    before.ccr_known = static_cast<std::uint8_t>(mask);
    cpu.set_state(before);
    const auto error = rejected([&] { (void)cpu.service_interrupt(); });
    require(error.contains(mask == partial_ccr ? "masked RAM storage" : "restricted RAM write") &&
                cpu.faulted() && cpu.state() == before && memory.accesses.empty() &&
                memory.known.none() && memory.entries == (mask == partial_ccr ? 0U : 1U),
            "Partial IRQ refuses missing capability before writes; fully known IRQ uses actual "
            "ordinary policy, with no accepted stack byte or fabricated known bits");
  }
}

void bdc_ownership() {
  IrqMemory memory;
  NoBdcMemory target;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state(foreground);
  Hcs08Bdc bdc(cpu, target, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
  require(!bdc.service_interrupt() && memory.accesses.empty() && cpu.state() == foreground &&
              memory.entries == 0 && memory.retired == 0,
          "Actual held BDC owns the CPU and prevents IRQ entry");
  bdc.receive(hcs08_bdc::go);
  bdc.service();
  require(bdc.service_interrupt() && memory.retired == 0,
          "BDC GO permits actual IRQ admission without an invented instruction retirement");
  entered(memory, cpu, foreground);
}
}

int main() {
  try {
    vectors_and_wrap();
    masked_frames();
    masking_and_invalid();
    failures();
    return_failures();
    partial_load_and_strict_consumers();
    required_capability();
    policy_capability();
    bdc_ownership();
    std::cout << "HCS08 hardware IRQ frames, native masked RTI and strict ownership pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
