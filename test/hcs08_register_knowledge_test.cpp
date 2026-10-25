// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr unsigned reset_ccr = 0x68, reset_hx_known = 0xff00, initial_hx = 0x1234;
constexpr unsigned initial_ccr = 0x79, initial_sp = 0x240, loaded_hx = 0x5aa5;
constexpr auto normal = Hcs08ResetEntry::normal, active = Hcs08ResetEntry::active_background;
constexpr auto power_on = Hcs08ResetCause::power_on;

constexpr unsigned lda_immediate = 0xa6;
constexpr unsigned shift_x = 0x58, pull_x = 0x88, push_x = 0x89;
constexpr unsigned jmp_direct = 0xbc, jmp_extended = 0xcc;
constexpr unsigned jmp_indexed16 = 0xdc, jmp_indexed8 = 0xec, jmp_indexed = 0xfc;
constexpr unsigned lda_direct = 0xb6;
constexpr unsigned lda_extended = 0xc6;
constexpr unsigned clra = 0x4f;
constexpr unsigned tpa = 0x85;
constexpr unsigned ldx_immediate = 0xae;
constexpr unsigned clrx = 0x5f;
constexpr unsigned ldhx_immediate = 0x45;
constexpr unsigned ldhx_direct = 0x55;
constexpr unsigned loaded_direct_hx = 0x5a5a;
constexpr unsigned clrh = 0x8c;
constexpr unsigned txs = 0x94;
constexpr unsigned aix = 0xaf;
constexpr unsigned cphx_immediate = 0x65;
constexpr unsigned sthx_direct = 0x35;
constexpr unsigned clr_indexed = 0x7f;
constexpr unsigned sta_direct = 0xb7;
constexpr unsigned sta_extended = 0xc7;
constexpr unsigned bit_immediate = 0xa5;
constexpr unsigned cmp_immediate = 0xa1;
constexpr unsigned tap = 0x84;
constexpr unsigned stack_prefix = 0x9e;
constexpr unsigned mov_immediate_direct = 0x6e;
constexpr unsigned clr_direct = 0x3f;
constexpr unsigned bra = 0x20;
constexpr unsigned bcs = 0x25;
constexpr unsigned beq = 0x27;
constexpr unsigned jsr_extended = 0xcd;
constexpr unsigned ais = 0xa7;
constexpr unsigned nop = 0x9d;
constexpr unsigned bgnd = 0x82;
constexpr unsigned cli = 0x9a;
constexpr unsigned sei = 0x9b;
constexpr unsigned clc = 0x98;
constexpr unsigned sec = 0x99;
constexpr unsigned ldx_direct = 0xbe, and_immediate = 0xa4, inc_a = 0x4c, inc_x = 0x5c;
constexpr unsigned mov_direct = 0x4e, cmp_direct = 0xb1;
constexpr unsigned lda_indexed16 = 0xd6, cmp_extended = 0xc1;
constexpr unsigned push_h = 0x8b, pull_h = 0x8a, tax = 0x97, txa = 0x9f, brclr7 = 0x0f, rti = 0x80;
constexpr unsigned vector_high = 0xfffe;
constexpr unsigned vector_low = 0xffff;
constexpr unsigned sign_bit = 0x80;
constexpr unsigned loaded_a = 0x42;
constexpr unsigned load_ccr_known = 0xee;
constexpr unsigned mixed_hx_mask = 0xa55a;
constexpr unsigned data_byte = 0x5a;
constexpr unsigned extended_address = 0x4040;
constexpr unsigned clear_ccr = 0x7b;
constexpr unsigned tap_ccr = 0x7a;
constexpr unsigned borrow_ccr = 0x7d;
constexpr unsigned carry_clear_ccr = 0x78;
constexpr unsigned interrupt_clear_ccr = 0x71;
constexpr unsigned index_high = 0x12;
constexpr unsigned index_low = 0x34;
constexpr unsigned write_pc_command = 0x4b;
constexpr unsigned write_ccr_command = 0x49;
constexpr unsigned write_a_command = 0x48;
constexpr unsigned write_hx_command = 0x4c;
constexpr unsigned read_hx_command = 0x6c;
constexpr unsigned write_sp_command = 0x4f;
constexpr unsigned partial_a_mask = 0x55;
constexpr unsigned stack_data_low = 0xa5;
constexpr unsigned read_a_command = 0x68;
constexpr unsigned ldhx_stack = 0xfe;

void cold_reset() {
  Memory memory;
  constexpr std::array<std::uint8_t, 2> vector{sign_bit, 0};
  memory.install(vector_high, vector);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  require(rejected([&] { cpu.reset(); }).contains("pre-reset A/X") && memory.accesses.empty(),
          "Fresh CPU cannot silently select warm inputs");
  cpu.reset(normal, power_on);
  const Hcs08State expected{.pc = entry,
                            .sp = byte_mask,
                            .ccr_known = reset_ccr,
                            .ccr = reset_ccr,
                            .a_known = 0,
                            .hx_known = reset_hx_known};
  require(cpu.state() == expected && memory.retired == 0 &&
              memory.accesses ==
                  std::vector<Access>{{.address = vector_high, .value = sign_bit, .write = false},
                                      {.address = vector_low, .value = 0, .write = false}},
          "Fresh power-on reads genuine vector, knows H/SP/I and invents neither A nor X");
  cpu.reset();
  require(cpu.state() == expected, "Warm reset retains unknown A/X rather than defining zeros");
  const auto before = cpu.state();
  require(rejected([&] { cpu.reset(normal, Hcs08ResetCause::unqualified); }).contains("cause") &&
              cpu.state() == before && !cpu.faulted(),
          "Unqualified reset cause cannot mutate a healthy CPU");
  Memory absent;
  Hcs08 other(absent, BLOFELD_HCS08_SLA);
  require(rejected([&] { other.reset(normal, power_on); }).contains("pc=unknown") &&
              other.faulted() && absent.retired == 0,
          "Missing cold vector remains a visible fault without an invented previous PC");
  Hcs08 held(absent, BLOFELD_HCS08_SLA);
  held.reset(active, power_on);
  auto prevector = expected;
  prevector.pc = 0;
  prevector.pc_known = false;
  require(held.state() == prevector && absent.accesses.empty(),
          "Fresh active power-on neither reads a vector nor defines a numerical PC");
}

void masks_and_checkpoint() {
  Memory memory;
  constexpr std::array<std::uint8_t, 3> load{lda_immediate, loaded_a, nop}; // Own LDA #42; NOP.
  memory.install(entry, load);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint16_t, 5> hx_masks{
      0, byte_mask, reset_hx_known, mixed_hx_mask, word_mask};
  for (unsigned mask = 0; mask < byte_values; ++mask) {
    for (const auto hx_mask : hx_masks) {
      const Hcs08State supplied{.pc = entry,
                                .hx = word_mask,
                                .a = byte_mask,
                                .a_known = static_cast<std::uint8_t>(mask),
                                .hx_known = hx_mask};
      cpu.set_state(supplied);
      auto expected = supplied;
      expected.a = static_cast<std::uint8_t>(mask);
      expected.hx = hx_mask;
      require(cpu.state() == expected, "Unknown representatives are masked for every A bit mask");
      cpu.reset(active);
      expected.pc = 0;
      expected.pc_known = false;
      expected.sp = byte_mask;
      expected.hx &= byte_mask;
      expected.hx_known = static_cast<std::uint16_t>((hx_mask & byte_mask) | reset_hx_known);
      expected.ccr_known = reset_ccr;
      expected.ccr = reset_ccr;
      require(cpu.state() == expected,
              "Warm reset retains each independently supplied A/X mask while clearing known H");
    }
  }
  cpu.set_state({.pc = entry,
                 .ccr_known = reset_ccr,
                 .ccr = reset_ccr,
                 .a_known = 0,
                 .hx_known = reset_hx_known});
  const auto checkpoint = cpu.checkpoint();
  for (unsigned replay = 0; replay < 2; ++replay) {
    Memory restored(memory);
    Hcs08 other(restored, BLOFELD_HCS08_SLA);
    other.restore_checkpoint(checkpoint);
    require(other.state() == cpu.state(), "Fresh opaque restore preserves unknown A/X masks");
    other.step();
    require(other.state() == Hcs08State{.pc = entry + 2,
                                        .a = loaded_a,
                                        .ccr_known = load_ccr_known,
                                        .ccr = reset_ccr,
                                        .hx_known = reset_hx_known},
            "Native LDA establishes A without establishing unrelated X or reset flags");
  }
  cpu.step();
  cpu.restore_checkpoint(checkpoint);
  require(cpu.state().a_known == 0 && cpu.state().hx_known == reset_hx_known,
          "Restoring an earlier boundary removes later register knowledge");
}

// Independent literal ISA classification; no production decoder/table reuse.
bool admitted(const unsigned opcode, const Hcs08State s) {
  switch (opcode) {
    case lda_immediate:
    case lda_direct:
    case lda_extended:
    case clra:
    case tpa:
    case ldx_immediate:
    case ldx_direct:
    case clrx:
    case pull_x:
    case ldhx_immediate:
    case ldhx_direct:
    case clrh:
    case mov_immediate_direct:
    case mov_direct:
    case pull_h:
    case brclr7:
    case rti:
    case clr_direct:
    case bra:
    case bcs:
    case beq:
    case jsr_extended:
    case jmp_direct:
    case jmp_extended:
    case ais:
    case nop:
    case bgnd:
    case cli:
    case sei:
    case clc:
    case sec: return true;
    case lda_indexed16:
    case txs:
    case aix:
    case cphx_immediate:
    case sthx_direct:
    case clr_indexed:
    case jmp_indexed16:
    case jmp_indexed8:
    case jmp_indexed: return s.hx_known == word_mask;
    case push_h: return (s.hx_known & reset_hx_known) == reset_hx_known;
    case txa:
    case inc_x:
    case shift_x:
    case push_x: return (s.hx_known & byte_mask) == byte_mask;
    case tax:
    case inc_a:
    case and_immediate:
    case cmp_direct:
    case cmp_extended:
    case sta_direct:
    case sta_extended:
    case bit_immediate:
    case cmp_immediate:
    case tap: return s.a_known == byte_mask;
    default: return false;
  }
}

Hcs08State irq_body_effect(const unsigned opcode, Hcs08State s) {
  switch (opcode) {
    case ldx_direct:
      s.hx = static_cast<std::uint16_t>((s.hx & reset_hx_known) | data_byte);
      s.hx_known |= byte_mask;
      s.ccr = initial_ccr;
      break;
    case pull_h:
      s.hx =
          static_cast<std::uint16_t>((s.hx & byte_mask) | static_cast<unsigned>(data_byte << 8U));
      s.hx_known |= reset_hx_known;
      ++s.sp;
      break;
    case push_h:
    case push_x: --s.sp; break;
    case pull_x:
      ++s.sp;
      s.hx = static_cast<std::uint16_t>((s.hx & reset_hx_known) | data_byte);
      s.hx_known |= byte_mask;
      break;
    case shift_x: {
      const unsigned old_x = s.hx & byte_mask;
      const unsigned next_x = (old_x * 2U) & byte_mask;
      s.hx = static_cast<std::uint16_t>((s.hx & reset_hx_known) | next_x);
      s.ccr = static_cast<std::uint8_t>(
          (s.ccr & ~(flag_v | flag_n | flag_z | flag_c)) | ((old_x & sign_bit) != 0 ? flag_c : 0U) |
          ((next_x & sign_bit) != 0 ? flag_n : 0U) | (next_x == 0 ? flag_z : 0U) |
          (((next_x ^ old_x) & sign_bit) != 0 ? flag_v : 0U));
      break;
    }
    case inc_x:
      ++s.hx;
      s.ccr = initial_ccr;
      break;
    case inc_a:
      ++s.a;
      s.ccr = initial_ccr;
      break;
    case tax:
      s.hx = static_cast<std::uint16_t>((s.hx & reset_hx_known) | data_byte);
      s.hx_known |= byte_mask;
      break;
    case txa:
      s.a = index_low;
      s.a_known = byte_mask;
      break;
    case and_immediate:
      s.a = operand;
      s.ccr = initial_ccr;
      break;
    case cmp_extended:
    case cmp_direct: s.ccr = tap_ccr; break;
    case mov_direct: s.ccr = initial_ccr; break;
    case brclr7: s.ccr = carry_clear_ccr; break;
    case rti:
      s.a = data_byte;
      s.a_known = byte_mask;
      s.hx = static_cast<std::uint16_t>((s.hx & reset_hx_known) | index_low);
      s.hx_known |= byte_mask;
      s.sp += 5;
      s.ccr = initial_ccr;
      break;
    default: break;
  }
  return s;
}

unsigned instruction_length(const unsigned opcode) {
  switch (opcode) {
    case ldhx_immediate:
    case lda_extended:
    case lda_indexed16:
    case cmp_extended:
    case sta_extended:
    case cphx_immediate:
    case jsr_extended:
    case mov_immediate_direct:
    case mov_direct:
    case brclr7:
    case rti: return 3; // Own RTI frame returns to8003.
    case lda_immediate:
    case lda_direct:
    case ldhx_direct:
    case ldx_immediate:
    case ldx_direct:
    case and_immediate:
    case cmp_direct:
    case aix:
    case sthx_direct:
    case sta_direct:
    case bit_immediate:
    case cmp_immediate:
    case clr_direct:
    case bra:
    case bcs:
    case beq:
    case ais: return 2;
    default: return 1;
  }
}

Hcs08State effect(const unsigned opcode, Hcs08State s) {
  s = irq_body_effect(opcode, s);
  switch (opcode) {
    case lda_immediate:
      s.a = operand;
      s.a_known = byte_mask;
      s.ccr = initial_ccr;
      break;
    case lda_direct:
    case lda_extended:
    case lda_indexed16:
      s.a = data_byte;
      s.a_known = byte_mask;
      s.ccr = initial_ccr;
      break;
    case clra:
      s.a = 0;
      s.a_known = byte_mask;
      s.ccr = clear_ccr;
      break;
    case tpa:
      s.a = initial_ccr;
      s.a_known = byte_mask;
      break;
    case ldx_immediate:
      s.hx = static_cast<std::uint16_t>((s.hx & reset_hx_known) | operand);
      s.hx_known |= byte_mask;
      s.ccr = initial_ccr;
      break;
    case clrx:
      s.hx &= reset_hx_known;
      s.hx_known |= byte_mask;
      s.ccr = clear_ccr;
      break;
    case clrh:
      s.hx &= byte_mask;
      s.hx_known |= reset_hx_known;
      s.ccr = clear_ccr;
      break;
    case ldhx_immediate:
      s.hx = extended_address;
      s.hx_known = word_mask;
      s.ccr = initial_ccr;
      break;
    case ldhx_direct:
      s.hx = loaded_direct_hx;
      s.hx_known = word_mask;
      s.ccr = initial_ccr;
      break;
    case txs: s.sp = initial_hx - 1; break;
    case aix: s.hx = initial_hx + operand; break;
    case cphx_immediate: s.ccr = borrow_ccr; break;
    case sthx_direct:
    case sta_direct:
    case sta_extended:
    case bit_immediate:
    case mov_immediate_direct: s.ccr = initial_ccr; break;
    case clr_indexed:
    case clr_direct: s.ccr = clear_ccr; break;
    case cmp_immediate: s.ccr = carry_clear_ccr; break; // A5A - immediate40 =1A, no borrow.
    case tap: s.ccr = tap_ccr; break;                   // A5A, fixed bits forced to one.
    case jsr_extended: s.sp = initial_sp - 2; break;
    case ais: s.sp = initial_sp + operand; break;
    case cli: s.ccr = interrupt_clear_ccr; break;
    case clc: s.ccr = carry_clear_ccr; break;
    default: break;
  }
  s.pc = static_cast<std::uint16_t>(entry + instruction_length(opcode));
  if (opcode == bra || opcode == bcs || opcode == brclr7) { s.pc += operand; }
  if (opcode == jsr_extended) { s.pc = extended_address; }
  if (opcode == jmp_direct) { s.pc = operand; }
  if (opcode == jmp_extended) { s.pc = extended_address; }
  if (opcode == jmp_indexed) { s.pc = s.hx; }
  if (opcode == jmp_indexed8 || opcode == jmp_indexed16) {
    const unsigned offset = opcode == jmp_indexed8 ? operand : extended_address;
    s.pc = static_cast<std::uint16_t>(s.hx + offset);
  }
  return s;
}

void opcode_case(Memory &memory, Hcs08 &cpu, const unsigned opcode, const Hcs08State initial) {
  memory.bytes.fill(data_byte);
  const std::array<std::uint8_t, 3> code{static_cast<std::uint8_t>(opcode), operand, operand};
  memory.install(entry, code);
  if (opcode == rti) {
    constexpr std::array<std::uint8_t, 5> frame{0x79, 0x5a, 0x34, 0x80, 3};
    memory.install(initial_sp + 1, frame);
  }
  memory.accesses.clear();
  cpu.set_state(initial);
  const auto before = cpu.state();
  const auto retired = memory.retired;
  const auto bytes = memory.bytes;
  if (!admitted(opcode, before)) {
    const auto failure = rejected([&] { cpu.step(); });
    require(failure.contains("partial-register") && failure.contains("pc=0x8000") &&
                failure.contains("a-known=") && failure.contains("hx-known=") && cpu.faulted() &&
                cpu.state() == before && memory.retired == retired && memory.accesses.empty() &&
                memory.bytes == bytes,
            "Every unsupported or unknown-consuming opcode rejects before data effects");
  } else {
    cpu.step();
    require(cpu.state() == effect(opcode, before) && memory.retired == retired + 1 &&
                cpu.background_requested() == (opcode == bgnd),
            "Literal independent register/flag/PC effects cover each partial-register form " +
                hex(opcode));
    if (opcode == jsr_extended) {
      require(memory.bytes[initial_sp] == 3 && memory.bytes[initial_sp - 1] == sign_bit,
              "Native JSR pushes the genuine return address while A remains unknown");
    }
    if (opcode == push_h) {
      require(memory.bytes[initial_sp] == index_high, "PSHH pushes actual independently known H");
    }
    if (opcode == mov_direct) {
      require(memory.bytes[operand] == data_byte,
              "MOV direct copies the actual independent source byte");
    }
    if (opcode == mov_immediate_direct) {
      require(memory.bytes[operand] == operand && cpu.state().a_known == before.a_known,
              "Native MOV immediate writes memory without defining unknown A");
    }
  }
}

void opcodes() {
  Memory memory;
  memory.known.set();
  memory.writable.set();
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  constexpr std::array<std::uint16_t, 3> hx_masks{word_mask, reset_hx_known, byte_mask};
  for (const auto hx_mask : hx_masks) {
    for (const unsigned a_mask : {0U, partial_a_mask, byte_mask}) {
      if (hx_mask == word_mask && a_mask == byte_mask) { continue; }
      for (unsigned opcode = 0; opcode < byte_values; ++opcode) {
        opcode_case(memory,
                    cpu,
                    opcode,
                    {.pc = entry,
                     .hx = initial_hx,
                     .sp = initial_sp,
                     .a = data_byte,
                     .ccr = initial_ccr,
                     .a_known = static_cast<std::uint8_t>(a_mask),
                     .hx_known = hx_mask});
      }
    }
  }
}

void prefixes_and_flags() {
  Memory memory;
  memory.known.set();
  memory.writable.set();
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  for (unsigned second = 0; second < byte_values; ++second) {
    const std::array<std::uint8_t, 3> code{
        stack_prefix, static_cast<std::uint8_t>(second), operand};
    memory.install(entry, code);
    memory.bytes[initial_sp + operand] = data_byte;
    memory.bytes[initial_sp + operand + 1] = stack_data_low;
    memory.accesses.clear();
    cpu.set_state({.pc = entry,
                   .sp = initial_sp,
                   .ccr = initial_ccr,
                   .a_known = 0,
                   .hx_known = reset_hx_known});
    if (second == ldhx_stack) {
      cpu.step();
      require(cpu.state().hx == loaded_hx && cpu.state().hx_known == word_mask &&
                  cpu.state().a_known == 0 && cpu.state().pc == entry + 3,
              "Actual stack-relative LDHX defines both bytes without consuming prior HX");
    } else {
      require(rejected([&] { cpu.step(); }).contains("partial-register") && memory.accesses.empty(),
              "Every other stack prefix, including STHX with unknown X, refuses data effects");
    }
  }
  constexpr std::array<std::uint8_t, 1> transfer{tpa};
  memory.install(entry, transfer);
  cpu.set_state({.pc = entry,
                 .ccr_known = reset_ccr,
                 .ccr = reset_ccr,
                 .a_known = 0,
                 .hx_known = reset_hx_known});
  require(rejected([&] { cpu.step(); }).contains("ccr-known=0x0068"),
          "TPA cannot turn unknown reset flags into known A");
  constexpr std::array<std::uint8_t, 1> prefix{stack_prefix};
  for (const unsigned address : {entry, word_mask}) {
    memory.known.reset();
    memory.install(address, prefix);
    cpu.set_state({.pc = static_cast<std::uint16_t>(address), .a_known = 0, .hx_known = 0});
    const auto error = rejected([&] { cpu.step(); });
    require(error.contains(address == entry ? "fetch" : "wraps"),
            "Partial-register prefix requires actual non-wrapped second opcode bytes");
  }
}

void direct_word_knowledge() {
  Memory memory;
  constexpr std::array<std::uint8_t, 2> code{ldhx_direct, operand};
  constexpr std::array<std::uint8_t, 2> data{data_byte, stack_data_low};
  memory.install(entry, code);
  memory.install(operand, data);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.set_state({.pc = entry,
                 .sp = initial_sp,
                 .ccr_known = reset_ccr,
                 .ccr = reset_ccr,
                 .a_known = 0,
                 .hx_known = 0});
  const auto before = cpu.state();
  const auto checkpoint = cpu.checkpoint();
  const auto initial_memory = memory;
  cpu.step();
  const Hcs08State expected{.pc = entry + 2,
                            .hx = loaded_hx,
                            .sp = initial_sp,
                            .ccr_known = load_ccr_known,
                            .ccr = reset_ccr,
                            .a_known = 0};
  require(cpu.state() == expected && memory.retired == 1 &&
              memory.accesses ==
                  std::vector<Access>{
                      {.address = operand, .value = data_byte, .write = false},
                      {.address = operand + 1, .value = stack_data_low, .write = false}},
          "Actual direct LDHX defines both HX bytes from ordered reads; A/H/C stay unknown");
  for (unsigned replay = 0; replay < 2; ++replay) {
    Memory restored(initial_memory);
    Hcs08 other(restored, BLOFELD_HCS08_SLA);
    other.restore_checkpoint(checkpoint);
    other.step();
    require(other.state() == expected && restored.accesses == memory.accesses &&
                restored.retired == 1,
            "Fresh direct-word continuations preserve the independent register/read oracle");
  }
  for (unsigned missing = 0; missing < data.size(); ++missing) {
    Memory unknown(initial_memory);
    unknown.known.reset(operand + missing);
    Hcs08 other(unknown, BLOFELD_HCS08_SLA);
    other.restore_checkpoint(checkpoint);
    const auto error = rejected([&] { other.step(); });
    require(error.contains("read address=" + hex(operand + missing)) &&
                error.contains("pc=0x8000") && other.faulted() && other.state() == before &&
                unknown.retired == 0 && unknown.accesses.size() == missing,
            "Unknown first/second data bytes cannot establish HX or retire the instruction");
  }
}

class RetirementMemory final : public Memory {
public:
  const Hcs08 *cpu = nullptr;
  std::optional<Hcs08State> observed;
  bool refuse = false, checkpoint_blocked = false;

  void retire_instruction() override {
    observed = cpu->state();
    checkpoint_blocked = rejected([this] {
                           static_cast<void>(cpu->checkpoint());
                         }).contains("healthy completed native boundary");
    if (refuse) { throw Fault("Own retirement observer failure"); }
    Memory::retire_instruction();
  }
};

void retirement_knowledge() {
  struct Case {
    std::array<std::uint8_t, 3> code;
    unsigned length;
    Hcs08State expected;
  };

  constexpr std::array<Case, 3> cases{{{.code = {lda_immediate, loaded_a, 0},
                                        .length = 2,
                                        .expected = {.pc = entry + 2,
                                                     .sp = initial_sp,
                                                     .a = loaded_a,
                                                     .ccr_known = load_ccr_known,
                                                     .ccr = reset_ccr,
                                                     .hx_known = 0}},
                                       {.code = {ldhx_immediate, data_byte, stack_data_low},
                                        .length = 3,
                                        .expected = {.pc = entry + 3,
                                                     .hx = loaded_hx,
                                                     .sp = initial_sp,
                                                     .ccr_known = load_ccr_known,
                                                     .ccr = reset_ccr,
                                                     .a_known = 0}},
                                       {.code = {ldhx_direct, operand, 0},
                                        .length = 2,
                                        .expected = {.pc = entry + 2,
                                                     .hx = loaded_hx,
                                                     .sp = initial_sp,
                                                     .ccr_known = load_ccr_known,
                                                     .ccr = reset_ccr,
                                                     .a_known = 0}}}};
  constexpr std::array<std::uint8_t, 2> data{data_byte, stack_data_low};
  for (const auto &sample : cases) {
    for (const bool refuse : {false, true}) {
      RetirementMemory memory;
      memory.install(entry, std::span(sample.code).first(sample.length));
      memory.install(operand, data);
      memory.refuse = refuse;
      Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
      memory.cpu = &cpu;
      cpu.set_state({.pc = entry,
                     .sp = initial_sp,
                     .ccr_known = reset_ccr,
                     .ccr = reset_ccr,
                     .a_known = 0,
                     .hx_known = 0});
      if (refuse) {
        require(rejected([&] { cpu.step(); }).contains("Own retirement observer failure") &&
                    cpu.faulted() && memory.retired == 0 &&
                    rejected([&] { cpu.step(); }).contains("coordinated recovery"),
                "A failing retirement hook quarantines the owner without undoing actual native "
                "effects");
      } else {
        cpu.step();
        require(!cpu.faulted() && memory.retired == 1, "A completed hook retires once");
        static_cast<void>(cpu.checkpoint());
      }
      require(memory.observed == sample.expected && cpu.state() == sample.expected &&
                  memory.checkpoint_blocked,
              "Retirement sees complete A/HX/CCR facts, while native checkpoint admission stays "
              "closed");
    }
  }
}

class NoMemory final : public Hcs08BdcMemory {
public:
  [[nodiscard]] bool secured(ExecutionContext /*context*/) const override {
    return false;
  } // Explicit unsecured fixture.

  std::uint8_t read(std::uint16_t /*address*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC read");
  }

  void write(ByteWrite /*access*/, ExecutionContext /*context*/) override {
    throw Fault("Unexpected BDC write");
  }
};

void command(Hcs08Bdc &bdc, const std::span<const std::uint8_t> frame) {
  for (const auto byte : frame) { bdc.receive(byte); }
  bdc.service();
}

void bdc_registers() {
  Memory memory;
  NoMemory target;
  for (const auto opcode : {hcs08_bdc::read_a, hcs08_bdc::read_hx}) {
    Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
    cpu.reset(active, power_on);
    Hcs08Bdc bdc(cpu, target, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
    const std::array<std::uint8_t, 1> frame{opcode};
    const auto failure = rejected([&] { command(bdc, frame); });
    require(failure.contains(opcode == hcs08_bdc::read_a ? "READ_A" : "READ_HX") &&
                failure.contains("unknown CPU") && bdc.faulted() && bdc.state().reply_size == 0 &&
                cpu.state().a_known == 0 && cpu.state().hx_known == reset_hx_known,
            "Real BDC reads refuse unknown cold registers without serializing representatives");
  }
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.reset(active, power_on);
  Hcs08Bdc bdc(cpu, target, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
  constexpr std::array<std::uint8_t, 3> write_pc{write_pc_command, sign_bit, 0};
  constexpr std::array<std::uint8_t, 3> write_sp{write_sp_command, 2, operand};
  constexpr std::array<std::uint8_t, 2> write_ccr{write_ccr_command, initial_ccr};
  constexpr std::array<std::uint8_t, 2> write_a{write_a_command, data_byte};
  constexpr std::array<std::uint8_t, 3> write_hx{write_hx_command, index_high, index_low};
  command(bdc, write_pc);
  command(bdc, write_sp);
  command(bdc, write_ccr);
  require(cpu.state().a_known == 0 && cpu.state().hx_known == reset_hx_known,
          "Actual non-A/HX BDC writes preserve unknown cold registers");
  command(bdc, write_a);
  require(cpu.state().a_known == byte_mask && cpu.state().hx_known == reset_hx_known,
          "WRITE_A establishes only A knowledge");
  command(bdc, write_hx);
  require(cpu.state().a == data_byte && cpu.state().hx == initial_hx &&
              cpu.state().a_known == byte_mask && cpu.state().hx_known == word_mask,
          "WRITE_HX establishes both actual index-register bytes");
  constexpr std::array<std::uint8_t, 1> read_a{read_a_command};
  constexpr std::array<std::uint8_t, 1> read_hx{read_hx_command};
  command(bdc, read_a);
  require(bdc.read_reply() == data_byte, "READ_A returns the actual written byte");
  command(bdc, read_hx);
  require(bdc.read_reply() == index_high && bdc.read_reply() == index_low,
          "READ_HX returns actual high and low bytes in command order");
}

void bdc_unknown_resume() {
  Memory memory;
  constexpr std::array<std::uint8_t, 2> code{lda_immediate, loaded_a};
  memory.install(entry, code);
  NoMemory target;
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  cpu.reset(active, power_on);
  Hcs08Bdc bdc(cpu, target, Hcs08FlashContract::candidate_gt8a, Hcs08BdcReset::bkgd_held_low);
  constexpr std::array<std::uint8_t, 3> write_pc{write_pc_command, sign_bit, 0};
  constexpr std::array<std::uint8_t, 1> go{hcs08_bdc::go};
  command(bdc, write_pc);
  command(bdc, go);
  require(!bdc.state().active && cpu.state().a_known == 0 && cpu.state().hx_known == reset_hx_known,
          "GO preserves unknown A/X instead of relabeling representative zeros as known");
  bdc.step_cpu();
  require(cpu.state() == Hcs08State{.pc = entry + 2,
                                    .sp = byte_mask,
                                    .a = loaded_a,
                                    .ccr_known = load_ccr_known,
                                    .ccr = reset_ccr,
                                    .hx_known = reset_hx_known} &&
              memory.retired == 1,
          "Real resumed native LDA establishes A while untouched cold X remains unknown");
}

}

int main() {
  try {
    cold_reset();
    masks_and_checkpoint();
    opcodes();
    prefixes_and_flags();
    direct_word_knowledge();
    retirement_knowledge();
    bdc_registers();
    bdc_unknown_resume();
    std::cout << "HCS08 cold-register knowledge and native instruction dependencies pass.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
