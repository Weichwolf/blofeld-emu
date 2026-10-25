// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_wire_fixture.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_reset.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <vector>

namespace {
using namespace blofeld;
using namespace blofeld::test::hcs08;
constexpr auto contract = Hcs08ResetContract::reference_bus_edges;
constexpr auto low = Hcs08ResetDrive::low, released = Hcs08ResetDrive::released;

constexpr unsigned drive_boundary = 34, sample_boundary = 72, long_hold = 100, replay_finish = 106;
constexpr unsigned vector_address = 0xfffe, vector_last = 0xffff, previous_pc = 0x1234;
constexpr unsigned retained_x = 0x22, loaded_x = 0x78, loaded_sp = 0x77, direct_store = 0x80;
constexpr unsigned loaded_a = 0x55, reset_status_known = 0x68, loaded_status_known = 0xee,
                   written_status = 0x69;

constexpr ExecutionContext context{.pc = 0x1397, .ccr = 0xd0};

struct EdgeExpectation {
  unsigned edge, release, finish;
};

void expect_edge(const Hcs08ResetSequence &sequence,
                 const Hcs08ResetDrive input,
                 const Hcs08ModePad mode,
                 const EdgeExpectation expected) {
  const auto s = sequence.state();
  require(s.edges == expected.edge && sequence.drives_low() == (expected.edge < drive_boundary) &&
              sequence.level(input) ==
                  (expected.edge >= drive_boundary && expected.edge >= expected.release),
          "Independent drive/expected.release boundary at each reference reset period");
  const unsigned rising = expected.release < drive_boundary ? drive_boundary : expected.release;
  require(s.mode.has_value() == (expected.edge >= rising),
          "MS is sampled only at actual resolved rising boundary");
  if (s.mode) {
    require(*s.mode ==
                (mode == Hcs08ModePad::low ? Hcs08BdcReset::bkgd_held_low : Hcs08BdcReset::normal),
            "Independent reset mode polarity");
  }
  require(s.external_at_sample.has_value() == (expected.edge >= sample_boundary),
          "Separate38-period cause sample");
  if (s.external_at_sample) {
    require(*s.external_at_sample == (expected.release > sample_boundary),
            "Actual external level at classification edge");
  }
  require((s.phase == Hcs08ResetPhase::ready) == (expected.edge == expected.finish),
          "Exactly six reference entry periods follow released/classified reset");
}

void reset_edges() {
  constexpr std::array<unsigned, 8> releases{2, 33, 34, 35, 71, 72, 73, 100};
  for (const auto release : releases) {
    for (const auto mode : {Hcs08ModePad::low, Hcs08ModePad::high}) {
      Hcs08ResetSequence sequence(contract);
      const unsigned finish = (release < sample_boundary ? sample_boundary : release) + 6;
      for (unsigned edge = 1; edge <= finish; ++edge) {
        const auto input = edge < release ? low : released;
        sequence.edge(input, mode, context);
        expect_edge(sequence, input, mode, {.edge = edge, .release = release, .finish = finish});
      }
      const auto saved = sequence.state();
      require(
          rejected([&] { sequence.edge(released, mode, context); }).contains("already completed") &&
              sequence.faulted() && sequence.state() == saved,
          "Completed reset cannot accrue an invented extra CPU period");
    }
  }
}

void reset_replay_and_failures() {
  constexpr unsigned half_hold = 50;
  Hcs08ResetSequence original(contract);
  for (unsigned edge = 1; edge <= half_hold; ++edge) {
    original.edge(low, Hcs08ModePad::low, context);
  }
  const auto checkpoint = original;
  for (unsigned replay = 0; replay < 2; ++replay) {
    auto restored = checkpoint;
    for (unsigned edge = half_hold + 1; edge <= replay_finish; ++edge) {
      restored.edge(edge < long_hold ? low : released, Hcs08ModePad::low, context);
    }
    const auto completed = restored.state();
    require(completed.edges == replay_finish && completed.phase == Hcs08ResetPhase::ready &&
                completed.external_at_sample && *completed.external_at_sample,
            "Fresh copied reset owner preserves drive/sample/hold/entry progression");
  }
  require(rejected([] {
            const Hcs08ResetSequence invalid(Hcs08ResetContract::unqualified);
          }).contains("explicit"),
          "Unknown reset timing cannot be selected by default");
  Hcs08ResetSequence unknown(contract);
  require(rejected([&] {
            unknown.edge(Hcs08ResetDrive::unknown, Hcs08ModePad::low, context);
          }).contains("pc=0x1397") &&
              unknown.faulted(),
          "Unknown reset driver retains native host context and quarantines");
  Hcs08ResetSequence ms(contract);
  for (unsigned edge = 1; edge < drive_boundary; ++edge) {
    ms.edge(released, Hcs08ModePad::unknown, context);
  }
  require(rejected([&] { ms.edge(released, Hcs08ModePad::unknown, context); }).contains("BKGD/MS"),
          "Unknown mode at rising RESET cannot fabricate normal/active entry");
}

void native_reset() {
  Memory memory;
  constexpr std::array<std::uint8_t, 2> vector{0x80, 0};
  constexpr std::array<std::uint8_t, 10> code{
      0xae, 0x78, 0x94, 0xa6, 0x55, 0xc7, 0, 0x80, 0x9d, 0x82};
  memory.install(vector_address, vector);
  memory.install(entry, code);
  memory.writable.set(direct_store);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  constexpr unsigned initial_hx = 0x3322;
  constexpr unsigned initial_sp = 0x4567;
  constexpr unsigned initial_a = 0xa5;
  constexpr unsigned reset_known = 0x68;
  cpu.set_state(
      {.pc = previous_pc, .hx = initial_hx, .sp = initial_sp, .a = initial_a, .ccr = byte_mask});
  cpu.reset();
  require(
      cpu.state() == Hcs08State{.pc = entry,
                                .hx = retained_x,
                                .sp = byte_mask,
                                .a = initial_a,
                                .ccr_known = reset_known,
                                .ccr = reset_known} &&
          memory.retired == 0 &&
          memory.accesses ==
              std::vector<Access>{{.address = vector_address, .value = entry >> 8U, .write = false},
                                  {.address = vector_last, .value = 0, .write = false}},
      "Native warm reset retains A/X, clears H, initializes SP/I, reads actual vector and "
      "keeps other flags unknown");
  const auto checkpoint = cpu.checkpoint();
  constexpr std::array<unsigned, 6> pcs{0x8002, 0x8003, 0x8005, 0x8008, 0x8009, 0x800a};
  for (unsigned n = 0; n < pcs.size(); ++n) {
    cpu.step();
    const auto s = cpu.state();
    require(s.pc == pcs[n] && s.hx == loaded_x && s.sp == (n == 0 ? byte_mask : loaded_sp) &&
                s.ccr_known == loaded_status_known && s.ccr == reset_known,
            "Native original-form loads/stores establish VNZ without inventing H/C; LDX retains "
            "reset-zero H");
  }
  require(cpu.background_requested() && memory.bytes[direct_store] == loaded_a,
          "Actual native store and BGND occur with partial CCR");
  cpu.restore_checkpoint(checkpoint);
  require(cpu.state().ccr_known == reset_known && !cpu.background_requested(),
          "Opaque CPU checkpoint retains unknown CCR bits");
  constexpr std::array<std::uint8_t, 1> transfer{0x85};
  memory.install(entry, transfer);
  require(rejected([&] { cpu.step(); }).contains("ccr-known=0x0068") && cpu.faulted(),
          "TPA cannot expose fabricated reset flags");
}

struct PartialExpected {
  unsigned opcode, known, value;
};

constexpr unsigned carry_known = 0x69;
constexpr unsigned tap_flags = 0x75;
constexpr unsigned interrupt_cleared = 0x60;
constexpr std::array<PartialExpected, 60> effects{
    {{.opcode = 0x0e, .known = 0x69, .value = 0x69},
     {.opcode = 0x58, .known = 0xef, .value = 0xec},
     {.opcode = 0x88, .known = 0x68, .value = 0x68},
     {.opcode = 0xcc, .known = 0x68, .value = 0x68},
     {.opcode = 0xbc, .known = 0x68, .value = 0x68},
     {.opcode = 0xdc, .known = 0x68, .value = 0x68},
     {.opcode = 0xec, .known = 0x68, .value = 0x68},
     {.opcode = 0xfc, .known = 0x68, .value = 0x68},
     {.opcode = 0x87, .known = 0x68, .value = 0x68},
     {.opcode = 0x89, .known = 0x68, .value = 0x68},
     {.opcode = 0x86, .known = 0x68, .value = 0x68},
     {.opcode = 0xe6, .known = 0xee, .value = 0x6c},
     {.opcode = 0xf1, .known = 0xef, .value = 0xed},
     {.opcode = 0xc1, .known = 0xef, .value = 0xed},
     {.opcode = 0xd6, .known = 0xee, .value = 0x6c},
     {.opcode = 0x0f, .known = 0x69, .value = 0x69},
     {.opcode = 0x4e, .known = 0xee, .value = 0x6c},
     {.opcode = 0xbe, .known = 0xee, .value = 0x6c},
     {.opcode = 0xce, .known = 0xee, .value = 0x6c},
     {.opcode = 0xb1, .known = 0xef, .value = 0xed},
     {.opcode = 0xa4, .known = 0xee, .value = 0x68},
     {.opcode = 0x4c, .known = 0xee, .value = 0x68},
     {.opcode = 0x5c, .known = 0xee, .value = 0x68},
     {.opcode = 0x8b, .known = 0x68, .value = 0x68},
     {.opcode = 0x8a, .known = 0x68, .value = 0x68},
     {.opcode = 0x97, .known = 0x68, .value = 0x68},
     {.opcode = 0x9f, .known = 0x68, .value = 0x68},
     {.opcode = 0xd7, .known = 0xee, .value = 0x68},
     {.opcode = 0x80, .known = 0xff, .value = 0x6b},
     {.opcode = 0x52, .known = 0x6b, .value = 0x6a},
     {.opcode = 0x4f, .known = loaded_status_known, .value = 0x6a},
     {.opcode = 0xae, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0xa6, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0xc7, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0x94, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0x9d, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0x82, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0x84, .known = byte_mask, .value = tap_flags},
     {.opcode = 0x9a, .known = reset_status_known, .value = interrupt_cleared},
     {.opcode = 0x9b, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0x98, .known = carry_known, .value = reset_status_known},
     {.opcode = 0x99, .known = carry_known, .value = carry_known},
     {.opcode = 0x45, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0x8c, .known = loaded_status_known, .value = 0x6a},
     {.opcode = 0x5f, .known = loaded_status_known, .value = 0x6a},
     {.opcode = 0x7f, .known = loaded_status_known, .value = 0x6a},
     {.opcode = 0x20, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0xaf, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0x35, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0xb7, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0x65, .known = 0xef, .value = 0x6d},
     {.opcode = 0x3f, .known = loaded_status_known, .value = 0x6a},
     {.opcode = 0x6e, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0xb6, .known = loaded_status_known, .value = 0x6c},
     {.opcode = 0xc6, .known = loaded_status_known, .value = 0x6c},
     {.opcode = 0xa5, .known = loaded_status_known, .value = reset_status_known},
     {.opcode = 0xa1, .known = 0xef, .value = 0x6d},
     {.opcode = 0xcd, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0xa7, .known = reset_status_known, .value = reset_status_known},
     {.opcode = 0x55, .known = loaded_status_known, .value = 0x6c}}};

void partial_registers(const Memory &memory, const Hcs08State &state, const unsigned opcode) {
  constexpr unsigned shift_x = 0x58;
  constexpr unsigned pull_x = 0x88;
  constexpr unsigned shifted_x = 0xf0;
  constexpr unsigned pulled_x = 0x5a;
  if (opcode == shift_x || opcode == pull_x) {
    const unsigned expected_x = opcode == shift_x ? shifted_x : pulled_x;
    require(state.hx == expected_x && state.a == loaded_a &&
                state.sp == loaded_sp + static_cast<unsigned>(opcode == pull_x),
            "Native ASLX/PULX preserve H/A and independently establish X/SP");
  }
  constexpr unsigned jmp_direct = 0xbc;
  constexpr unsigned jmp_extended = 0xcc;
  constexpr unsigned jmp_indexed16 = 0xdc;
  constexpr unsigned jmp_indexed8 = 0xec;
  constexpr unsigned jmp_indexed = 0xfc;
  const bool jump = opcode == jmp_direct || opcode == jmp_extended || opcode == jmp_indexed16 ||
                    opcode == jmp_indexed8 || opcode == jmp_indexed;
  if (jump) {
    constexpr unsigned absolute = 0x7800;
    unsigned target = loaded_x;
    if (opcode == jmp_extended) { target = absolute; }
    if (opcode == jmp_indexed16) { target = absolute + loaded_x; }
    if (opcode == jmp_indexed8) { target += loaded_x; }
    require(state.pc == target && state.hx == loaded_x && state.a == loaded_a &&
                state.sp == loaded_sp && state.hx_known == word_mask && state.a_known == byte_mask,
            "Five native JMP forms select independent destinations and preserve unrelated state");
  }
  constexpr unsigned ldhx_direct = 0x55;
  if (opcode == ldhx_direct) {
    require(state.hx == 0x8000 && state.pc == entry + 2 && state.a == loaded_a &&
                state.sp == loaded_sp,
            "Direct word load defines HX/VNZ and preserves A/SP and unknown H/C");
  }
  constexpr unsigned ldx_extended = 0xce;
  constexpr unsigned negative_byte = 0x80;
  if (opcode == ldx_extended) {
    require(state.hx == negative_byte && state.pc == entry + 3 && state.a == loaded_a &&
                state.sp == loaded_sp && state.hx_known == word_mask && state.a_known == byte_mask,
            "Extended byte load defines only X/VNZ, preserving H/A/SP and unknown H/C flags");
  }
  constexpr unsigned divide = 0x52;
  if (opcode == divide) {
    constexpr unsigned remainder_hx = 0x5578;
    require(state.a == 0 && state.hx == remainder_hx && state.pc == entry + 1 &&
                state.sp == loaded_sp && state.a_known == byte_mask && state.hx_known == word_mask,
            "Independent DIV85/120 gives quotient0/remainder85, preserves X/SP and unknown "
            "V/H/N while defining C0/Z1");
  }
  constexpr unsigned jsr = 0xcd;
  constexpr unsigned expected_subroutine = 0x7800;
  if (opcode == jsr) {
    require(state.pc == expected_subroutine && state.sp == loaded_sp - 2 &&
                memory.bytes[loaded_sp] == 3 && memory.bytes[loaded_sp - 1] == (entry >> 8U),
            "Partial-CCR JSR preserves flags and pushes the independent return PC descending");
  }
}

void partial_flags() {
  Memory memory;
  memory.writable.set();
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  unsigned accepted = 0;
  for (unsigned opcode = 0; opcode < byte_values; ++opcode) {
    const std::array<std::uint8_t, 3> code{static_cast<std::uint8_t>(opcode), loaded_x, 0};
    memory.install(entry, code);
    constexpr std::array<std::uint8_t, 2> negative{0x80, 0};
    constexpr unsigned extended_operand = 0x7800;
    memory.install(loaded_x, negative);
    memory.install(extended_operand, negative);
    constexpr unsigned indexed_operand = 0x7878;
    memory.install(indexed_operand, negative);
    constexpr unsigned indexed_byte_operand = 0xf0;
    memory.install(indexed_byte_operand, negative);
    constexpr unsigned rti = 0x80;
    constexpr unsigned pull_a = 0x86;
    constexpr unsigned pull_x = 0x88;
    if (opcode == pull_x) {
      constexpr std::array<std::uint8_t, 1> saved_x{0x5a};
      memory.install(loaded_sp + 1, saved_x);
    }
    if (opcode == rti || opcode == pull_a) {
      constexpr std::array<std::uint8_t, 5> frame{0x6b, 0x5a, 0x34, 0x80, 3};
      memory.install(loaded_sp + 1, frame);
    }
    cpu.set_state({.pc = entry,
                   .hx = loaded_x,
                   .sp = loaded_sp,
                   .a = loaded_a,
                   .ccr_known = reset_status_known,
                   .ccr = reset_status_known});
    const PartialExpected *effect = nullptr;
    for (const auto &candidate : effects) {
      if (candidate.opcode == opcode) {
        effect = &candidate;
        break;
      }
    }
    const auto before = cpu.state();
    const auto retired = memory.retired;
    if (effect != nullptr) {
      cpu.step();
      ++accepted;
      require(cpu.state().ccr_known == effect->known && cpu.state().ccr == effect->value &&
                  memory.retired == retired + 1,
              "Primary instruction effects establish only defined CCR bits and preserve unknown "
              "status");
      partial_registers(memory, cpu.state(), opcode);
    } else {
      require(rejected([&] { cpu.step(); }).contains("partial-CCR") && cpu.faulted() &&
                  cpu.state() == before && memory.retired == retired,
              "Every unqualified partial-CCR form rejects before architectural effects; no branch "
              "or TPA can consume fabricated flags");
    }
  }
  require(accepted == effects.size(),
          "All256 opcode classifications retain an independent explicit result");
}

void partial_zero() {
  Memory memory;
  constexpr std::array<std::uint8_t, 2> branch{0x27, 3};
  constexpr std::array<unsigned, 4> masks{0x68, 0x6a, 0x78, 0x7a};
  constexpr std::array<unsigned, 2> seeds{0x68, 0x7b};
  memory.install(entry, branch);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  for (const auto mask : masks) {
    for (const auto flags : seeds) {
      cpu.set_state({.pc = entry,
                     .ccr_known = static_cast<std::uint8_t>(mask),
                     .ccr = static_cast<std::uint8_t>(flags)});
      const auto before = cpu.state();
      const auto retired = memory.retired;
      if ((mask & flag_z) == 0) {
        require(rejected([&] { cpu.step(); }).contains("partial-CCR") && cpu.state() == before &&
                    memory.retired == retired,
                "BEQ cannot consume an unknown Z representative");
      } else {
        cpu.step();
        auto expected = before;
        expected.pc = static_cast<std::uint16_t>(entry + ((flags & flag_z) != 0 ? 5 : 2));
        require(cpu.state() == expected && memory.retired == retired + 1,
                "Independent taken/not-taken BEQ preserves partial CCR and unrelated registers");
      }
    }
  }
}

void partial_prefixes() {
  constexpr unsigned stack = 0x200;
  constexpr unsigned high = stack + 1;
  constexpr unsigned low_byte = stack + 2;
  constexpr unsigned prefix = 0x9e;

  struct PrefixExpected {
    unsigned opcode, a, hx, known, value, high, low_byte;
  };

  constexpr std::array<PrefixExpected, 8> prefix_effects{{{.opcode = 0xe7,
                                                           .a = 0,
                                                           .hx = loaded_x,
                                                           .known = 0xee,
                                                           .value = 0x6a,
                                                           .high = 0,
                                                           .low_byte = 0},
                                                          {.opcode = 0xef,
                                                           .a = 0,
                                                           .hx = loaded_x,
                                                           .known = 0xee,
                                                           .value = 0x68,
                                                           .high = loaded_x,
                                                           .low_byte = 0},
                                                          {.opcode = 0xfe,
                                                           .a = 0,
                                                           .hx = 0x8000,
                                                           .known = 0xee,
                                                           .value = 0x6c,
                                                           .high = 0x80,
                                                           .low_byte = 0},
                                                          {.opcode = 0xff,
                                                           .a = 0,
                                                           .hx = loaded_x,
                                                           .known = 0xee,
                                                           .value = 0x68,
                                                           .high = 0,
                                                           .low_byte = loaded_x},
                                                          {.opcode = 0xee,
                                                           .a = 0,
                                                           .hx = 0x80,
                                                           .known = 0xee,
                                                           .value = 0x6c,
                                                           .high = 0x80,
                                                           .low_byte = 0},
                                                          {.opcode = 0xe1,
                                                           .a = 0,
                                                           .hx = loaded_x,
                                                           .known = 0xef,
                                                           .value = 0xed,
                                                           .high = 0x80,
                                                           .low_byte = 0},
                                                          {.opcode = 0x68,
                                                           .a = 0,
                                                           .hx = loaded_x,
                                                           .known = 0xef,
                                                           .value = 0xeb,
                                                           .high = 0,
                                                           .low_byte = 0},
                                                          {.opcode = 0xeb,
                                                           .a = 0x80,
                                                           .hx = loaded_x,
                                                           .known = 0xff,
                                                           .value = 0x6c,
                                                           .high = 0x80,
                                                           .low_byte = 0}}};
  constexpr std::array<std::uint8_t, 2> data{0x80, 0};
  Memory memory;
  memory.writable.set();
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  for (unsigned second = 0; second < byte_values; ++second) {
    const std::array<std::uint8_t, 3> code{prefix, static_cast<std::uint8_t>(second), 1};
    memory.install(entry, code);
    memory.install(high, data);
    cpu.set_state({.pc = entry,
                   .hx = loaded_x,
                   .sp = stack,
                   .ccr_known = reset_status_known,
                   .ccr = reset_status_known});
    const auto before = cpu.state();
    const auto retired = memory.retired;
    const auto events = memory.accesses;
    const PrefixExpected *effect = nullptr;
    for (const auto &candidate : prefix_effects) {
      if (candidate.opcode == second) {
        effect = &candidate;
        break;
      }
    }
    if (effect != nullptr) {
      cpu.step();
      require(cpu.state() == Hcs08State{.pc = entry + 3,
                                        .hx = static_cast<std::uint16_t>(effect->hx),
                                        .sp = stack,
                                        .a = static_cast<std::uint8_t>(effect->a),
                                        .ccr_known = static_cast<std::uint8_t>(effect->known),
                                        .ccr = static_cast<std::uint8_t>(effect->value)} &&
                  memory.bytes[high] == effect->high &&
                  memory.bytes[low_byte] == effect->low_byte && memory.retired == retired + 1,
              "Independent SP1 effects preserve unknown H/C except flags actually defined");
    } else {
      require(rejected([&] { cpu.step(); }).contains("partial-CCR") && cpu.faulted() &&
                  cpu.state() == before && memory.retired == retired && memory.accesses == events,
              "Every other prefixed form refuses before register, RAM or retirement effects");
    }
  }
  constexpr std::array<std::uint8_t, 1> truncated{prefix};
  for (const unsigned pc : {entry, word_mask}) {
    memory.known.reset();
    memory.install(pc, truncated);
    cpu.set_state({.pc = static_cast<std::uint16_t>(pc),
                   .ccr_known = reset_status_known,
                   .ccr = reset_status_known});
    const auto before = cpu.state();
    const auto retired = memory.retired;
    require(rejected([&] { cpu.step(); }).contains(pc == entry ? "fetch" : "wraps") &&
                cpu.state() == before && memory.retired == retired,
            "Unknown or wrapped second opcode cannot qualify a prefixed instruction");
  }
}

void partial_carry() {
  Memory memory;
  memory.writable.set(operand);
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  constexpr std::array<unsigned, 4> masks{0x68, 0x69, 0x78, 0x79};
  constexpr std::array<unsigned, 2> seeds{0x68, 0x79};
  constexpr std::array<std::uint8_t, 3> clear_opcodes{0x8c, 0x5f, 0x7f};
  constexpr unsigned defined_vnz = 0x86;
  for (const auto mask : masks) {
    for (const unsigned flags : seeds) {
      for (const auto opcode : clear_opcodes) {
        const std::array code{opcode};
        memory.install(entry, code);
        cpu.set_state({.pc = entry,
                       .hx = operand,
                       .ccr_known = static_cast<std::uint8_t>(mask),
                       .ccr = static_cast<std::uint8_t>(flags)});
        cpu.step();
        require(cpu.state().ccr_known == (mask | defined_vnz) &&
                    cpu.state().ccr == ((flags & mask) | flag_z),
                "CLRH/CLRX/CLR indexed preserve both values and unknownness of H/C");
      }
      constexpr std::array<std::uint8_t, 2> branch{0x25, 3};
      memory.install(entry, branch);
      cpu.set_state({.pc = entry,
                     .ccr_known = static_cast<std::uint8_t>(mask),
                     .ccr = static_cast<std::uint8_t>(flags)});
      const auto before = cpu.state();
      const auto retired = memory.retired;
      const auto events = memory.accesses;
      if ((mask & flag_c) != 0) {
        cpu.step();
        auto expected = before;
        expected.pc = static_cast<std::uint16_t>(entry + ((flags & flag_c) != 0 ? 5 : 2));
        require(cpu.state() == expected && memory.retired == retired + 1 &&
                    memory.accesses == events,
                "BCS consumes only known C; literal taken/not-taken PCs preserve all flags");
      } else {
        require(rejected([&] { cpu.step(); }).contains("partial-CCR") && cpu.state() == before &&
                    memory.retired == retired && memory.accesses == events,
                "BCS cannot consume an unknown carry, even if its representative is zero");
      }
    }
  }
}

void bdc_ccr() {
  namespace wire = blofeld::test::hcs08_wire;
  for (const bool write : {false, true}) {
    wire::Target target;
    auto s = target.cpu.state();
    s.ccr_known = reset_status_known;
    s.ccr = byte_mask;
    target.cpu.set_state(s);
    Hcs08Bdc bdc(target.cpu, target.mapping, wire::chip, Hcs08BdcReset::bkgd_held_low);
    constexpr std::array<std::uint8_t, 3> pc{0x4b, 0x80, 0};
    blofeld::test::hcs08_bdc::command(bdc, pc);
    require(target.cpu.state().ccr_known == reset_status_known,
            "PC writes preserve actual CCR unknownness");
    if (write) {
      constexpr std::array<std::uint8_t, 2> ccr{0x49, 9};
      blofeld::test::hcs08_bdc::command(bdc, ccr);
      constexpr std::array<std::uint8_t, 1> read{0x69};
      blofeld::test::hcs08_bdc::command(bdc, read);
      require(bdc.read_reply() == written_status && target.cpu.state().ccr_known == byte_mask,
              "Actual WRITE_CCR establishes bits before native READ_CCR reply");
    } else {
      constexpr std::array<std::uint8_t, 1> read{0x69};
      const auto error = rejected([&] { blofeld::test::hcs08_bdc::command(bdc, read); });
      require(error.contains("unknown CPU CCR bits mask=0x0097") &&
                  error.contains("ccr-known=0x0068") && bdc.faulted(),
              "BDC cannot serialize unknown reset flags as a successful byte");
    }
  }
}
}

int main() {
  try {
    reset_edges();
    reset_replay_and_failures();
    native_reset();
    bdc_ccr();
    partial_flags();
    partial_prefixes();
    partial_carry();
    partial_zero();
    std::cout << "HCS08 reset edges, native register/CCR effects and BDC unknownness verified.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
