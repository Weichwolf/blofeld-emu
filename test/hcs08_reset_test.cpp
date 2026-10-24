// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/hcs08.hpp"
#include "blofeld/hcs08_bdc.hpp"
#include "blofeld/hcs08_reset.hpp"
#include "hcs08_bdc_fixture.hpp"
#include "hcs08_fixture.hpp"
#include "hcs08_wire_fixture.hpp"

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

void partial_flags() {
  struct Expected {
    unsigned opcode, known, value;
  };

  constexpr unsigned carry_known = 0x69;
  constexpr unsigned tap_flags = 0x75;
  constexpr unsigned interrupt_cleared = 0x60;
  constexpr std::array<Expected, 11> effects{
      {{.opcode = 0xae, .known = loaded_status_known, .value = reset_status_known},
       {.opcode = 0xa6, .known = loaded_status_known, .value = reset_status_known},
       {.opcode = 0xc7, .known = loaded_status_known, .value = reset_status_known},
       {.opcode = 0x94, .known = reset_status_known, .value = reset_status_known},
       {.opcode = 0x9d, .known = reset_status_known, .value = reset_status_known},
       {.opcode = 0x82, .known = reset_status_known, .value = reset_status_known},
       {.opcode = 0x84, .known = byte_mask, .value = tap_flags},
       {.opcode = 0x9a, .known = reset_status_known, .value = interrupt_cleared},
       {.opcode = 0x9b, .known = reset_status_known, .value = reset_status_known},
       {.opcode = 0x98, .known = carry_known, .value = reset_status_known},
       {.opcode = 0x99, .known = carry_known, .value = carry_known}}};
  Memory memory;
  memory.writable.set();
  Hcs08 cpu(memory, BLOFELD_HCS08_SLA);
  unsigned accepted = 0;
  for (unsigned opcode = 0; opcode < byte_values; ++opcode) {
    const std::array<std::uint8_t, 3> code{static_cast<std::uint8_t>(opcode), loaded_x, 0};
    memory.install(entry, code);
    cpu.set_state({.pc = entry,
                   .hx = loaded_x,
                   .sp = loaded_sp,
                   .a = loaded_a,
                   .ccr_known = reset_status_known,
                   .ccr = reset_status_known});
    const Expected *effect = nullptr;
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
      require(rejected([&] {
                blofeld::test::hcs08_bdc::command(bdc, read);
              }).contains("unknown CPU CCR") &&
                  bdc.faulted(),
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
    std::cout << "HCS08 reset edges, native register/CCR effects and BDC unknownness verified.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
