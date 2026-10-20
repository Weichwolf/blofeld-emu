// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_machine.hpp"
#include "blofeld/dsp_native_access.hpp"
#include "dsp_norm_fixture.hpp"
#include "native_access_fixture.hpp"

#include <exception>
#include <iostream>

namespace blofeld::test::native_access {
namespace {
void fetch_classification() {
  DspMachine machine;
  DspProgram code;
  code.immediate("x0", first);
  code.emit("nop"); // The following ROM word is unknown decoder lookahead.
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  Recorder capture;
  capture.step(machine);
  require(capture.rows.size() == 4 && machine.cpu().state().x == first &&
              machine.cpu().state().pc == boot + 2 &&
              machine.cpu().debug_state().native_cycles == 2,
          "One real long MOVE fetches only its opcode and consumed extension");
  pair(capture.rows, {.kind = Kind::opcode, .target = {.address = boot}, .value = immediate_x0});
  pair(capture.rows,
       {.kind = Kind::extension,
        .target = {.address = boot + 1},
        .value = first,
        .offset = 2,
        .ordinal = 1});
  capture.step(machine);
  require(capture.rows.size() == 2 && machine.cpu().state().pc == boot + 3 &&
              machine.cpu().debug_state().native_cycles == 3,
          "A one-word NOP neither reads nor reports its unknown lookahead");
  pair(capture.rows,
       {.kind = Kind::opcode, .target = {.address = boot + 2}, .value = 0, .pc = boot + 2});
}

void data_and_parallel() {
  DspMachine machine;
  DspProgram code;
  code.immediate("m0", word_mask);
  code.immediate("m4", word_mask);
  code.immediate("r0", data);
  code.immediate("r4", second_data);
  code.immediate("a", first);
  code.immediate("b", second);
  const auto store_pc = code.pc();
  constexpr unsigned parallel_store_ab = 0xbb1800; // FM parallel A->X:(R0)+, B->Y:(R4)+.
  code.word(parallel_store_ab);
  const auto load_pc = code.pc();
  code.emit("move x:(r0)-,x0");
  const auto program_pc = code.pc();
  // FM13-133: W=1, MMMRRR=110000 (absolute extension), dddddd=000100 (X0).
  constexpr unsigned absolute_p_to_x0 = 0x07f084;
  code.word(absolute_p_to_x0);
  code.word(0x800);
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  machine.bus().write({.space = DspSpace::x, .address = data + 1, .value = second});
  machine.bus().write({.address = 0x800, .value = first});
  execute(machine, 6);
  Recorder capture;
  capture.step(machine);
  require(capture.rows.size() == 6 && machine.cpu().state().r[0] == data + 1 &&
              machine.cpu().state().r[4] == second_data + 1 &&
              machine.bus().read({.space = DspSpace::x, .address = data}) == first &&
              machine.bus().read({.space = DspSpace::y, .address = second_data}) == second,
          "Both actual parallel stores and post-increments happen once");
  pair(capture.rows,
       {.kind = Kind::write,
        .target = {.space = DspSpace::x, .address = data},
        .value = first,
        .offset = 2,
        .ordinal = 1,
        .pc = store_pc});
  pair(capture.rows,
       {.kind = Kind::write,
        .target = {.space = DspSpace::y, .address = second_data},
        .value = second,
        .offset = 4,
        .ordinal = 2,
        .pc = store_pc});
  capture.step(machine);
  require(capture.rows.size() == 4 && machine.cpu().state().x == second &&
              machine.cpu().state().r[0] == data,
          "The load uses its old R0 then decrements exactly once");
  pair(capture.rows,
       {.kind = Kind::read,
        .target = {.space = DspSpace::x, .address = data + 1},
        .value = second,
        .offset = 2,
        .ordinal = 1,
        .pc = load_pc});
  capture.step(machine);
  require(capture.rows.size() == 6 && machine.cpu().state().x == first,
          "P-space data is distinct from a real extension fetch");
  pair(capture.rows,
       {.kind = Kind::extension,
        .target = {.address = program_pc + 1},
        .value = 0x800,
        .offset = 2,
        .ordinal = 1,
        .pc = program_pc});
  pair(capture.rows,
       {.kind = Kind::read,
        .target = {.address = 0x800},
        .value = first,
        .offset = 4,
        .ordinal = 2,
        .pc = program_pc});
}

void peripheral_routes() {
  constexpr unsigned counter = 0xffff8c;
  DspMachine machine;
  DspProgram code;
  code.emit("bset #1,x:<<$ffffd5");
  code.emit("btst #0,x:<<$ffff8c"); // Counter register uses the normal word fallback.
  code.emit("btst #0,x:<<$ffff94"); // HRX: one destructive FIFO read, no fake bit probe.
  code.install(machine.bus());
  machine.cpu().reset_spi_boot();
  constexpr unsigned port_direction = 0xffffd6;
  constexpr unsigned port_data = 0xffffd5;
  // Both outputs retain their documented reset data; bit0 must survive the RMW.
  machine.bus().write({.space = DspSpace::x, .address = port_direction, .value = 3});
  Recorder capture;
  capture.step(machine);
  require(capture.rows.size() == 4 &&
              machine.bus().read({.space = DspSpace::x, .address = port_data}) == 2,
          "The specialized native bit route reports one actual RMW and its previous bit");
  pair(capture.rows,
       {.kind = Kind::bit,
        .target = {.space = DspSpace::x, .address = port_data},
        .value = 0,
        .bit = DspNativeAccessBit{.index = 1, .operation = DspBitOperation::set},
        .offset = 2,
        .ordinal = 1});
  capture.step(machine);
  require(capture.rows.size() == 4, "A declined bit route emits no speculative access event");
  pair(capture.rows,
       {.kind = Kind::read,
        .target = {.space = DspSpace::x, .address = counter},
        .value = 0,
        .offset = 2,
        .ordinal = 1,
        .pc = boot + 1});
  const auto error = fault([&] { capture.step(machine); });
  require(error.contains("empty DSP SHI FIFO") && capture.rows.size() == 3 &&
              capture.rows.back().kind == Kind::read &&
              capture.rows.back().stage == Stage::before && !capture.rows.back().value,
          "An empty FIFO has one real attempted read and no fabricated completed bit/value");
}
}
}

int main() {
  using namespace blofeld::test::native_access;
  try {
    fetch_classification();
    data_and_parallel();
    peripheral_routes();
    failure_checks();
    continuation_checks();
    std::cout << "Native access kinds, strict I/O, failure prefixes and continuations passed.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
