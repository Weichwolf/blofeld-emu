// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_gpio.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp_core_fixture.hpp"
#include "dsp_rom_program.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld::test::core;
using blofeld::DspSpace;
using blofeld::test::DspRomRegister;
constexpr unsigned boot = 0xff0000;
constexpr unsigned reset_omr = 0x305; // FM figure 5-3/table 5-2, mode-five pins.
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned data = 0x100;
constexpr unsigned input = 0x123456;
constexpr unsigned second_input = 0xabcdef;
constexpr unsigned word_bits = 24;
constexpr unsigned last_switched = 0x2fff;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned negative_address = 0x21;
constexpr unsigned next_repeat_address = 0x201;
constexpr unsigned timer0_status = 0xffff8f, timer1_status = 0xffff8b, timer1_count = 0xffff88;
constexpr unsigned c_data = 0xffffbd, c_direction = 0xffffbe, c_control = 0xffffbf;
constexpr unsigned f_data = 0xfffff5, f_direction = 0xfffff6, port_f_bcr = 0xfffffb;
constexpr unsigned port_f_bcr_enabled = 0x01ffff, e_data = 0xffff9d, e_control = 0xffff9f;
constexpr unsigned shi_rate = 0xffff90, shi_control = 0xffff91, shi_receive = 0xffff94;
constexpr unsigned shi_fifo_enabled = 0xa9, shi_polled_words = 0x29, shi_fifo_bit = 17;
constexpr unsigned gpio_pin_mask = 0xfff, initial_c_latch = 0x615, adjacent_ram = 0x21;
constexpr unsigned retained_layout = 0x600085, retained_r0 = 0x55;

void require(const bool value, const std::string &message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

void send_word(blofeld::DspShi &shi, const unsigned value) {
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool level = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
}

void test_memory_boundaries_rom_and_retained_switch() {
  blofeld::DspBus bus;
  constexpr std::array<blofeld::DspAddress, 10> edges{
      {{.address = 0},
       {.address = 0xfff},
       {.space = blofeld::DspSpace::x, .address = 0},
       {.space = blofeld::DspSpace::x, .address = 0x2fff},
       {.space = blofeld::DspSpace::x, .address = 0xc000},
       {.space = blofeld::DspSpace::x, .address = 0x11fff},
       {.space = blofeld::DspSpace::y, .address = 0},
       {.space = blofeld::DspSpace::y, .address = 0x1fff},
       {.space = blofeld::DspSpace::y, .address = 0xc000},
       {.space = blofeld::DspSpace::y, .address = 0x15fff}}};
  for (const auto edge : edges) {
    reject([&bus, edge] { static_cast<void>(bus.read(edge)); },
           "Cold DSP RAM is unknown at every boundary");
    bus.write({.space = edge.space, .address = edge.address, .value = input});
    require(bus.read(edge) == input, "Each documented RAM boundary stores a full 24-bit word");
  }
  constexpr std::array<blofeld::DspAddress, 6> holes{
      {{.address = 0x1000},
       {.address = 0xfeffff},
       {.space = blofeld::DspSpace::x, .address = 0x3000},
       {.space = blofeld::DspSpace::x, .address = 0x12000},
       {.space = blofeld::DspSpace::y, .address = 0x2000},
       {.space = blofeld::DspSpace::y, .address = 0x16000}}};
  for (const auto hole : holes) {
    reject([&bus, hole] { static_cast<void>(bus.read(hole)); },
           "Reserved ranges never alias other RAM");
  }
  const std::array<std::uint32_t, 2> own_rom{input, second_input};
  for (const auto area : {blofeld::DspSpace::p, blofeld::DspSpace::x, blofeld::DspSpace::y}) {
    const unsigned base = area == blofeld::DspSpace::p ? boot : 0x4000;
    bus.install_rom({.space = area, .start = base, .words = own_rom});
    require(bus.read({.space = area, .address = base}) == input,
            "ROM inputs preserve independent spaces");
    reject([&bus, area, base] { bus.write({.space = area, .address = base, .value = 0}); },
           "Runtime stores cannot rewrite installed ROM");
    reject(
        [&bus, area, base] { static_cast<void>(bus.read({.space = area, .address = base + 2})); },
        "Adjacent missing ROM contents remain unknown");
  }
  bus.write({.space = blofeld::DspSpace::x, .address = 0x10000, .value = input});
  constexpr unsigned switched_omr = 0x600080;
  bus.observe_omr(switched_omr);
  bus.advance_instruction_cycles(2);
  reject([&bus] { static_cast<void>(bus.read({.address = 0x1000})); },
         "Switch cannot complete early");
  bus.advance_instruction_cycles(1);
  require(bus.read({.address = 0x1000}) == input && bus.read({.address = last_switched}) == input,
          "MS/MSW=111 maps the preserved upper eight X RAM blocks into P");
  bus.write({.address = 0x1000, .value = second_input});
  bus.observe_omr(0);
  bus.advance_instruction_cycles(3);
  require(bus.read({.space = blofeld::DspSpace::x, .address = 0x10000}) == second_input,
          "Switching back exposes the same physical RAM contents");
  bus.reset();
  require(bus.read({.space = blofeld::DspSpace::x, .address = 0x10000}) == second_input,
          "External reset retains initialized physical RAM");
}

void test_native_reset_fetch_fault_and_arithmetic() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  const std::array<std::uint32_t, 1> nop{0};
  bus.install_rom({.start = boot, .words = nop});
  Driver cpu(machine);
  require(cpu.state().pc == boot && cpu.state().omr == reset_omr && cpu.state().sp == 0,
          "SPI boot reset enters the documented P:ff0000 mode-five vector");
  cpu.step();
  require(cpu.state().pc == boot + 1, "One-word NOP does not require known lookahead data");
  bool unknown = false;
  try {
    cpu.step();
  } catch (const blofeld::Fault &fault) {
    const std::string message = fault.what();
    require(message.find("pc=0xff0001") != std::string::npos &&
                message.find("P:0xff0001") != std::string::npos,
            "Missing ROM reports CPU context and memory operation/address");
    unknown = true;
  }
  require(unknown, "Missing program contents cannot become an executable zero/NOP");
  reject([&cpu] { cpu.step(); }, "Faulted reference state cannot silently resume");
  const std::array<std::uint32_t, 1> missing_extension{immediate_x0};
  bus.install_rom({.start = boot, .words = missing_extension});
  cpu.hardware_reset();
  reject([&cpu] { cpu.step(); }, "A required two-word operand cannot use zero-filled lookahead");
  Program program;
  program.immediate(DspRomRegister::x0, input);
  require(program.words()[0] == immediate_x0 && program.words()[1] == input,
          "Immediate X0 encoding agrees with the independent instruction reference");
  program.immediate(DspRomRegister::x1, 0x10);
  program.emit(clear_a);
  program.emit(add_x0_a);
  program.emit(add_x1_a);
  program.emit(store_a_absolute, 0x20);
  program.emit(negate_a);
  program.emit(store_a_absolute, negative_address);
  program.install(bus);
  cpu.hardware_reset();
  for (unsigned index = 0; index < program.instructions(); ++index) { cpu.step(); }
  constexpr unsigned sum = 0x123466;
  constexpr unsigned negative = 0xedcb9a;
  constexpr std::uint64_t accumulator = 0xffedcb9a000000;
  require(bus.read({.space = blofeld::DspSpace::x, .address = 0x20}) == sum &&
              bus.read({.space = blofeld::DspSpace::x, .address = negative_address}) == negative,
          "Native addition/negation and data stores match independent 24-bit results");
  require(cpu.state().a == accumulator && (cpu.state().sr & 12U) == 8,
          "Canonical 56-bit negative accumulator exposes independently expected N/Z flags");
}

void test_native_code_write_and_decoder_invalidation() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program start;
  start.immediate(DspRomRegister::x0, 0);
  start.immediate(DspRomRegister::r0, 0x100);
  start.emit(store_x0_program_r0);
  start.jump(data);
  start.install(bus);
  Driver cpu(machine);
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  require(cpu.state().pc == data + 1, "Writing unknown P RAM does not require a synthetic preread");
  Program loop;
  loop.immediate(DspRomRegister::x0, 1);
  loop.jump(data);
  unsigned offset = data;
  for (const auto value : loop.words()) { bus.write({.address = offset++, .value = value}); }
  cpu.hardware_reset();
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step(); // The reset diagnostic writes its own first NOP again.
  offset = data;
  for (const auto value : loop.words()) { bus.write({.address = offset++, .value = value}); }
  cpu.step();
  cpu.step();
  bus.write({.address = data + 1, .value = 2});
  cpu.step();
  cpu.step();
  require((cpu.state().x & word_mask) == 2, "Changed P operand cannot execute from stale state");
  Program replacement;
  replacement.immediate(DspRomRegister::x1, 3);
  bus.write({.address = data, .value = replacement.words()[0]});
  bus.write({.address = data + 1, .value = replacement.words()[1]});
  cpu.step();
  constexpr std::uint64_t changed = 0x3000002;
  require(cpu.state().x == changed, "Changing a cached P opcode changes its destination register");
}

void test_native_timer_input_poll_and_reset() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.bit({.operand = {.space = DspSpace::x, .address = timer1_status},
               .index = 0,
               .action = BitAction::test});
  const auto wait = program.pc();
  program.branch(
      {.operand = {.space = DspSpace::x, .address = timer1_status}, .index = 12, .target = 0});
  const auto ready = program.pc();
  program.receive({.space = DspSpace::x, .address = timer1_count}, DspRomRegister::x0);
  program.install(bus);
  Driver cpu(machine);
  cpu.step();
  require((cpu.state().sr & 1U) == 0,
          "Known TE can be tested while the separate live DI is unknown");
  reject([&cpu] { cpu.step(); }, "A native NAND-style ready poll cannot fabricate unknown TIO1");
  bus.timers().set_tio_drive(1, false);
  cpu.hardware_reset();
  cpu.step();
  const auto status = cpu.state().sr;
  for (unsigned index = 0; index < 3; ++index) {
    cpu.step();
    require(cpu.state().pc == wait && cpu.state().sr == status,
            "Native BRCLR polls the observed low DI without changing flags");
  }
  bus.timers().set_tio_drive(1, true);
  cpu.step();
  require(cpu.state().pc == ready,
          "Only an explicit high TIO1 observation releases the ready loop");
  cpu.step();
  require((cpu.state().x & word_mask) == 0, "GPIO polling does not invent timer counter progress");
  cpu.hardware_reset();
  constexpr unsigned tcsr1 = 0xffff8b;
  constexpr unsigned input_bit = 0x1000;
  require(bus.timers().read(tcsr1, input_bit) == input_bit && bus.timers().read(tcsr1, 1) == 0,
          "Hardware reset clears timer control while preserving the externally held ready level");
}

void test_native_timer_w1c_word_writeback() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.bit({.operand = {.space = DspSpace::x, .address = timer0_status},
               .index = 13,
               .action = BitAction::set});
  program.receive({.space = DspSpace::x, .address = timer0_status}, DspRomRegister::x0);
  program.install(bus);
  Driver cpu(machine);
  constexpr unsigned tcsr0 = 0xffff8f;
  constexpr unsigned tlr0 = 0xffff8e;
  constexpr unsigned tcpr0 = 0xffff8d;
  constexpr unsigned tcr0 = 0xffff8c;
  constexpr unsigned flags = 0x300000;
  constexpr unsigned data_output = 0x2000;
  bus.timers().write({.address = tlr0, .value = word_mask});
  bus.timers().write({.address = tcpr0, .value = word_mask});
  bus.timers().write({.address = tcsr0, .value = 7});
  bus.timers().advance_clk2(2);
  require(bus.timers().read(tcsr0, flags) == flags,
          "Known compare followed by wrap sets both W1C flags");
  cpu.step();
  require(
      (cpu.state().sr & 1U) == 0 && bus.timers().read(tcsr0, data_output) == data_output &&
          bus.timers().read(tcsr0, flags) == 0 && bus.timers().interrupt_vectors().empty(),
      "BSET DO performs full TCSR writeback and clears both read-as-one W1C flags per NXP warning");
  require(bus.timers().read(tcr0) == 0,
          "One CPU reference step contributes no implicit CLK/2 event");
  reject([&cpu] { cpu.step(); },
         "Successful control RMW cannot make a live unknown DI word-readable");
}

void test_native_timer_read_only_input_bit() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.bit({.operand = {.space = DspSpace::x, .address = timer1_status},
               .index = 12,
               .action = BitAction::clear});
  program.bit({.operand = {.space = DspSpace::x, .address = timer1_status},
               .index = 12,
               .action = BitAction::test});
  program.install(bus);
  Driver cpu(machine);
  bus.timers().set_tio_drive(1, true);
  for (unsigned index = 0; index < program.instructions(); ++index) {
    cpu.step();
    require((cpu.state().sr & 1U) == 1,
            "BCLR DI observes old high Carry but cannot clear the read-only pin input");
  }
}

void test_native_symbolic_gpio_bits_and_effective_address() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.write({.space = DspSpace::x, .address = c_direction, .value = 1});
  program.write({.space = DspSpace::x, .address = c_data, .value = 1});
  program.write({.space = DspSpace::x, .address = c_direction, .value = 3});
  program.bit({.operand = {.space = DspSpace::x, .address = c_data},
               .index = 0,
               .action = BitAction::clear});
  program.bit(
      {.operand = {.space = DspSpace::x, .address = c_data}, .index = 0, .action = BitAction::set});
  program.bit({.operand = {.space = DspSpace::x, .address = c_data},
               .index = 0,
               .action = BitAction::change});
  program.bit({.operand = {.space = DspSpace::x, .address = c_data},
               .index = 0,
               .action = BitAction::test});
  program.immediate(DspRomRegister::r0, c_data);
  program.emit(change_r0_post);
  program.bit(
      {.operand = {.space = DspSpace::x, .address = c_data}, .index = 1, .action = BitAction::set});
  program.install(bus);
  Driver cpu(machine);
  for (unsigned index = 0; index < 3; ++index) { cpu.step(); }
  constexpr std::array<unsigned, 4> carries{1, 0, 1, 0};
  constexpr std::array<unsigned, 4> levels{0, 1, 0, 0};
  constexpr unsigned pdr = 0xffffbd;
  for (std::size_t index = 0; index < carries.size(); ++index) {
    cpu.step();
    const auto output = bus.gpio(blofeld::DspPort::c).outputs();
    require((cpu.state().sr & 1U) == carries.at(index) && output.mask == 3 && output.known == 1 &&
                output.levels == levels.at(index),
            "BCLR/BSET/BCHG/BTST report the previous bit and preserve unrelated unknown outputs");
    reject([&bus] { static_cast<void>(bus.read({.space = blofeld::DspSpace::x, .address = pdr})); },
           "Successful bit operations cannot qualify an unknown whole PDR word");
  }
  cpu.step();
  cpu.step();
  constexpr unsigned prr = 0xffffbe;
  require(cpu.state().r[0] == prr && (cpu.state().sr & 1U) == 0 &&
              bus.gpio(blofeld::DspPort::c).outputs().levels == 1 &&
              bus.gpio(blofeld::DspPort::c).read(blofeld::DspGpioRegister::direction) == 3,
          "EA postincrement happens once and writeback uses the captured PDR address");
  reject([&cpu] { cpu.step(); }, "An unknown selected bit cannot supply the required carry value");
  require(bus.gpio(blofeld::DspPort::c).outputs().known == 1,
          "Rejected bit access does not invent the unknown output's previous value");
}

void test_native_symbolic_port_f_pp_bits() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.write({.space = DspSpace::x, .address = port_f_bcr, .value = port_f_bcr_enabled});
  program.write({.space = DspSpace::y, .address = f_direction, .value = 1});
  program.write({.space = DspSpace::y, .address = f_data, .value = 1});
  program.write({.space = DspSpace::y, .address = f_direction, .value = 3});
  program.bit({.operand = {.space = DspSpace::y, .address = f_data},
               .index = 0,
               .action = BitAction::clear});
  program.bit({.operand = {.space = DspSpace::y, .address = f_data},
               .index = 0,
               .action = BitAction::change});
  program.bit({.operand = {.space = DspSpace::y, .address = f_data},
               .index = 0,
               .action = BitAction::test});
  program.install(bus);
  Driver cpu(machine);
  for (unsigned index = 0; index < 4; ++index) { cpu.step(); }
  constexpr std::array<unsigned, 3> carries{1, 0, 1};
  constexpr std::array<unsigned, 3> levels{0, 1, 1};
  for (std::size_t index = 0; index < carries.size(); ++index) {
    cpu.step();
    const auto output = bus.gpio(blofeld::DspPort::f).outputs();
    require((cpu.state().sr & 1U) == carries.at(index) && output.mask == 3 && output.known == 1 &&
                output.levels == levels.at(index),
            "Y pp-addressed Port F bits preserve unknown latches after explicit BIU setup");
  }
}

void test_native_symbolic_bit_branches() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.write({.space = DspSpace::x, .address = c_control, .value = 4});
  const auto wait = program.pc();
  program.branch({.operand = {.space = DspSpace::x, .address = c_data}, .index = 2, .target = 0});
  const auto after_wait = program.pc();
  const auto landing = after_wait + 3;
  program.branch({.operand = {.space = DspSpace::x, .address = c_data},
                  .index = 2,
                  .target = landing,
                  .kind = BranchKind::jump_set});
  program.receive({.space = DspSpace::x, .address = c_data}, DspRomRegister::x0);
  require(program.pc() == landing,
          "Independent branch length skips one deliberately unknown MOVEP");
  program.emit(0);
  program.bit({.operand = {.space = DspSpace::x, .address = c_data},
               .index = 2,
               .action = BitAction::test});
  program.install(bus);
  Driver cpu(machine);
  cpu.step();
  bus.gpio(blofeld::DspPort::c).set_drive({.mask = 4});
  const auto status = cpu.state().sr;
  cpu.step();
  require(cpu.state().pc == wait && cpu.state().sr == status,
          "BRCLR samples only its known input bit and does not modify CCR");
  bus.gpio(blofeld::DspPort::c).set_drive({.floating = 4});
  cpu.step();
  require(cpu.state().pc == wait && cpu.state().sr == status,
          "Native BRCLR keeps waiting on an explicitly floating Port C pull-down");
  bus.gpio(blofeld::DspPort::c).set_drive({.mask = 4, .levels = 4});
  cpu.step();
  require(cpu.state().pc == after_wait, "Observed high input releases the original-style bit poll");
  cpu.step();
  require(cpu.state().pc == landing && cpu.state().sr == status,
          "JSET reads the same masked pin and skips the unknown whole-word read");
  cpu.step();
  bus.gpio(blofeld::DspPort::c).set_drive({});
  reject([&cpu] { cpu.step(); }, "A released input cannot become a fabricated successful bit test");
}

void test_native_highest_bit_and_rejected_encoding() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.bit({.operand = {.space = DspSpace::x, .address = 0x20},
               .index = word_bits - 1,
               .action = BitAction::test});
  program.bit({.operand = {.space = DspSpace::x, .address = 0x20},
               .index = word_bits - 1,
               .action = BitAction::change});
  program.bit({.operand = {.space = DspSpace::x, .address = 0x20},
               .index = word_bits - 1,
               .action = BitAction::set});
  program.bit({.operand = {.space = DspSpace::x, .address = 0x20},
               .index = word_bits - 1,
               .action = BitAction::clear});
  const auto invalid_pc = program.pc();
  program.bit(
      {.operand = {.space = DspSpace::x, .address = 0x20}, .index = 0, .action = BitAction::test});
  program.install(bus);
  // DSP56300FM BTST's low five opcode bits are bbbbb; 24 is outside the 0..23 range.
  constexpr unsigned bit_field = 0x1f;
  constexpr unsigned invalid_index = 24;
  const std::array<std::uint32_t, 1> invalid{(program.words().back() & ~bit_field) | invalid_index};
  bus.install_rom({.start = invalid_pc, .words = invalid});
  constexpr unsigned target = 0x20;
  constexpr unsigned high_bit = 0x800000;
  bus.write({.space = blofeld::DspSpace::x, .address = target, .value = high_bit});
  Driver cpu(machine);
  constexpr std::array<unsigned, 4> carries{1, 1, 0, 1};
  constexpr std::array<unsigned, 4> words{high_bit, 0, high_bit, 0};
  for (std::size_t index = 0; index < carries.size(); ++index) {
    cpu.step();
    require((cpu.state().sr & 1U) == carries.at(index) &&
                bus.read({.space = blofeld::DspSpace::x, .address = target}) == words.at(index),
            "Bit 23 remains a valid 24-bit source/result boundary for all four bit operations");
  }
  require(cpu.state().pc == invalid_pc, "Reserved bit-number encoding is the next native opcode");
  reject([&cpu] { cpu.step(); }, "Native reserved bit number 24 must fail before any data shift");
  require(bus.read({.space = blofeld::DspSpace::x, .address = target}) == 0,
          "Rejected bit encoding cannot modify RAM");
}

void test_native_bit_ram_and_fifo_fallback() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.immediate(DspRomRegister::r0, 0x20);
  program.emit(set_r0_post | 2U);
  program.bit(
      {.operand = {.space = DspSpace::x, .address = 0x20}, .index = 7, .action = BitAction::clear});
  program.bit({.operand = {.space = DspSpace::x, .address = adjacent_ram},
               .index = 0,
               .action = BitAction::change});
  program.bit({.operand = {.space = DspSpace::x, .address = adjacent_ram},
               .index = 0,
               .action = BitAction::test});
  program.write({.space = DspSpace::x, .address = shi_control, .value = shi_fifo_enabled});
  program.bit({.operand = {.space = DspSpace::x, .address = shi_receive},
               .index = 0,
               .action = BitAction::test});
  program.receive({.space = DspSpace::x, .address = shi_receive}, DspRomRegister::x0);
  program.install(bus);
  constexpr unsigned first_address = 0x20;
  constexpr unsigned second_address = 0x21;
  constexpr unsigned initial_value = 0x80;
  constexpr unsigned set_value = 0x84;
  bus.write({.space = blofeld::DspSpace::x, .address = first_address, .value = initial_value});
  bus.write({.space = blofeld::DspSpace::x, .address = second_address, .value = 1});
  Driver cpu(machine);
  cpu.step();
  cpu.step();
  require(cpu.state().r[0] == second_address &&
              bus.read({.space = blofeld::DspSpace::x, .address = first_address}) == set_value &&
              bus.read({.space = blofeld::DspSpace::x, .address = second_address}) == 1,
          "Reference RAM EA bit writeback shares one effective-address update");
  cpu.step();
  require(bus.read({.space = blofeld::DspSpace::x, .address = first_address}) == 4 &&
              (cpu.state().sr & 1U) == 1,
          "Absolute-short BCLR preserves unselected RAM bits and returns the original high bit");
  cpu.step();
  cpu.step();
  require(bus.read({.space = blofeld::DspSpace::x, .address = second_address}) == 0 &&
              (cpu.state().sr & 1U) == 0,
          "Absolute-short BCHG and BTST retain normal RAM semantics");
  cpu.step();
  constexpr unsigned first_word = 0x101;
  constexpr unsigned second_word = 0x202;
  constexpr unsigned hrx = 0xffff94;
  bus.shi().observe_spi({.mosi = false});
  send_word(bus.shi(), first_word);
  send_word(bus.shi(), second_word);
  cpu.step();
  require((cpu.state().sr & 1U) == 1, "Fallback BTST samples the first destructive FIFO read");
  cpu.step();
  require((cpu.state().x & word_mask) == second_word,
          "A declined GPIO hook does not issue an extra SHI FIFO read");
  reject([&bus] { static_cast<void>(bus.shi().read(hrx)); },
         "Exactly two reads drain two FIFO words");
}

void test_native_gpio_output_bits_and_external_input() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.write({.space = DspSpace::x, .address = c_direction, .value = gpio_pin_mask});
  program.write({.space = DspSpace::x, .address = c_data, .value = initial_c_latch});
  program.bit({.operand = {.space = DspSpace::x, .address = c_data},
               .index = 4,
               .action = BitAction::clear});
  program.bit({.operand = {.space = DspSpace::x, .address = c_data},
               .index = 11,
               .action = BitAction::set});
  program.write({.space = DspSpace::y, .address = e_control, .value = gpio_pin_mask});
  program.receive({.space = DspSpace::y, .address = e_data}, DspRomRegister::x0);
  const auto before_reset = program.instructions();
  program.emit(reset_opcode);
  const auto after_reset = program.pc();
  program.write({.space = DspSpace::x, .address = c_direction, .value = gpio_pin_mask});
  program.receive({.space = DspSpace::x, .address = c_data}, DspRomRegister::x0);
  program.install(bus);
  Driver cpu(machine);
  constexpr unsigned inputs = 0xa53;
  constexpr unsigned all_pins = 0xfff;
  constexpr unsigned output = 0xe05;
  bus.gpio(blofeld::DspPort::e).set_drive({.mask = all_pins, .levels = inputs});
  for (unsigned index = 0; index < before_reset; ++index) { cpu.step(); }
  const auto driven = bus.gpio(blofeld::DspPort::c).outputs();
  require(driven.mask == all_pins && driven.known == all_pins && driven.levels == output,
          "Native MOVEP/BCLR/BSET update the GPIO latch and its physical output mask");
  require((cpu.state().x & word_mask) == inputs && (cpu.state().sr & 1U) == 0,
          "Native Y-space MOVEP samples observed E inputs; BSET carry reflects previous C bit");
  cpu.step();
  require(cpu.state().pc == after_reset && bus.gpio(blofeld::DspPort::c).outputs().mask == 0 &&
              bus.gpio(blofeld::DspPort::e).read(blofeld::DspGpioRegister::control) == 0,
          "Executed DSP RESET disconnects both native GPIO banks without changing PC flow");
  cpu.step();
  cpu.step();
  require((cpu.state().x & word_mask) == output,
          "Native re-enabling and reading C outputs after RESET sees the nonreset PDR latch");
}

void test_software_reset_preserves_clock_and_memory_mode() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.immediate(DspRomRegister::omr, retained_layout);
  program.immediate(DspRomRegister::r0, retained_r0);
  program.emit(reset_opcode);
  const auto after_reset = program.pc();
  program.emit(0);
  program.install(bus);
  Driver cpu(machine);
  cpu.step();
  cpu.step();
  bus.advance_instruction_cycles(3);
  constexpr unsigned alias = 0x1000;
  constexpr unsigned pll_command = 0x03e10b;
  constexpr unsigned hcsr = 0xffff91;
  constexpr unsigned enable_spi_fifo = 0xa9;
  bus.write({.address = alias, .value = input});
  bus.pll().write(pll_command);
  bus.shi().write({.address = hcsr, .value = enable_spi_fifo});
  bus.pll().observe_pinit(true);
  constexpr unsigned timer_control = 0xffff8f;
  constexpr unsigned timer_compare_flag = 0x200000;
  constexpr unsigned timer_input = 0x1000;
  bus.timers().set_tio_drive(0, true);
  bus.timers().write({.address = timer_control - 1, .value = 5});
  bus.timers().write({.address = timer_control - 2, .value = 5});
  bus.timers().write({.address = timer_control, .value = 1});
  bus.timers().advance_clk2(1);
  require(bus.timers().read(timer_control, timer_compare_flag) == timer_compare_flag,
          "An explicitly clocked timer compare is pending before native RESET");
  cpu.step();
  constexpr unsigned retained_mode = 0x600085;
  constexpr unsigned retained_pointer = 0x55;
  require(cpu.state().pc == after_reset && cpu.state().omr == retained_mode &&
              cpu.state().r[0] == retained_pointer,
          "DSP56300FM RESET resets peripheral/interrupt state while CPU execution continues");
  require(bus.pll().read() == pll_command && bus.read({.address = alias}) == input &&
              (bus.shi().read(hcsr) & 1U) == 0,
          "Software RESET clears SHI but preserves CPM PLL control and the OMR memory mapping");
  require(bus.timers().read(timer_control - 3) == 0 &&
              bus.timers().read(timer_control, timer_compare_flag | 1U) == 0 &&
              bus.timers().read(timer_control, timer_input) == timer_input,
          "Executed RESET clears timer count/control/flags and retains the external TIO drive");
  cpu.assert_reset(true);
  constexpr unsigned pinit_high_control = 0x04601d;
  require(bus.pll().read() == pinit_high_control && cpu.state().omr == reset_omr,
          "Hardware reset samples PINIT and restores the selected SPI boot profile");
  require(bus.read({.space = blofeld::DspSpace::x, .address = 0x10000}) == input,
          "Hardware reset restores the original map and retains the physical RAM contents");
  require(machine.clock().hardware_reset_active(),
          "PINIT-high reset state is captured while the actual CPU remains held");
  reject([&cpu] { cpu.step(); }, "Held PINIT-high reset cannot execute a native instruction");
  reject([&machine] { machine.release_hardware_reset(); },
         "PLL-enabled recovery cannot silently reuse bypass EXTAL timing");
}

void test_hardware_reset_restores_core_dma_priority() {
  constexpr unsigned priority_shift = 8;
  constexpr unsigned previous_mode = 0xa;
  for (unsigned priority = 0; priority < 4; ++priority) {
    const auto configured = (priority << priority_shift) | previous_mode;
    blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
    auto &bus = machine.bus();
    Program program;
    program.immediate(DspRomRegister::omr, configured);
    program.emit(reset_opcode);
    program.install(bus);
    Driver cpu(machine);
    cpu.step();
    require(cpu.state().omr == configured,
            "A native MOVEC can configure every documented core-DMA priority");
    cpu.step();
    require(cpu.state().omr == configured,
            "The executed RESET instruction preserves CPU priority and operating-mode bits");
    cpu.hardware_reset();
    require(cpu.state().omr == reset_omr && cpu.state().pc == boot,
            "External hardware reset restores literal CDP=11 and mode-five pins from every prior "
            "priority");
  }
}

void test_short_immediates_and_register_repeat_count() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.emit(short_x0_five);
  program.emit(short_r0_seven);
  program.emit(short_a1_nine);
  program.emit(repeat_x0);
  const auto body = program.pc();
  program.emit(store_x0_post);
  program.install(bus);
  Driver cpu(machine);
  for (unsigned index = 0; index < 4; ++index) { cpu.step(); }
  // DSP56300FM immediate-short parallel move: X0 is fractional; R0/A1 are integers.
  constexpr unsigned fractional_five = 0x050000;
  constexpr unsigned decremented_count = 0x04ffff;
  constexpr std::uint64_t integer_accumulator = 0x09000000;
  require((cpu.state().x & word_mask) == fractional_five && cpu.state().r[0] == 7 &&
              cpu.state().a == integer_accumulator,
          "Short immediate alignment depends on destination register, independently of spelling");
  require(
      cpu.state().pc == body && cpu.state().lc == fractional_five,
      "REP S uses the full 24-bit source count, not a truncated or integer-reinterpreted value");
  cpu.step();
  require(bus.read({.space = blofeld::DspSpace::x, .address = 7}) == fractional_five &&
              cpu.state().r[0] == 8 && cpu.state().lc == decremented_count,
          "A large register-specified REP remains bounded to one store per reference step");
}

void test_bounded_rep_steps() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.immediate(DspRomRegister::r0, 0x200);
  program.immediate(DspRomRegister::x0, 5);
  program.emit(repeat_three);
  const auto body = program.pc();
  program.emit(store_x0_post);
  const auto end = program.pc();
  program.install(bus);
  Driver cpu(machine);
  cpu.step();
  cpu.step();
  cpu.step();
  require(cpu.state().pc == body && cpu.state().lc == 3, "REP yields before executing its body");
  for (unsigned index = 0; index < 3; ++index) {
    cpu.step();
    require(bus.read({.space = blofeld::DspSpace::x, .address = 0x200 + index}) == 5 &&
                cpu.state().r[0] == next_repeat_address + index,
            "Each reference REP step performs exactly one original body store/postincrement");
    require(cpu.state().pc == (index == 2 ? end : body),
            "REP stays at its body until the final iteration");
  }
}

void test_bounded_do_waits_for_real_spi_words() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  bus.shi().observe_spi({});
  Program program;
  program.write({.space = DspSpace::x, .address = shi_rate, .value = 0});
  program.write({.space = DspSpace::x, .address = shi_control, .value = shi_polled_words});
  program.immediate(DspRomRegister::r0, 0x100);
  const auto loop = program.pc() + 2;
  Program body;
  body.branch({.operand = {.space = DspSpace::x, .address = shi_control},
               .index = shi_fifo_bit,
               .target = loop,
               .kind = BranchKind::jump_clear});
  body.receive({.space = DspSpace::x, .address = shi_receive}, DspRomRegister::a0);
  body.emit(store_a0_post);
  body.emit(0);
  const auto last = loop + static_cast<unsigned>(body.words().size()) - 1;
  program.emit(do_two, last);
  require(program.words().back() == last, "Literal DO encodes the preceding NOP as LA");
  program.append(body);
  program.install(bus);
  Driver cpu(machine);
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  require(cpu.state().pc == loop && cpu.state().lc == 2,
          "DO yields with its documented loop count");
  for (unsigned count = 0; count < 4; ++count) {
    cpu.step();
    require(cpu.state().pc == loop && cpu.state().lc == 2,
            "Native polling yields without inventing FIFO data");
  }
  for (const auto value : {input, second_input}) {
    send_word(bus.shi(), value);
    cpu.step();
    cpu.step();
    cpu.step();
    cpu.step();
  }
  require(bus.read({.space = blofeld::DspSpace::x, .address = data}) == input &&
              bus.read({.space = blofeld::DspSpace::x, .address = data + 1}) == second_input,
          "Two separately clocked SPI words reach RAM through original native DSP instructions");
  require(cpu.state().pc == program.pc() && cpu.state().sp == 0,
          "Final DO retirement falls through and restores both system-stack frames");
}

void test_rep_latches_single_word_without_stale_cache() {
  blofeld::OwnDspMachine machine(blofeld::test::dma_test::native_clocks);
  auto &bus = machine.bus();
  Program program;
  program.immediate(DspRomRegister::x0, 5);
  program.immediate(DspRomRegister::x1, 7);
  program.emit(repeat_three);
  const auto body = program.pc();
  program.emit(short_x0_store);
  const auto after = program.pc();
  program.jump(body);
  program.install(bus);
  Driver cpu(machine);
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  constexpr unsigned first_target = 0x10;
  constexpr unsigned changed_target = 0x11;
  require(bus.read({.space = blofeld::DspSpace::x, .address = first_target}) == 5,
          "First REP pass writes the independent original operand");
  Program modified;
  modified.emit(short_x1_store);
  bus.install_rom({.start = body, .words = modified.words()});
  for (unsigned index = 0; index < 2; ++index) {
    bus.write({.space = blofeld::DspSpace::x, .address = first_target, .value = 0});
    cpu.step();
    require(bus.read({.space = blofeld::DspSpace::x, .address = first_target}) == 5,
            "Later REP passes use the latched single-word opcode despite changed backing ROM");
    reject(
        [&bus] {
          static_cast<void>(bus.read({.space = blofeld::DspSpace::x, .address = changed_target}));
        },
        "The modified single-word destination is not consumed during REP");
  }
  require(cpu.state().pc == after, "REP ends after exactly three body executions");
  cpu.step();
  cpu.step();
  require(
      bus.read({.space = blofeld::DspSpace::x, .address = changed_target}) == 7,
      "A normal revisit decodes the changed single-word register/destination after REP retires");
}
}

int main() {
  try {
    test_memory_boundaries_rom_and_retained_switch();
    test_native_reset_fetch_fault_and_arithmetic();
    test_native_code_write_and_decoder_invalidation();
    test_native_timer_input_poll_and_reset();
    test_native_timer_w1c_word_writeback();
    test_native_timer_read_only_input_bit();
    test_native_symbolic_gpio_bits_and_effective_address();
    test_native_symbolic_port_f_pp_bits();
    test_native_symbolic_bit_branches();
    test_native_highest_bit_and_rejected_encoding();
    test_native_bit_ram_and_fifo_fallback();
    test_native_gpio_output_bits_and_external_input();
    test_software_reset_preserves_clock_and_memory_mode();
    test_hardware_reset_restores_core_dma_priority();
    test_short_immediates_and_register_repeat_count();
    test_bounded_rep_steps();
    test_bounded_do_waits_for_real_spi_words();
    test_rep_latches_single_word_without_stale_cache();
    pinit_checks();
    std::cout << "Native DSP reset, strict memory, arithmetic, code mutation and bounded "
                 "DO/REP/SPI checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
