// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/bus.hpp"
#include "blofeld/dsp_bus.hpp"
#include "blofeld/dsp_core.hpp"
#include "blofeld/dsp_gpio.hpp"
#include "blofeld/dsp_shi.hpp"
#include "dsp56kEmu/assembler.h"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr unsigned boot = 0xff0000;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned data = 0x100;
constexpr unsigned input = 0x123456;
constexpr unsigned second_input = 0xabcdef;
constexpr unsigned word_bits = 24;
constexpr unsigned last_switched = 0x2fff;
constexpr unsigned immediate_x0 = 0x44f400;
constexpr unsigned negative_address = 0x21;
constexpr unsigned next_repeat_address = 0x201;

void require(const bool value, const std::string &message) {
  if (!value) { throw blofeld::Fault(message); }
}

void reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &) { return; }
  throw blofeld::Fault(message);
}

std::string address(const unsigned value) {
  std::ostringstream text;
  text << '$' << std::hex << value;
  return text.str();
}

class Program {
public:
  void jump(const std::uint32_t target) {
    // DSP56300FM JMP ea: MMMRRR=110000 selects a 24-bit absolute extension.
    constexpr unsigned absolute_jump = 0x0af080;
    require(target <= word_mask, "Synthetic jump target is a 24-bit word");
    words_.push_back(absolute_jump);
    words_.push_back(target);
    ++instructions_;
  }

  void emit(const std::string &text) {
    const auto instruction = assembler_.assemble(text.c_str());
    require(instruction.success(), "Cannot assemble own DSP diagnostic: " + text);
    for (unsigned index = 0; index < instruction.wordCount; ++index) {
      words_.push_back(instruction.word[index]);
    }
    ++instructions_;
  }

  void append(const Program &program) {
    words_.insert(words_.end(), program.words_.begin(), program.words_.end());
    instructions_ += program.instructions_;
  }

  [[nodiscard]] std::span<const std::uint32_t> words() const { return words_; }

  [[nodiscard]] unsigned pc() const { return boot + static_cast<unsigned>(words_.size()); }

  [[nodiscard]] unsigned instructions() const { return instructions_; }

  void install(blofeld::DspBus &bus) const { bus.install_rom({.start = boot, .words = words_}); }

private:
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
  unsigned instructions_ = 0;
};

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
  blofeld::DspBus bus;
  const std::array<std::uint32_t, 1> nop{0};
  bus.install_rom({.start = boot, .words = nop});
  blofeld::DspCore cpu(bus);
  require(cpu.state().pc == boot && cpu.state().omr == 5 && cpu.state().sp == 0,
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
  cpu.reset_spi_boot();
  reject([&cpu] { cpu.step(); }, "A required two-word operand cannot use zero-filled lookahead");
  Program program;
  program.emit("move #>$123456,x0");
  require(program.words()[0] == immediate_x0 && program.words()[1] == input,
          "Immediate X0 encoding agrees with the independent instruction reference");
  program.emit("move #>$10,x1");
  program.emit("clr a");
  program.emit("add x0,a");
  program.emit("add x1,a");
  program.emit("move a,x:>$20");
  program.emit("neg a");
  program.emit("move a,x:>$21");
  program.install(bus);
  cpu.reset_spi_boot();
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
  blofeld::DspBus bus;
  Program start;
  start.emit("move #>0,x0");
  start.emit("move #>$100,r0");
  start.emit("move x0,p:(r0)");
  start.jump(data);
  start.install(bus);
  blofeld::DspCore cpu(bus);
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  require(cpu.state().pc == data + 1, "Writing unknown P RAM does not require a synthetic preread");
  Program loop;
  loop.emit("move #>1,x0");
  loop.jump(data);
  unsigned offset = data;
  for (const auto value : loop.words()) { bus.write({.address = offset++, .value = value}); }
  cpu.reset_spi_boot();
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
  replacement.emit("move #>3,x1");
  bus.write({.address = data, .value = replacement.words()[0]});
  bus.write({.address = data + 1, .value = replacement.words()[1]});
  cpu.step();
  constexpr std::uint64_t changed = 0x3000002;
  require(cpu.state().x == changed, "Changing a cached P opcode changes its destination register");
}

void test_native_gpio_output_bits_and_external_input() {
  blofeld::DspBus bus;
  Program program;
  program.emit("movep #>$fff,x:<<$ffffbe");
  program.emit("movep #>$615,x:<<$ffffbd");
  program.emit("bclr #4,x:<<$ffffbd");
  program.emit("bset #11,x:<<$ffffbd");
  program.emit("movep #>$fff,y:<<$ffff9f");
  program.emit("movep y:<<$ffff9d,x0");
  const auto before_reset = program.instructions();
  program.emit("reset");
  const auto after_reset = program.pc();
  program.emit("movep #>$fff,x:<<$ffffbe");
  program.emit("movep x:<<$ffffbd,x0");
  program.install(bus);
  blofeld::DspCore cpu(bus);
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
  blofeld::DspBus bus;
  Program program;
  program.emit("move #>$600085,omr");
  program.emit("move #>$55,r0");
  program.emit("reset");
  const auto after_reset = program.pc();
  program.emit("nop");
  program.install(bus);
  blofeld::DspCore cpu(bus);
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
  cpu.step();
  constexpr unsigned retained_mode = 0x600085;
  constexpr unsigned retained_pointer = 0x55;
  require(cpu.state().pc == after_reset && cpu.state().omr == retained_mode &&
              cpu.state().r[0] == retained_pointer,
          "DSP56300FM RESET resets peripheral/interrupt state while CPU execution continues");
  require(bus.pll().read() == pll_command && bus.read({.address = alias}) == input &&
              (bus.shi().read(hcsr) & 1U) == 0,
          "Software RESET clears SHI but preserves CPM PLL control and the OMR memory mapping");
  cpu.reset_spi_boot();
  constexpr unsigned pinit_high_control = 0x04601d;
  require(bus.pll().read() == pinit_high_control && cpu.state().omr == 5,
          "Hardware reset samples PINIT and restores the selected SPI boot profile");
  require(bus.read({.space = blofeld::DspSpace::x, .address = 0x10000}) == input,
          "Hardware reset restores the original map and retains the physical RAM contents");
}

void test_short_immediates_and_register_repeat_count() {
  blofeld::DspBus bus;
  Program program;
  program.emit("move #<5,x0");
  program.emit("move #<7,r0");
  program.emit("move #<9,a1");
  program.emit("rep x0");
  const auto body = program.pc();
  program.emit("move x0,x:(r0)+");
  program.install(bus);
  blofeld::DspCore cpu(bus);
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
  blofeld::DspBus bus;
  Program program;
  program.emit("move #>$200,r0");
  program.emit("move #>5,x0");
  program.emit("rep #3");
  const auto body = program.pc();
  program.emit("move x0,x:(r0)+");
  const auto end = program.pc();
  program.install(bus);
  blofeld::DspCore cpu(bus);
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
  blofeld::DspBus bus;
  bus.shi().observe_spi({});
  Program program;
  program.emit("movep #>0,x:<<$ffff90");
  program.emit("movep #>$29,x:<<$ffff91");
  program.emit("move #>$100,r0");
  const auto loop = program.pc() + 2;
  Program body;
  body.emit("jclr #17,x:<<$ffff91," + address(loop));
  body.emit("movep x:<<$ffff94,a0");
  body.emit("move a0,x:(r0)+");
  body.emit("nop");
  const auto last = loop + static_cast<unsigned>(body.words().size()) - 1;
  program.emit("do #2," + address(last + 1));
  require(program.words().back() == last, "Assembler end label encodes the preceding NOP as LA");
  program.append(body);
  program.install(bus);
  blofeld::DspCore cpu(bus);
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

void test_rep_latches_opcode_and_extension_without_stale_cache() {
  blofeld::DspBus bus;
  Program program;
  program.emit("move #>5,x0");
  program.emit("move #>7,x1");
  program.emit("rep #3");
  const auto body = program.pc();
  program.emit("move x0,x:>$210");
  const auto after = program.pc();
  program.jump(body);
  program.install(bus);
  blofeld::DspCore cpu(bus);
  cpu.step();
  cpu.step();
  cpu.step();
  cpu.step();
  constexpr unsigned first_target = 0x210;
  constexpr unsigned changed_target = 0x211;
  require(bus.read({.space = blofeld::DspSpace::x, .address = first_target}) == 5,
          "First REP pass writes the independent original operand");
  Program modified;
  modified.emit("move x1,x:>$211");
  bus.install_rom({.start = body, .words = modified.words()});
  for (unsigned index = 0; index < 2; ++index) {
    bus.write({.space = blofeld::DspSpace::x, .address = first_target, .value = 0});
    cpu.step();
    require(bus.read({.space = blofeld::DspSpace::x, .address = first_target}) == 5,
            "Later REP passes use the latched opcode and extension despite changed backing ROM");
    reject(
        [&bus] {
          static_cast<void>(bus.read({.space = blofeld::DspSpace::x, .address = changed_target}));
        },
        "The modified extension is not consumed during REP");
  }
  require(cpu.state().pc == after, "REP ends after exactly three body executions");
  cpu.step();
  cpu.step();
  require(bus.read({.space = blofeld::DspSpace::x, .address = changed_target}) == 7,
          "A normal revisit decodes the changed opcode and extension after REP retires");
}
}

int main() {
  try {
    test_memory_boundaries_rom_and_retained_switch();
    test_native_reset_fetch_fault_and_arithmetic();
    test_native_code_write_and_decoder_invalidation();
    test_native_gpio_output_bits_and_external_input();
    test_software_reset_preserves_clock_and_memory_mode();
    test_short_immediates_and_register_repeat_count();
    test_bounded_rep_steps();
    test_bounded_do_waits_for_real_spi_words();
    test_rep_latches_opcode_and_extension_without_stale_cache();
    std::cout << "Native DSP reset, strict memory, arithmetic, code mutation and bounded "
                 "DO/REP/SPI checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
