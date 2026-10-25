// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp56kEmu/assembler.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr unsigned boot = 0xff0000;
constexpr unsigned iprp = 0xfffffe;
constexpr unsigned iprc = 0xffffff;
constexpr unsigned csr0 = 0xffff8f;
constexpr unsigned csr2 = 0xffff87;
constexpr unsigned timer_vector = 0x54;
constexpr unsigned overflow_vector = 0x56;
constexpr unsigned shi_vector = 0x44;
constexpr unsigned handler = 0x300;
constexpr unsigned nested_handler = 0x320;
constexpr unsigned mask_bits = 0x300;
constexpr unsigned compare_flag = 0x200000;
constexpr unsigned hcsr = 0xffff91;
constexpr unsigned hrx = 0xffff94;
constexpr unsigned payload = 0x123456;
constexpr unsigned word_bits = 24;
constexpr unsigned ordinary_sr = 0xc00000;
constexpr unsigned scaling_sr = 0xc20455;
constexpr unsigned long_cleared_bits = 0x38f00;
constexpr unsigned rti_opcode = 4;
constexpr unsigned foreground_value = 0x77;
constexpr unsigned short_jsr = 0x0d0000;
constexpr unsigned absolute_jsr = 0x0bf080;
constexpr unsigned base = 0x500;
constexpr unsigned nested_priorities = 0x10c;
constexpr unsigned timer_two_vector = 0x5c;
enum class VectorForm : std::uint8_t { first, second, absolute };

struct Priorities {
  unsigned shi;
  unsigned timer;
};

void require(const bool condition, const std::string &message) {
  if (!condition) { throw blofeld::Fault(message); }
}

std::string reject(const auto &operation, const std::string &message) {
  try {
    operation();
  } catch (const blofeld::Fault &error) { return error.what(); }
  throw blofeld::Fault(message);
}

std::string number(const unsigned value) {
  std::ostringstream text;
  text << '$' << std::hex << value;
  return text.str();
}

void load_program(blofeld::DspBus &bus,
                  const unsigned start,
                  const std::span<const std::uint32_t> words) {
  if (start >= boot) {
    bus.install_rom({.start = start, .words = words});
  } else {
    for (std::size_t index = 0; index < words.size(); ++index) {
      bus.write({.address = start + static_cast<unsigned>(index), .value = words[index]});
    }
  }
}

class Program {
public:
  void emit(const std::string &text) {
    const auto instruction = assembler_.assemble(text.c_str());
    require(instruction.success(), "Cannot assemble independent interrupt diagnostic: " + text);
    for (unsigned index = 0; index < instruction.wordCount; ++index) {
      words_.push_back(instruction.word[index]);
    }
    ++instructions_;
  }

  void timer(const unsigned csr, const unsigned load = 5, const unsigned compare = 5) {
    emit("movep #>" + number(load) + ",x:<<" + number(csr - 1));
    emit("movep #>" + number(compare) + ",x:<<" + number(csr - 2));
    emit("movep #>7,x:<<" + number(csr));
  }

  void setup(const unsigned priorities, const unsigned status = ordinary_sr) {
    emit("move #>" + number(status) + ",sr");
    emit("movep #>" + number(priorities) + ",x:<<$fffffe");
  }

  void install(blofeld::DspBus &bus, const unsigned start = boot) const {
    load_program(bus, start, words_);
  }

  [[nodiscard]] unsigned instructions() const { return instructions_; }

  [[nodiscard]] unsigned pc() const { return boot + static_cast<unsigned>(words_.size()); }

  [[nodiscard]] std::span<const std::uint32_t> words() const { return words_; }

private:
  dsp56k::Assembler assembler_;
  std::vector<std::uint32_t> words_;
  unsigned instructions_ = 0;
};

void run(blofeld::DspCore &cpu, const unsigned instructions) {
  for (unsigned index = 0; index < instructions; ++index) { cpu.step(); }
}

unsigned pending_vector(const blofeld::DspBus &bus) {
  const auto request = bus.interrupt_request(0);
  if (!request) { throw blofeld::Fault("Expected an independently generated pending interrupt"); }
  return request->vector;
}

void send(blofeld::DspShi &shi, const unsigned value = payload) {
  shi.observe_spi({.mosi = false});
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool level = ((value >> (word_bits - 1 - bit)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = level});
    shi.observe_spi({.selected = true, .clock = false, .mosi = level});
  }
  shi.observe_spi({.mosi = false});
}

void check_priority_mask(const blofeld::DspInterrupts &controller,
                         const Priorities priorities,
                         const unsigned mask) {
  constexpr std::array<std::uint32_t, 2> pending{overflow_vector, 0x4c};
  const auto selected = controller.select(pending, mask);
  const bool shi_allowed = priorities.shi != 0 && priorities.shi > mask;
  const bool timer_allowed = priorities.timer != 0 && priorities.timer > mask;
  require(selected.has_value() == (shi_allowed || timer_allowed),
          "Every encoded source priority and CPU mask has the documented eligibility");
  if (!selected) { return; }
  const bool choose_shi = shi_allowed && (!timer_allowed || priorities.shi >= priorities.timer);
  constexpr unsigned shi_bus_error = 0x4c;
  require(selected->vector == (choose_shi ? shi_bus_error : overflow_vector) &&
              selected->level == (choose_shi ? priorities.shi : priorities.timer) - 1,
          "Highest eligible level wins, then fixed hardware priority, independent of arrival");
}

void test_priority_matrix_and_ties() {
  // NXP DSP56371UM chapter 4: SHI before TIM; overflow before compare per timer.
  constexpr std::array<std::uint32_t, 12> ranked{
      0x4c, 0x4a, 0x42, 0x48, 0x40, 0x44, 0x56, 0x54, 0x5a, 0x58, 0x5e, 0x5c};
  blofeld::DspInterrupts controller;
  require(controller.read(iprp) == 0 && controller.read(iprc) == 0,
          "Hardware reset disables all maskable priority fields");
  for (unsigned shi = 0; shi < 4; ++shi) {
    for (unsigned timer = 0; timer < 4; ++timer) {
      controller.write(iprp, (shi << 2U) | (timer << 8U));
      for (unsigned mask = 0; mask < 4; ++mask) {
        check_priority_mask(controller, {.shi = shi, .timer = timer}, mask);
      }
    }
  }
  controller.write(iprp, mask_bits | 12U);
  for (std::size_t index = 0; index < ranked.size(); ++index) {
    const auto selected = controller.select(std::span(ranked).subspan(index), 0);
    require(selected && selected->vector == ranked.at(index) && selected->level == 2,
            "All twelve internal sources retain their independent documented tie order");
  }
}

void test_priority_rejections_and_reset() {
  blofeld::DspInterrupts controller;
  constexpr unsigned known_priorities = 0x204;
  controller.write(iprp, known_priorities);
  constexpr std::array<unsigned, 5> unsupported{16, 0x20, 0x40, 0x10000, 0x1000000};
  for (const auto value : unsupported) {
    static_cast<void>(
        reject([&controller, value] { controller.write(iprp, value); },
               "Unimplemented/reserved priority fields cannot become ignored writes"));
    require(controller.read(iprp) == known_priorities,
            "Rejected priorities preserve register state");
  }
  static_cast<void>(reject([&controller] { controller.write(iprc, 8); },
                           "Enabled unimplemented IRQB/core sources remain unsupported"));
  static_cast<void>(
      reject([&controller] { controller.write(0, 0); }, "Unknown write address fails"));
  static_cast<void>(reject([&controller] { static_cast<void>(controller.read(0)); },
                           "Unknown read address fails"));
  constexpr std::array<std::uint32_t, 1> invalid{0x52};
  static_cast<void>(
      reject([&controller, &invalid] { static_cast<void>(controller.select(invalid, 0)); },
             "Unsupported pending vectors cannot disappear"));
  static_cast<void>(reject([&controller] { static_cast<void>(controller.select({}, 4)); },
                           "Out-of-range CPU masks fail"));
  controller.write(iprc, 0);
  controller.reset();
  require(controller.read(iprp) == 0 && controller.read(iprc) == 0,
          "Peripheral RESET restores disabled priorities");
}

void test_fast_entry(const bool double_word) {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100);
  program.timer(csr0);
  const auto setup_count = program.instructions();
  const auto foreground = program.pc();
  program.emit("move #>$77,r7");
  program.install(bus);
  Program vector;
  if (double_word) {
    vector.emit("move #>$123456,x0");
  } else {
    vector.emit("move #<5,r1");
    vector.emit("move #<7,r2");
  }
  require(vector.words().size() == 2, "Fast fixture contains exactly two vector words");
  vector.install(bus, timer_vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  const auto before = cpu.state();
  bus.timers().advance_clk2(1);
  require(bus.interrupt_request(0).has_value(), "A real timer compare requests native service");
  cpu.step();
  const auto after = cpu.state();
  require(after.pc == foreground && after.sp == before.sp && after.sr == before.sr &&
              after.r[7] == 0,
          "Fast entry freezes PC, preserves stack and does not execute a foreground instruction");
  require(double_word ? (after.x & iprc) == payload : (after.r[1] == 5 && after.r[2] == 7),
          "Fast entry executes one double-word or two single-word native instructions");
  require(bus.timers().read(csr0, compare_flag) == 0 && !bus.interrupt_request(0),
          "Timer flag clears on the real second vector-word fetch");
  cpu.step();
  require(cpu.state().r[7] == foreground_value, "Next step resumes the unchanged foreground PC");
}

void test_fast_latches_both_vector_words() {
  blofeld::DspBus bus;
  Program changed;
  changed.emit("move #<9,r2");
  require(changed.words().size() == 1, "Replacement diagnostic opcode is one word");
  Program program;
  program.setup(0x100);
  program.timer(csr0);
  program.emit("move #>" + number(changed.words().front()) + ",x0");
  program.emit("move #>" + number(timer_vector + 1) + ",r0");
  const auto setup_count = program.instructions();
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(bus);
  Program vector;
  vector.emit("move x0,p:(r0)");
  vector.emit("move #<7,r2");
  require(vector.words().size() == 2,
          "Self-modifying vector contains two single-word instructions");
  vector.install(bus, timer_vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  bus.timers().advance_clk2(1);
  cpu.step();
  require(bus.read({.address = timer_vector + 1}) == changed.words().front(),
          "First native vector word actually writes the replacement opcode into P RAM");
  require(
      cpu.state().pc == foreground && cpu.state().r[2] == 7,
      "Second prefetched vector word executes its original seven despite the intervening P store");
}

void test_native_mask_release() {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100, ordinary_sr | mask_bits);
  program.timer(csr0);
  const auto setup_count = program.instructions();
  program.emit("andi #$fc,mr");
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> vector{0, 0};
  load_program(bus, timer_vector, vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  bus.timers().advance_clk2(1);
  cpu.step();
  require(cpu.state().pc == foreground && (cpu.state().sr & mask_bits) == 0 &&
              bus.timers().read(csr0, compare_flag) == compare_flag,
          "Masked IRQ remains pending while the foreground native ANDI releases IPL");
  cpu.step();
  require(cpu.state().pc == foreground && !bus.interrupt_request(0),
          "The reference boundary after native mask release services the retained request");
}

void test_masked_source_does_not_probe_unknown_pins() {
  blofeld::DspBus bus;
  Program program;
  constexpr unsigned priorities = 0x304;
  constexpr unsigned cpu_mask = 0x200;
  program.setup(priorities, ordinary_sr | cpu_mask);
  program.emit("movep #>$1009,x:<<$ffff91");
  program.timer(csr0);
  const auto setup_count = program.instructions();
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> vector{0, 0};
  load_program(bus, timer_vector, vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  bus.timers().advance_clk2(1);
  cpu.step();
  require(cpu.state().pc == foreground && bus.timers().read(csr0, compare_flag) == 0,
          "Eligible higher timer priority fetch/ack does not probe a masked lower SHI source");
  static_cast<void>(reject([&bus] { static_cast<void>(bus.shi().read(hcsr)); },
                           "Unused unknown SHI inputs remain unknown after timer service"));
}

void test_bus_tie_order_and_shi_acknowledgement() {
  blofeld::DspBus bus;
  Program program;
  constexpr unsigned all_ones = 0xffffff;
  constexpr unsigned same_level_priorities = 0x104;
  program.setup(same_level_priorities);
  program.timer(csr0, all_ones, 0);
  program.emit("movep #>$1009,x:<<$ffff91");
  const auto setup_count = program.instructions();
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> shi{short_jsr | handler, 0};
  constexpr std::array<std::uint32_t, 2> fast{0, 0};
  load_program(bus, shi_vector, shi);
  load_program(bus, timer_vector, fast);
  load_program(bus, overflow_vector, fast);
  Program receive;
  receive.emit("movep x:<<$ffff94,x0");
  receive.emit("rti");
  receive.install(bus, handler);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  bus.timers().advance_clk2(2);
  send(bus.shi());
  cpu.step();
  require(cpu.state().pc == handler && pending_vector(bus) == shi_vector,
          "SHI wins a cross-source tie; vector acknowledgement alone does not consume its FIFO");
  cpu.step();
  require((cpu.state().x & iprc) == payload && pending_vector(bus) == overflow_vector,
          "Native HRX consumption exposes the already pending timer overflow before compare");
  cpu.step();
  cpu.step();
  constexpr unsigned overflow = 0x100000;
  require(cpu.state().pc == foreground && bus.timers().read(csr0, overflow) == 0 &&
              bus.timers().read(csr0, compare_flag) == compare_flag &&
              pending_vector(bus) == timer_vector,
          "Native overflow service clears only its own flag, retaining the same timer's compare");
  cpu.step();
  require(cpu.state().pc == foreground && !bus.interrupt_request(0),
          "The subsequent native compare fetch acknowledges the remaining event");
}

void test_long_entry(const VectorForm form, const unsigned level) {
  blofeld::DspBus bus;
  Program program;
  program.setup((level + 1) << 8U, scaling_sr);
  program.timer(csr0);
  const auto setup_count = program.instructions();
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(bus);
  // DSP56300FM JSR xxx/ea and RTI opcodes, independent of the SDK assembler.
  std::array<std::uint32_t, 2> vector{};
  switch (form) {
    case VectorForm::first: vector = {short_jsr | handler, rti_opcode}; break;
    case VectorForm::second: vector = {0, short_jsr | handler}; break;
    case VectorForm::absolute: vector = {absolute_jsr, handler}; break;
    default: throw blofeld::Fault("Invalid synthetic JSR vector form");
  }
  load_program(bus, timer_vector, vector);
  constexpr std::array<std::uint32_t, 1> return_code{rti_opcode};
  load_program(bus, handler, return_code);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  const auto before = cpu.state();
  require(before.sr == scaling_sr, "Native startup retains the independently specified SR bits");
  bus.timers().advance_clk2(1);
  cpu.step();
  const auto entered = cpu.state();
  require(entered.pc == handler && entered.sp == 1 && entered.stack_pc == foreground &&
              entered.stack_sr == before.sr,
          "JSR in either vector slot or an absolute pair stacks the native return PC/SR once");
  const auto expected_sr = (before.sr & ~long_cleared_bits) | ((level + 1) << 8U);
  require(entered.sr == expected_sr,
          "Long entry clears scaling/SA/loop flags and raises actual IPL");
  cpu.step();
  require(cpu.state().pc == foreground && cpu.state().sp == before.sp &&
              cpu.state().sr == before.sr,
          "Native RTI independently restores the exact saved PC/SR and SP");
}

void test_fast_status_and_shi_fifo() {
  blofeld::DspBus bus;
  Program program;
  program.setup(4);
  program.emit("movep #>$1009,x:<<$ffff91");
  const auto setup_count = program.instructions();
  program.emit("nop");
  program.install(bus);
  Program vector;
  vector.emit("movep x:<<$ffff94,x0");
  vector.emit("clr a");
  vector.install(bus, shi_vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  const auto before = cpu.state();
  send(bus.shi());
  cpu.step();
  constexpr unsigned zero_flag = 4;
  require(cpu.state().pc == before.pc && cpu.state().sp == 0 && (cpu.state().x & iprc) == payload &&
              (cpu.state().sr & zero_flag) != 0,
          "Fast SHI vector consumes the real FIFO word; ALU status is not implicitly preserved");
  require(!bus.interrupt_request(0), "Native HRX read removes the level-sensitive receive request");
  static_cast<void>(reject([&bus] { static_cast<void>(bus.shi().read(hrx)); },
                           "Vector delivery cannot synthesize a second FIFO word"));
}

void test_nested_priority_and_pending_retention() {
  blofeld::DspBus bus;
  Program program;
  program.setup(nested_priorities);
  program.timer(csr0);
  program.timer(csr2);
  program.emit("movep #>$1009,x:<<$ffff91");
  const auto setup_count = program.instructions();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> timer{short_jsr | handler, 0};
  constexpr std::array<std::uint32_t, 2> shi{0, short_jsr | nested_handler};
  load_program(bus, timer_vector, timer);
  load_program(bus, shi_vector, shi);
  constexpr std::array<std::uint32_t, 2> outer{0, rti_opcode};
  load_program(bus, handler, outer);
  Program nested;
  nested.emit("movep x:<<$ffff94,x0");
  nested.emit("rti");
  nested.install(bus, nested_handler);
  blofeld::DspCore cpu(bus);
  bus.shi().observe_spi({.mosi = false});
  run(cpu, setup_count);
  const auto foreground = cpu.state();
  bus.timers().advance_clk2(1);
  cpu.step();
  const auto interrupted = cpu.state();
  require(interrupted.pc == handler && (interrupted.sr & mask_bits) == 0x100,
          "Timer zero compare wins over timer two compare at the same level");
  require(pending_vector(bus) == timer_two_vector && !bus.interrupt_request(1),
          "Lower/equal pending timer remains latched but masked inside its handler");
  send(bus.shi());
  cpu.step();
  require(cpu.state().pc == nested_handler && cpu.state().sp == 2 &&
              cpu.state().stack_pc == interrupted.pc && cpu.state().stack_sr == interrupted.sr &&
              (cpu.state().sr & mask_bits) == mask_bits,
          "Higher-level SHI preempts a native long timer interrupt with a second exact frame");
  cpu.step();
  cpu.step();
  require(cpu.state().pc == handler && cpu.state().sp == 1 && cpu.state().sr == interrupted.sr,
          "Nested native RTI restores the interrupted handler and its lower IPL");
  cpu.step();
  cpu.step();
  require(cpu.state().pc == foreground.pc && cpu.state().sp == 0 &&
              cpu.state().sr == foreground.sr && pending_vector(bus) == timer_two_vector,
          "Outer native RTI restores foreground while the unserviced timer remains pending");
}

void test_vba_relocation_and_resets() {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100);
  program.emit("move #>$5ff,vba");
  program.timer(csr0);
  const auto setup_count = program.instructions();
  program.emit("reset");
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> vector{0, 0};
  load_program(bus, base + timer_vector, vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  require(cpu.state().vba == base, "Native MOVEC cannot write the eight read-only low VBA bits");
  bus.timers().advance_clk2(1);
  const auto before = cpu.state();
  cpu.step();
  require(cpu.state().pc == before.pc && !bus.interrupt_request(0),
          "CPU fetches the real vector at VBA plus the peripheral-relative offset");
  cpu.step();
  require(cpu.state().vba == base && cpu.state().sr == before.sr &&
              bus.read({.space = blofeld::DspSpace::x, .address = iprp}) == 0,
          "Native RESET clears priority registers while retaining CPU VBA and SR");
  cpu.reset_spi_boot();
  require(cpu.state().vba == 0 && (cpu.state().sr & mask_bits) == mask_bits,
          "Hardware reset restores zero VBA and blocks all maskable levels");
}

void test_sequential_rep_deferral() {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100);
  program.timer(csr0);
  const auto setup_count = program.instructions();
  program.emit("rep #2");
  program.emit("move x0,x1");
  program.emit("rep #3");
  program.emit("move y0,y1");
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> vector{0, 0};
  load_program(bus, timer_vector, vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count + 1);
  bus.timers().advance_clk2(1);
  // Two first bodies, next REP, three next bodies: all six steps defer service.
  for (unsigned index = 0; index < 6; ++index) {
    cpu.step();
    require(bus.interrupt_request(0).has_value(),
            "Pending interrupt remains unserviced throughout both consecutive REP sequences");
  }
  require(cpu.state().pc == foreground, "Both native REP counts retire before IRQ entry");
  cpu.step();
  require(cpu.state().pc == foreground && !bus.interrupt_request(0),
          "Pending interrupt is delivered at the first interruptible instruction boundary");
}

void test_do_loop_interrupt_restore() {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100);
  program.timer(csr0);
  const auto setup_count = program.instructions();
  // DO is two words, then one native NOP; assembler end label is one beyond LA.
  const auto loop = program.pc() + 2;
  program.emit("do #2," + number(loop + 1));
  program.emit("nop");
  const auto foreground = program.pc();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> vector{short_jsr | handler, 0};
  constexpr std::array<std::uint32_t, 1> restore{rti_opcode};
  load_program(bus, timer_vector, vector);
  load_program(bus, handler, restore);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count + 1);
  const auto before = cpu.state();
  constexpr unsigned lf = 0x8000;
  require(before.pc == loop && before.sp == 2 && before.lc == 2 && (before.sr & lf) != 0,
          "Native DO establishes its two independent loop stack entries");
  bus.timers().advance_clk2(1);
  cpu.step();
  require(cpu.state().pc == handler && cpu.state().sp == 3 && (cpu.state().sr & lf) == 0 &&
              cpu.state().stack_pc == loop && cpu.state().stack_sr == before.sr,
          "Long IRQ inside DO saves the loop flag and disables loop redirection inside the ISR");
  cpu.step();
  require(cpu.state().pc == loop && cpu.state().sp == 2 && cpu.state().sr == before.sr &&
              cpu.state().lc == before.lc && cpu.state().la == before.la,
          "Native RTI preserves the suspended DO count, endpoint, frames and loop flag");
  run(cpu, 2);
  require(cpu.state().pc == foreground && cpu.state().sp == 0 && (cpu.state().sr & lf) == 0,
          "Original DO executes exactly two remaining bodies and restores its own stack frames");
}

void test_unknown_vector_and_extension(const unsigned known_words) {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100);
  program.timer(csr0);
  const auto setup_count = program.instructions();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 1> first{absolute_jsr};
  if (known_words != 0) { load_program(bus, timer_vector, first); }
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  const auto before = cpu.state();
  bus.timers().advance_clk2(1);
  const auto error = reject([&cpu] { cpu.step(); }, "Unknown interrupt contents must fail");
  const auto *const missing = known_words == 0 ? "P:0x000054" : "P:0x000055";
  require(error.find(missing) != std::string::npos &&
              error.find("irq=0x54 level=0") != std::string::npos &&
              error.find("pc=" + blofeld::hex(before.pc, 6)) != std::string::npos,
          "Unknown vector fetch reports exact P address, source, level and foreground PC");
  require(bus.timers().read(csr0, compare_flag) == compare_flag && cpu.state().sp == 0,
          "Rejected vector fetch cannot acknowledge the timer or push an invented frame");
  static_cast<void>(reject([&cpu] { cpu.step(); }, "Faulted native interrupt requires reset"));
  cpu.reset_spi_boot();
  run(cpu, setup_count);
  require(
      cpu.state().pc == before.pc && cpu.state().sp == 0 && !bus.interrupt_request(0),
      "Hardware reset clears rejected interrupt mode and executes the normal native boot again");
}

void test_interrupt_stack_overflow_rejection() {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100);
  program.timer(csr0);
  program.emit("move #>15,sp");
  const auto setup_count = program.instructions();
  program.emit("nop");
  program.install(bus);
  constexpr std::array<std::uint32_t, 2> vector{short_jsr | handler, 0};
  load_program(bus, timer_vector, vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  const auto before = cpu.state();
  bus.timers().advance_clk2(1);
  const auto error = reject([&cpu] { cpu.step(); }, "Interrupt stack overflow must fail visibly");
  require(error.find("Unsupported reference interrupt stack overflow") != std::string::npos &&
              cpu.state().pc == before.pc && cpu.state().sp == before.sp,
          "Full native stack rejects JSR before writing/wrapping a frame or transferring control");
}

void test_rejected_fast_control_flow() {
  blofeld::DspBus bus;
  Program program;
  program.setup(0x100);
  program.timer(csr0);
  const auto setup_count = program.instructions();
  program.emit("nop");
  program.install(bus);
  // JMP xxx is prohibited for this maskable fast-reference subset.
  constexpr std::array<std::uint32_t, 2> vector{0x0c0300, 0};
  load_program(bus, timer_vector, vector);
  blofeld::DspCore cpu(bus);
  run(cpu, setup_count);
  const auto before = cpu.state();
  bus.timers().advance_clk2(1);
  const auto error = reject([&cpu] { cpu.step(); }, "Unsupported fast-vector jump fails");
  require(error.find("Unsupported reference interrupt vector instruction") != std::string::npos &&
              cpu.state().pc == before.pc && cpu.state().sp == 0,
          "Unsupported vector flow fails before executing the branch");
}
}

int main() {
  try {
    test_priority_matrix_and_ties();
    test_priority_rejections_and_reset();
    test_fast_entry(false);
    test_fast_entry(true);
    test_fast_latches_both_vector_words();
    test_native_mask_release();
    test_masked_source_does_not_probe_unknown_pins();
    test_bus_tie_order_and_shi_acknowledgement();
    for (const auto form : {VectorForm::first, VectorForm::second, VectorForm::absolute}) {
      for (unsigned level = 0; level < 3; ++level) { test_long_entry(form, level); }
    }
    test_fast_status_and_shi_fifo();
    test_nested_priority_and_pending_retention();
    test_vba_relocation_and_resets();
    test_sequential_rep_deferral();
    test_do_loop_interrupt_restore();
    test_unknown_vector_and_extension(0);
    test_unknown_vector_and_extension(1);
    test_rejected_fast_control_flow();
    test_interrupt_stack_overflow_rejection();
    std::cout << "DSP priority, native fast/long entry, nested IRQ, RTI, VBA, DO/REP and rejection "
                 "checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
