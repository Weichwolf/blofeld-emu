// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_interrupts.hpp"
#include "dsp/dsp_shi.hpp"
#include "dsp/dsp_timers.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "dsp_interrupt_fixture.hpp"

#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

namespace blofeld::test::irq {
namespace {
constexpr unsigned timer_vector = 0x54, shi_vector = 0x44, handler = 0x300;
constexpr unsigned csr = 0xffff8f, shi_control = 0xffff91, compare_flag = 0x200000;
constexpr unsigned payload = 0x123456, foreground_opcode = 0x377700, foreground_value = 0x77;
constexpr unsigned short_call = 0x0d0300, absolute_call = 0x0bf080, return_code = 4;
constexpr unsigned short_r3_nine = 0x330900, word_bits = 24, suffix_bound = 128;
constexpr unsigned changed_ccr = 0x3e, zero_ccr = 0x14;
enum class Kind : std::uint8_t { fast, two_word, first_call, second_call, absolute, shi, store };
constexpr std::array kinds{Kind::fast,
                           Kind::two_word,
                           Kind::first_call,
                           Kind::second_call,
                           Kind::absolute,
                           Kind::shi,
                           Kind::store};

void check(const bool condition, const char *message) {
  if (!condition) { throw Fault(message); }
}

void send(DspShi &shi) {
  shi.observe_spi({.mosi = false});
  shi.observe_spi({.selected = true, .mosi = false});
  for (unsigned bit = 0; bit < word_bits; ++bit) {
    const bool value = ((payload >> (word_bits - bit - 1)) & 1U) != 0;
    shi.observe_spi({.selected = true, .clock = true, .mosi = value});
    shi.observe_spi({.selected = true, .clock = false, .mosi = value});
  }
  shi.observe_spi({.mosi = false});
}

struct Observation {
  dsp56300::Registers registers{};
  dsp56300::Progress progress{};
  std::optional<dsp56300::InstructionEffect> pending;
  std::optional<DspInterruptRequest> request;
  std::optional<unsigned> opcode;
  std::optional<bool> vector;
  unsigned address = 0, second_word = 0, timer_flag = 0;
  std::uint64_t ticks = 0;
  bool acknowledging = false;
  bool operator==(const Observation &) const = default;
};

class Scenario {
public:
  explicit Scenario(const Kind kind)
      : kind_(kind), vector_(kind == Kind::shi ? shi_vector : timer_vector) {
    Program code;
    code.setup({.priorities = kind == Kind::shi ? 4U : 0x100U});
    if (kind == Kind::shi) {
      code.write({.space = DspSpace::x, .address = shi_control, .value = shi_receive_enabled});
    } else {
      code.timer(csr);
    }
    if (kind == Kind::store) {
      code.emit(move_x0, short_r2_nine);
      code.emit(move_r0, vector_ + 1);
    }
    foreground_ = code.pc();
    const auto setup = code.instructions();
    code.emit(foreground_opcode);
    code.install(machine_.bus());
    const auto vector = words(kind);
    load_program(machine_.bus(), vector_, vector);
    constexpr std::array service{short_r3_nine, return_code};
    load_program(machine_.bus(), handler, service);
    const Driver driver(machine_);
    for (unsigned instruction = 0; instruction < setup; ++instruction) {
      dma_test::step_native(machine_);
    }
    if (kind == Kind::shi) {
      send(machine_.bus().shi());
    } else {
      machine_.bus().timers().advance_clk2(1);
    }
    check(machine_.admit_interrupt({.address = foreground_, .opcode = foreground_opcode}),
          "A real timer/SHI source admits the installed native continuation");
    initial_ = machine_.cpu().registers();
    instructions_ = machine_.cpu().progress().instructions;
  }

  [[nodiscard]] bool done() const {
    const auto regs = machine_.cpu().registers();
    return regs.pc == foreground_ + 1 && regs.r[7] == foreground_value &&
           !machine_.cpu().pending_instruction();
  }

  void advance() {
    const auto pending = machine_.cpu().pending_instruction();
    const auto result = pending ? machine_.advance(pending->ticket) : machine_.begin_fetch();
    check(result.fault.kind == dsp56300::FaultKind::none,
          "Every native IRQ/checkpoint suffix remains healthy");
  }

  Observation observe() {
    const auto pending = machine_.cpu().pending_instruction();
    const auto fetch = pending ? std::optional<dsp56300::InstructionFetch>{}
                               : std::optional{machine_.cpu().instruction_fetch()};
    return {.registers = machine_.cpu().registers(),
            .progress = machine_.cpu().progress(),
            .pending = pending ? std::optional{pending->effect} : std::nullopt,
            .request = machine_.bus().interrupt_request(0),
            .opcode = pending ? pending->opcode : fetch->latched_opcode,
            .vector = fetch ? std::optional{fetch->exception_vector} : std::nullopt,
            .address = pending ? pending->effect.pc : fetch->address,
            .second_word = machine_.bus().inspect_program(vector_ + 1),
            .timer_flag = machine_.bus().timers().read(csr, compare_flag),
            .ticks = machine_.clock().elapsed_ticks(),
            .acknowledging = machine_.interrupt_pending()};
  }

  void verify() {
    auto expected = initial_;
    unsigned retired = 3;
    switch (kind_) {
      case Kind::fast:
        expected.r[1] = 5;
        expected.r[2] = 7;
        break;
      case Kind::two_word:
        expected.x = (expected.x & ~std::uint64_t{dsp56300::word_mask}) | payload;
        retired = 2;
        break;
      case Kind::first_call:
      case Kind::second_call:
      case Kind::absolute:
        expected.r[3] = 9;
        expected.stack[1] = (std::uint64_t{foreground_} << word_bits) | expected.sr;
        retired = kind_ == Kind::second_call ? 5U : 4U;
        break;
      case Kind::shi:
        expected.x = (expected.x & ~std::uint64_t{dsp56300::word_mask}) | payload;
        expected.a = 0;
        // FM CLR: U/Z set for zero, E/N/V clear; C/L/S remain unchanged.
        expected.sr = (expected.sr & ~changed_ccr) | zero_ccr;
        break;
      case Kind::store:
        expected.r[2] = 7;
        check(machine_.bus().inspect_program(vector_ + 1) == short_r2_nine,
              "An actual P store changes RAM while the accepted second opcode retains seven");
        break;
      default: throw Fault("Invalid IRQ verification scenario");
    }
    expected.pc = foreground_ + 1;
    expected.r[7] = foreground_value;
    check(
        machine_.cpu().registers() == expected &&
            machine_.cpu().progress().instructions == instructions_ + retired &&
            !machine_.bus().interrupt_request(0),
        "Independent full register/frame/flag/count expectations hold after the actual foreground");
  }

  [[nodiscard]] OwnDspMachine &chip() noexcept { return machine_; }

  [[nodiscard]] const OwnDspMachine &chip() const noexcept { return machine_; }

private:
  static std::array<unsigned, 2> words(const Kind kind) {
    switch (kind) {
      case Kind::fast: return {short_r1_five, short_r2_seven};
      case Kind::two_word: return {move_x0, payload};
      case Kind::first_call: return {short_call, 0};
      case Kind::second_call: return {0, short_call};
      case Kind::absolute: return {absolute_call, handler};
      case Kind::shi: return {receive_x0, clear_a};
      case Kind::store: return {movem_x0_r0, short_r2_seven};
      default: throw Fault("Invalid authored IRQ checkpoint scenario");
    }
  }

  OwnDspMachine machine_{dma_test::native_clocks};
  Kind kind_ = Kind::fast;
  unsigned vector_ = 0, foreground_ = 0;
  dsp56300::Registers initial_{};
  std::uint64_t instructions_ = 0;
};

std::vector<Observation> suffix(Scenario &scenario) {
  std::vector<Observation> result{scenario.observe()};
  const auto ticks = scenario.chip().clock().elapsed_ticks();
  for (unsigned effect = 0; effect < suffix_bound && !scenario.done(); ++effect) {
    scenario.advance();
    result.push_back(scenario.observe());
  }
  check(scenario.done() && scenario.chip().clock().elapsed_ticks() == ticks,
        "A finite real IRQ/ISR/foreground suffix finishes without invented clock ticks");
  scenario.verify();
  return result;
}

void stale(Scenario &scenario, const std::optional<dsp56300::EffectTicket> &ticket) {
  if (!ticket) { return; }
  const auto before = scenario.observe();
  bool rejected = false;
  try {
    static_cast<void>(scenario.chip().advance(*ticket));
  } catch (const std::logic_error &) { rejected = true; }
  check(rejected && scenario.observe() == before,
        "Restoring either owner rejects the old native ticket before any peripheral effect");
}

unsigned checkpoints(const Kind kind) {
  Scenario original(kind);
  const auto complete = suffix(original);
  for (unsigned cut = 0; cut < complete.size(); ++cut) {
    Scenario source(kind);
    for (unsigned effect = 0; effect < cut; ++effect) { source.advance(); }
    const auto before = source.observe();
    const auto pending = source.chip().cpu().pending_instruction();
    const auto ticket = pending ? std::optional{pending->ticket} : std::nullopt;
    const auto saved = source.chip().checkpoint();
    Scenario fresh(kind);
    fresh.chip().restore(saved);
    check(fresh.observe() == before, "Fresh chip owner retains the complete accepted IRQ prefix");
    stale(fresh, ticket);
    const auto expected = suffix(source);
    source.chip().restore(saved);
    stale(source, ticket);
    check(suffix(source) == expected && suffix(fresh) == expected,
          "Same/fresh owners repeat every real IRQ effect without replaying FIFO or P writes");
  }
  return static_cast<unsigned>(complete.size());
}
}

void checkpoint_checks() {
  unsigned contexts = 0;
  for (const auto kind : kinds) { contexts += checkpoints(kind); }
  std::cout << "PASS GENERAL_IRQ_RESTORE contexts=" << contexts << " continuations=" << contexts * 2
            << " kinds=" << kinds.size() << '\n';
}
}
