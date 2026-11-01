// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dma_native_fixture.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"
#include "dsp/own_dsp_fetch.hpp"
#include "dsp/own_dsp_machine.hpp"
#include "dsp56300/core.hpp"
#include "esai_fixture.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>

namespace {
using namespace blofeld;
using test::esai::require;
namespace native = test::dma_test;
// FM MOVEC SR, MOVE X0/R0/R7, TRAP, MOVEM X0,P:(R0) and short MOVE #7/#9,R2.
constexpr unsigned move_sr = 0x05f439, move_x0 = 0x44f400, move_r0 = 0x60f400;
constexpr unsigned move_r7 = 0x67f400, trap = 6, program_store = 0x076084;
constexpr unsigned original = 0x320700, replacement = 0x320900;
constexpr unsigned irqa = 0x10, software_vector = 8, priorities = 0xffffff;
constexpr unsigned effect_bound = 32;

enum class Stage : std::uint8_t {
  begin,
  read_first,
  read_second,
  store,
  retire_first,
  synchronize_first,
  begin_second,
  retire_second,
  synchronize_second,
  foreground,
  done
};

struct Meta {
  bool software;
  Stage stage = Stage::begin;
  unsigned vector = 0, foreground = 0, reads = 0;
  dsp56300::Registers initial{};
  std::uint64_t instructions = 0, ticks = 0;
};

void retire(OwnDspMachine &machine) {
  for (unsigned effect = 0; effect < effect_bound; ++effect) {
    const auto selected = machine.cpu().pending_instruction();
    if (!selected) { throw Fault("Vector fixture lost its actual native effect"); }
    const auto result = machine.advance(selected->ticket);
    require(result.fault.kind == dsp56300::FaultKind::none, "Actual native vector remains healthy");
    if (result.retired) { return; }
  }
  throw Fault("Vector fixture exceeded its native effect bound");
}

void read(OwnDspFetch &fetch) {
  fetch.issue();
  fetch.capture();
}

class Scenario {
public:
  explicit Scenario(const bool software) : meta_{.software = software} {
    meta_.vector = software ? software_vector : irqa;
    native::NativeProgram code;
    for (const auto word : {move_sr, 0U, move_x0, replacement, move_r0, meta_.vector + 1}) {
      code.word(word);
    }
    code.write({.space = DspSpace::x, .address = priorities, .value = 6});
    if (software) { code.word(trap); }
    meta_.foreground = code.pc();
    code.word(move_r7);
    constexpr unsigned foreground_value = 0x77;
    code.word(foreground_value);
    native::reset_native(machine_, code);
    machine_.bus().write({.address = meta_.vector, .value = program_store});
    machine_.bus().write({.address = meta_.vector + 1, .value = original});
    for (unsigned instruction = 0; instruction < 4; ++instruction) {
      native::step_native(machine_);
    }
    fetch_ = std::make_unique<OwnDspFetch>(machine_);
    if (software) {
      read(*fetch_);
      static_cast<void>(fetch_->begin());
      retire(machine_);
      fetch_->synchronize();
    } else {
      read(*fetch_);
      read(*fetch_);
      machine_.bus().observe_irqa(false);
      machine_.bus().observe_irqa(true);
      require(fetch_->admit_interrupt(),
              "Actual released IRQA edge admits the captured foreground");
    }
    meta_.initial = machine_.cpu().registers();
    meta_.instructions = machine_.cpu().progress().instructions;
    meta_.ticks = machine_.clock().elapsed_ticks();
    require(meta_.initial.pc == meta_.foreground &&
                machine_.cpu().instruction_fetch().exception_vector,
            "Real setup/TRAP or admitted IRQ establishes native vector ownership");
  }

  Scenario(const Meta meta, const OwnDspFetchCheckpoint &checkpoint) : meta_(meta) {
    fetch_ = std::make_unique<OwnDspFetch>(machine_, checkpoint);
  }

  [[nodiscard]] Meta meta() const { return meta_; }

  [[nodiscard]] OwnDspFetchCheckpoint checkpoint() const { return fetch_->checkpoint(); }

  void step() {
    using enum Stage;
    switch (meta_.stage) {
      case begin: static_cast<void>(fetch_->begin()); break;
      case read_first:
      case read_second: vector_read(); break;
      case store: program_write(); break;
      case retire_first:
        retire(machine_);
        require(machine_.cpu().progress().instructions == meta_.instructions + 1 &&
                    machine_.cpu().registers() == meta_.initial &&
                    machine_.cpu().instruction_fetch().exception_vector &&
                    machine_.cpu().instruction_fetch().latched_opcode == original,
                "First real retirement retains the second vector opcode after actual P mutation");
        break;
      case synchronize_first:
        fetch_->synchronize();
        require(machine_.cpu().instruction_fetch().exception_vector &&
                    !machine_.interrupt_pending(),
                "Acknowledged peripheral and remaining native vector have distinct ownership");
        break;
      case begin_second: {
        const auto progress = fetch_->begin();
        require(progress.pending &&
                    progress.pending->effect.kind != dsp56300::EffectKind::fetch_opcode &&
                    progress.pending->effect.kind != dsp56300::EffectKind::fetch_extension,
                "The accepted second opcode is never sampled again from changed P RAM");
        break;
      }
      case retire_second: retire(machine_); break;
      case synchronize_second:
        fetch_->synchronize();
        require(!machine_.cpu().instruction_fetch().exception_vector &&
                    fetch_->view().consume_address == meta_.foreground && fetch_->view().used == 0,
                "Only native vector completion releases F2 to the real foreground");
        break;
      case foreground:
        read(*fetch_);
        read(*fetch_);
        static_cast<void>(fetch_->begin());
        retire(machine_);
        fetch_->synchronize();
        break;
      case done:
      default: throw Fault("Vector fixture cannot advance its completed script");
    }
    meta_.stage = static_cast<Stage>(static_cast<unsigned>(meta_.stage) + 1);
  }

  void complete() {
    while (meta_.stage != Stage::done) { step(); }
    auto expected = meta_.initial;
    expected.r[2] = 7;
    constexpr unsigned foreground_value = 0x77;
    expected.r[7] = foreground_value;
    expected.pc += 2;
    require(machine_.cpu().registers() == expected &&
                machine_.cpu().progress().instructions == meta_.instructions + 3 &&
                machine_.clock().elapsed_ticks() == meta_.ticks && meta_.reads == 2 &&
                machine_.bus().inspect_program(meta_.vector + 1) == replacement &&
                !machine_.bus().interrupt_request(0),
            "Real P store, retained-seven vector and foreground preserve independent full-state "
            "expectations");
  }

  [[nodiscard]] const OwnDspMachine &machine() const { return machine_; }

private:
  void vector_read() {
    const auto pending = machine_.cpu().pending_instruction();
    if (!pending) { throw Fault("Vector fixture lost its program read"); }
    require(pending->effect.kind == (meta_.reads == 0 ? dsp56300::EffectKind::fetch_opcode
                                                      : dsp56300::EffectKind::fetch_extension) &&
                pending->effect.space == dsp56300::Space::program &&
                pending->effect.address == meta_.vector + meta_.reads &&
                pending->effect.pc == meta_.vector,
            "Both actual vector reads precede the first instruction with exact addresses and PC");
    const auto result = machine_.advance(pending->ticket);
    ++meta_.reads;
    require(result.fault.kind == dsp56300::FaultKind::none &&
                machine_.cpu().progress().instructions == meta_.instructions &&
                machine_.interrupt_pending() == (!meta_.software && meta_.reads == 1),
            "No native retirement precedes the pair; external acknowledgement follows only its "
            "second read");
  }

  void program_write() {
    const auto pending = machine_.cpu().pending_instruction();
    if (!pending) { throw Fault("Vector fixture lost its actual P write"); }
    require(meta_.reads == 2 && pending->effect.kind == dsp56300::EffectKind::write &&
                pending->effect.space == dsp56300::Space::program &&
                pending->effect.address == meta_.vector + 1 &&
                pending->effect.value == replacement && pending->effect.pc == meta_.vector,
            "The first vector instruction offers its actual self-modifying store after both reads");
    const auto result = machine_.advance(pending->ticket);
    require(result.fault.kind == dsp56300::FaultKind::none &&
                machine_.bus().inspect_program(meta_.vector + 1) == replacement,
            "The actual native store changes P RAM while its accepted opcode remains separate");
  }

  OwnDspMachine machine_{native::native_clocks};
  std::unique_ptr<OwnDspFetch> fetch_;
  Meta meta_;
};
}

int main() {
  try {
    unsigned contexts = 0;
    for (const bool software : {false, true}) {
      for (unsigned cut = 0; cut < static_cast<unsigned>(Stage::done); ++cut) {
        Scenario source(software);
        for (unsigned prefix = 0; prefix < cut; ++prefix) { source.step(); }
        Scenario fresh(source.meta(), source.checkpoint());
        source.complete();
        fresh.complete();
        require(source.machine().cpu().registers() == fresh.machine().cpu().registers() &&
                    source.machine().cpu().progress() == fresh.machine().cpu().progress(),
                "Every before/after-read/write/retirement/synchronization cut restores actual "
                "native state");
        ++contexts;
      }
    }
    constexpr unsigned expected = 20;
    require(contexts == expected, "Both source types execute every declared checkpoint cut");
    std::cout << "PASS OWN_VECTOR_PAIR contexts=20 continuations=40\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
