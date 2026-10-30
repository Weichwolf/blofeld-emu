// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_memory_form.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_checkpoint_fixture.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_norm_fixture.hpp"
#include "dsp_scalar_store_fixture.hpp"

#include <array>
#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
using test::require;
constexpr unsigned entry = 0x100, data = 0x407, omr = 0x300, cookie = 0x123456;
constexpr unsigned read_bit = 0x8000;
constexpr unsigned scalar_base = 0x400000, absolute_mode = 0x7000;
constexpr unsigned control_base = 0x057020, y_bit = 0x80000, control_y = 0x40;
constexpr unsigned high_fields = 0x18, high_shift = 17, low_fields = 7, low_shift = 16;
constexpr unsigned source_count = 32, reserved_scale = 0xc00, bound = 8;
constexpr unsigned io_begin = 0xffff80, hcsr = 0xffff91, last = 0xffffff;
constexpr unsigned x_unmapped = 0x3000, y_unmapped = 0x2000, rom = 0x4000;
constexpr unsigned parallel_clr = 0x13, m_field = 0, x0_field = 4, r6_field = 22, a_field = 14;
unsigned fields = 0, faults = 0;

unsigned encoded(const unsigned field, const bool control, const bool load, const bool y) {
  if (control) { return control_base | field | (load ? read_bit : 0U) | (y ? control_y : 0U); }
  return scalar_base | ((field & high_fields) << high_shift) | ((field & low_fields) << low_shift) |
         absolute_mode | (load ? read_bit : 0U) | (y ? y_bit : 0U);
}

void field(const unsigned opcode, const bool valid, const bool y, const bool load) {
  const auto form = decode_memory_stage(opcode);
  require(form.has_value() == valid, "Independent FM scalar/MOVEC register field admission");
  if (valid) {
    if (!form) { throw Fault("Qualified own scalar field has no native memory form"); }
    require(form->lane_count == 1 && !form->peripheral && !form->long_pair &&
                !form->absolute_short && form->lanes[0].mode == 6 && form->lanes[0].reg == 0 &&
                form->lanes[0].space == (y ? DspSpace::y : DspSpace::x) &&
                form->lanes[0].operation ==
                    (load ? DspMemoryOperation::read : DspMemoryOperation::write) &&
                decode_pipeline_word(
                    {.address = entry, .value = opcode}, omr, DspFetchPipeline::automatic_memory)
                        .words == 2,
            "Only literal absolute24 address fields qualify with the exact direction and space");
  }
  ++fields;
}

void register_fields(const bool control, const bool load, const bool y) {
  for (unsigned reg = 0; reg < source_count; ++reg) {
    const bool valid = control ? reg < 8 : reg >= 4;
    const auto word = encoded(reg, control, load, y);
    field(word, valid, y, load);
    if (!valid) { continue; }
    if (!control) { field(word | parallel_clr, false, y, load); }
    // MMM6/RRR0 alone is absolute24. RRR4 belongs to immediate transfers.
    for (unsigned index = 1; index < 8; ++index) {
      const auto other = word | (index << 8U);
      require(!decode_memory_stage(other), "Other extended selectors cannot become absolute24 RAM");
      ++fields;
    }
  }
}

struct Case {
  unsigned word;
  unsigned address;
  std::string marker;
  unsigned flags = 0;
};

DspDebugState prepare(DspMachine &machine, const Case &sample) {
  test::DspProgram code;
  code.immediate("omr", omr);
  code.immediate("r6", cookie);
  code.immediate("x0", cookie);
  code.immediate("sr", test::masked_status | sample.flags);
  code.word(test::scalar_store::absolute_jump);
  code.word(entry);
  code.install(machine.bus());
  machine.bus().pll().observe_pinit(false);
  machine.cpu().reset_spi_boot();
  test::checkpoint::until(machine, entry);
  for (unsigned index = 0; index < 16; ++index) {
    machine.bus().write({.address = entry + index, .value = 0});
  }
  machine.bus().write({.address = entry, .value = sample.word});
  machine.bus().write({.address = entry + 1, .value = sample.address});
  for (const auto space : {DspSpace::x, DspSpace::y}) {
    machine.bus().write({.space = space, .address = data - 1, .value = cookie});
    machine.bus().write({.space = space, .address = data + 1, .value = cookie});
  }
  const auto before = machine.cpu().debug_state();
  static_cast<void>(
      machine.begin_reference_instruction(DspReferenceExecution::staged_register_boundaries));
  return before;
}

void suffix(DspFetchRun &run,
            DspMachine &machine,
            const DspDebugState &before,
            const Case &sample) {
  for (unsigned index = 0; index < bound; ++index) {
    try {
      static_cast<void>(run.advance_cycle(machine, test::fetch_clock::input()));
    } catch (const Fault &error) {
      const std::string text = error.what();
      require(text.contains(sample.marker) && text.contains("pc=" + hex(entry, 6)) &&
                  text.contains("omr=" + hex(omr, 6)) && run.faulted() &&
                  machine.cpu().debug_state() == before,
              "Unknown/unsupported access preserves complete native state and reports context: " +
                  text);
      for (const auto space : {DspSpace::x, DspSpace::y}) {
        for (const auto address : {data - 1, data + 1}) {
          require(machine.bus().read({.space = space, .address = address}) == cookie,
                  "Faulted native memory access preserves independent guard cells");
        }
      }
      static_cast<void>(
          test::checkpoint::rejects([&] { static_cast<void>(run.checkpoint(machine)); },
                                    "Faulted native access cannot publish a trusted checkpoint"));
      ++faults;
      return;
    }
  }
  throw Fault("Unsupported native memory operation completed without its declared refusal");
}

void one(const Case &sample) {
  DspMachine machine;
  const auto before = prepare(machine, sample);
  DspFetchRun run(test::scalar_store::contract(), machine);
  for (unsigned index = 0; index < 2; ++index) {
    static_cast<void>(run.advance_cycle(machine, test::fetch_clock::input()));
  }
  const auto cut = run.checkpoint(machine);
  for (unsigned replay = 0; replay < 2; ++replay) {
    run.restore(cut, machine);
    suffix(run, machine, before, sample);
  }
  DspMachine target;
  DspFetchRun clone(cut, target);
  suffix(clone, target, before, sample);
}

void peripheral(const bool load, const bool y) {
  for (const unsigned source : {m_field, x0_field, r6_field}) {
    for (const unsigned address : {io_begin, hcsr, last}) {
      const auto *const operation = load ? "read" : "write";
      const auto *const space = y ? "Y:" : "X:";
      one({.word = encoded(source, source == m_field, load, y),
           .address = address,
           .marker = "Unqualified absolute data peripheral " + std::string(operation) + " at " +
                     space + hex(address, 6)});
    }
  }
}

void memory(const bool load, const bool y) {
  const auto word = encoded(r6_field, false, load, y);
  const auto address = y ? y_unmapped : x_unmapped;
  const auto *const space = y ? "Y:" : "X:";
  one({.word = word, .address = address, .marker = space + hex(address, 6)});
  one({.word = word, .address = rom, .marker = space + hex(rom, 6)});
  if (load) {
    one({.word = word,
         .address = data,
         .marker = "Unknown DSP memory read at " + std::string(space)});
  } else {
    one({.word = encoded(a_field, false, false, y),
         .address = data,
         .marker = "accumulator MOVE scaling read P:",
         .flags = reserved_scale});
  }
}
}

int main() {
  try {
    for (const bool load : {false, true}) {
      for (const bool y : {false, true}) {
        register_fields(false, load, y);
        register_fields(true, load, y);
        peripheral(load, y);
        memory(load, y);
      }
    }
    std::cout << "PASS ABSOLUTE_DATA_GUARDS fields=" << fields << " faulted_suffixes=" << faults
              << '\n';
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
