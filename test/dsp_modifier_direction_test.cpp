// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/detail/dsp_pipeline.hpp"
#include "dsp/dsp_core.hpp"
#include "dsp/dsp_fetch_run.hpp"
#include "dsp/dsp_machine.hpp"
#include "dsp/dsp_reference_execution.hpp"
#include "dsp_fetch_run_fixture.hpp"
#include "dsp_modifier_move_fixture.hpp"
#include "dsp_norm_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>

namespace {
using namespace blofeld;
using namespace blofeld::test;
namespace fixture = blofeld::test::modifier_move;
// FM13-131/12-20: W=0,S1=00sss,D2=100ddd; the mandatory bit6 remains zero.
constexpr unsigned from_control = 0x0460a0;
constexpr unsigned field_shift = 8;
constexpr unsigned undocumented_bit = 0x40;
constexpr unsigned all_flags = 0xff;
constexpr unsigned high_bit = 0x800000;
constexpr unsigned clocks = 7;
unsigned streams = 0;
unsigned restores = 0;
unsigned clones = 0;

unsigned encoded(const fixture::Case sample) {
  return from_control | (sample.destination << field_shift) | sample.source;
}

void one(const fixture::Case sample) {
  DspMachine machine;
  fixture::prepare(machine, sample, encoded(sample));
  const auto before = machine.cpu().debug_state();
  DspFetchRun run(fixture::contract(), machine);
  for (unsigned period = 1; period <= clocks; ++period) {
    fixture::verify(
        machine, before, sample, run.advance_cycle(machine, fetch_clock::input()), period);
  }
  ++streams;
}

void matrix() {
  one({.source = 0, .destination = 1, .value = word_mask});
  for (unsigned source = 0; source < fixture::register_count; ++source) {
    for (unsigned destination = 0; destination < fixture::register_count; ++destination) {
      for (const unsigned value : {0U, high_bit, word_mask}) {
        for (const unsigned scale : {0U, scale_down, scale_up}) {
          for (const unsigned flags : {0U, all_flags}) {
            if (source == 0 && destination == 1 && value == word_mask && scale == 0 && flags == 0) {
              continue;
            }
            one({.source = source,
                 .destination = destination,
                 .value = value,
                 .scale = scale,
                 .flags = flags});
          }
        }
      }
    }
  }
}

void suffixes() {
  constexpr std::array cases{fixture::Case{.source = 0, .destination = 1, .value = word_mask},
                             fixture::Case{.source = 7, .destination = 0, .value = high_bit},
                             fixture::Case{.source = 3, .destination = 3, .value = 0}};
  for (const auto sample : cases) {
    for (unsigned cut = 0; cut < clocks; ++cut) {
      DspMachine machine;
      fixture::prepare(machine, sample, encoded(sample));
      const auto before = machine.cpu().debug_state();
      DspFetchRun run(fixture::contract(), machine);
      for (unsigned period = 1; period <= cut; ++period) {
        fixture::verify(
            machine, before, sample, run.advance_cycle(machine, fetch_clock::input()), period);
      }
      const auto saved = run.checkpoint(machine);
      for (unsigned replay = 0; replay < 2; ++replay) {
        run.restore(saved, machine);
        for (unsigned period = cut + 1; period <= clocks; ++period) {
          fixture::verify(
              machine, before, sample, run.advance_cycle(machine, fetch_clock::input()), period);
        }
        ++restores;
      }
      DspMachine target;
      DspFetchRun clone(saved, target);
      for (unsigned period = cut + 1; period <= clocks; ++period) {
        fixture::verify(
            target, before, sample, clone.advance_cycle(target, fetch_clock::input()), period);
      }
      require(target.cpu().debug_state() == machine.cpu().debug_state() &&
                  clone.elapsed_ticks() == run.elapsed_ticks(),
              "Fresh native ownership preserves the literal modifier endpoint and clock suffix");
      ++clones;
    }
  }
}

void fields_and_guards() {
  for (unsigned source = 0; source < fixture::register_count; ++source) {
    for (unsigned destination = 0; destination < fixture::register_count; ++destination) {
      const fixture::Case sample{.source = source, .destination = destination};
      const DspMoveFields expected{.m = static_cast<std::uint8_t>(1U << destination)};
      require(pipeline_transfer_fields(encoded(sample)) == expected &&
                  pipeline_transfer_fields(fixture::opcode(sample)) == expected,
              "Both documented W directions select exactly the independent destination M field");
    }
  }
  fixture::guards();
  fixture::refusal({}, from_control | (1U << field_shift) | undocumented_bit);
}
}

int main() {
  try {
    matrix();
    suffixes();
    fields_and_guards();
    std::cout << "PASS MODIFIER_DIRECTION streams=" << streams << " restores=" << restores
              << " fresh-owner-clones=" << clones << " field-pairs=64 guards=15\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
