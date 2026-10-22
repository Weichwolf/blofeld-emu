// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"
#include "dsp_master_output_fixture.hpp"
#include "dsp_master_output_model.hpp"
#include "dsp_norm_fixture.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <ostream>

namespace blofeld::test::master {
namespace {
void matrix(Fixture &fixture, std::ostream &trace) {
  constexpr unsigned maximum_volume = 127;
  for (unsigned volume = 0; volume <= maximum_volume; ++volume) {
    for (const auto scale : {0U, down, up}) {
      for (const auto phase : {0U, 32U}) {
        for (const bool twos : {false, true}) {
          for (unsigned pattern = 0; pattern < 4; ++pattern) {
            const Input input{
                .volume = volume, .scale = scale, .phase = phase, .pattern = pattern, .twos = twos};
            const auto source = samples(pattern);
            const auto expected = predict(input, source, fixture.coefficients());
            const bool replay = volume == maximum_volume && !twos && pattern == 2;
            fixture.run(input, source, expected, replay);
            trace << "CASE " << volume << ' ' << scale << ' ' << phase << ' ' << twos << ' '
                  << pattern << ' ' << expected.clipped << '\n';
          }
        }
      }
    }
  }
}

void streams(Fixture &fixture, std::ostream &trace) {
  for (const auto scale : {0U, down, up}) {
    for (const bool twos : {false, true}) {
      History previous{};
      for (unsigned block = 0; block < 4; ++block) {
        auto source = samples(block);
        if (block != 0) { source.history = previous; }
        constexpr unsigned volume = 64;
        constexpr unsigned half_words = 32;
        const Input input{.volume = volume,
                          .scale = scale,
                          .phase = (block % 2) * half_words,
                          .pattern = block,
                          .twos = twos,
                          .dependent = block != 0};
        const auto expected = predict(input, source, fixture.coefficients());
        fixture.run(input, source, expected, false);
        previous = expected.history;
        trace << "STREAM " << scale << ' ' << twos << ' ' << block << '\n';
      }
    }
  }
}

void run(const Firmware &firmware, std::ostream &trace) {
  Fixture fixture(firmware);
  matrix(fixture, trace);
  streams(fixture, trace);
  fixture.failures();
  const Input recovery{.volume = 127, .pattern = 2};
  const auto source = samples(recovery.pattern);
  fixture.run(recovery, source, predict(recovery, source, fixture.coefficients()), false);
  fixture.intact();
  trace << "RESULT " << fixture.calls() << ' ' << fixture.restores() << ' '
        << fixture.failed_reads() << '\n';
  std::cout
      << "Original master output: independent lane arithmetic, real accesses, native returns, "
         "dependent histories and trusted fault recovery verified; no boot, voice or physical "
         "audio claim.\n";
}
}
}

int main(const int count, char **arguments) {
  try {
    blofeld::test::require(count == 3,
                           "Usage: original-dsp-master-output firmware.zip private.trace");
    std::ofstream trace(arguments[2]);
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    blofeld::test::master::run(blofeld::Firmware::load(arguments[1]), trace);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
