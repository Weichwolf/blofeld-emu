// SPDX-License-Identifier: GPL-3.0-only
#include "blofeld/firmware.hpp"
#include "dsp_norm_fixture.hpp"
#include "original_dsp_gpio_request_fixture.hpp"

#include <array>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <ostream>

namespace {
using namespace blofeld::test::gpio_request;
using blofeld::test::require;

// Fixed peer bytes, independent source addresses and own data words.
// The selected routes/bounds are transcribed from the recovered firmware.
constexpr std::array<Case, 22> inputs{
    {{.first = 0, .second = 0, .address = 0x1200, .value = 0},
     {.first = 3, .second = 127, .address = 0x13ff, .value = 0xffffff},
     {.first = 0x3c, .second = 0, .address = 0x1980, .value = 0x12347f},
     {.first = 0x3f, .second = 127, .address = 0x1b7f, .value = 0x800080},
     {.first = 0x1d, .second = 64, .address = 0x1640, .value = 0xf00001, .upper = 0xa00},
     {.kind = Kind::multi, .first = 0x40, .second = 0x40, .address = 0xe816, .value = 0},
     {.kind = Kind::multi,
      .first = 0x4f,
      .second = 0x7f,
      .address = 0xe855,
      .value = 0xffffff,
      .upper = 0xa00},
     {.kind = Kind::multi,
      .first = 0x50,
      .second = 0x60,
      .address = 0xe836,
      .value = 0x800080,
      .upper = 0xa00},
     {.kind = Kind::metadata, .first = 0x40, .second = 0, .address = 0xe836, .value = 1},
     {.kind = Kind::metadata,
      .first = 0x4f,
      .second = 0x3f,
      .address = 0xe9dd,
      .value = 0x76547f,
      .upper = 0xa00},
     {.kind = Kind::metadata, .first = 0x47, .second = 24, .address = 0xe8f6, .value = 0x123456},
     {.kind = Kind::global, .first = 0x60, .second = 0, .address = 0x720, .value = 0},
     {.kind = Kind::global,
      .first = 0x6f,
      .second = 0x7f,
      .address = 0x79f,
      .value = 0x800080,
      .upper = 0xa00},
     {.kind = Kind::global,
      .first = 0x68,
      .second = 64,
      .address = 0x760,
      .value = 0x1234ff,
      .upper = 0xa00},
     {.kind = Kind::deferred, .first = 0x70, .second = 0x55, .upper = 0xa00},
     {.stop = Stop::low},
     {.first = 3, .second = 127, .address = 0x13ff, .stop = Stop::high},
     {.failure = Failure::low_unknown},
     {.failure = Failure::second_unknown},
     {.first = 0x3c, .address = 0x1980, .failure = Failure::index_unknown},
     {.first = 3, .second = 127, .address = 0x13ff, .failure = Failure::word_unknown},
     {.first = 3, .second = 127, .address = 0x13ff, .failure = Failure::contention}}};

void run(const blofeld::Firmware &firmware, std::ostream &trace) {
  unsigned number = 0;
  unsigned instructions = 0;
  unsigned restores = 0;
  for (const auto input : inputs) {
    trace << "BEGIN " << number << ' ' << input.first << ' ' << input.second << ' ' << input.address
          << '\n';
    trace.flush();
    Rig rig(firmware, input);
    const auto saved = rig.save();
    const auto turnaround = rig.turnaround();
    rig.finish();
    const auto expected = rig.outcome();
    for (unsigned replay = 0; replay < 2; ++replay) {
      rig.restore(saved);
      ++restores;
      rig.finish();
      require(rig.outcome() == expected,
              "Joint CPU/bus and peer-phase restores reproduce every actual F4 effect");
    }
    if (turnaround.has_value()) {
      rig.restore(*turnaround);
      ++restores;
      rig.finish();
      require(rig.outcome() == expected,
              "A mid-turnaround restore retains pending peer release and the real GPIO prefix");
    }
    instructions += expected.stats.steps;
    trace << "CASE " << number++ << " steps=" << expected.stats.steps
          << " low=" << expected.stats.low << " high=" << expected.stats.high
          << " reads=" << expected.stats.source_reads << " reply=";
    if (expected.stats.reply.has_value()) {
      trace << *expected.stats.reply;
    } else {
      trace << "none";
    }
    trace << '\n';
  }
  trace << "RESULT cases=" << number << " restores=" << restores
        << " instructions_excluding_replays=" << instructions << '\n';
  std::cout
      << "Original F4 GPIO request routes, real source reads/replies, waits and fault recovery "
         "checked under explicit untimed own context. No board boot or physical timing.\n";
}
}

int main(const int count, char **arguments) {
  try {
    require(count == 3, "Usage: original-dsp-gpio-request-test firmware.zip private.trace");
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot create private original F4 request trace");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    run(blofeld::Firmware::load(std::filesystem::path(arguments[1])), trace);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
