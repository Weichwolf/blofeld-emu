// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {
using namespace blofeld;
constexpr unsigned base = 0x100;
constexpr unsigned ordinary_omr = 0x300;
constexpr unsigned unmapped = 0x400000;
constexpr unsigned immediate_omr = 0x05f43a;

void require(const bool condition, const std::string &message) {
  if (!condition) { throw Fault(message); }
}

void refuses(const auto &operation, const std::string &expected) {
  try {
    operation();
  } catch (const Fault &error) {
    require(std::string(error.what()).contains(expected),
            "Refusal lost exact context: " + expected);
    return;
  }
  throw Fault("Unknown memory sample unexpectedly supplied a value");
}

void capture(DspProgramFetch &fetch, DspBus &bus) {
  fetch.issue();
  fetch.capture(bus);
}

void unknown_and_fresh() {
  DspBus bus;
  require(!bus.sample_program(base), "Mapped unwritten P memory has no invented value");
  refuses([&] { static_cast<void>(bus.read({.address = base})); },
          "Unknown DSP memory read at P:" + hex(base, 6));
  DspProgramFetch fetch;
  fetch.start(base, ordinary_omr);
  capture(fetch, bus);
  const auto unknown = fetch.sampled(0);
  if (!unknown) { throw Fault("Actual unknown F2 receipt is missing"); }
  require(unknown && !unknown->sample.data && unknown->sample.address == base &&
              unknown->occurrence != 0 && fetch.view().used == 1 && !fetch.view().pending_address &&
              fetch.view().next_address == base + 1,
          "Actual unknown F2 retains exact address, occurrence, FIFO slot and completed F1");
  const auto before = fetch.view();
  const auto error = "Unknown DSP program sample read P:" + hex(base, 6) + " pc=" + hex(base, 6) +
                     " omr=" + hex(ordinary_omr, 6);
  refuses([&] { static_cast<void>(fetch.queued(0)); }, error);
  refuses([&] { fetch.check_consume(base); }, error);
  refuses([&] { static_cast<void>(fetch.consume(base)); }, error);
  require(fetch.view() == before, "Refused consumption preserves the actual unknown sample");
  auto restored = fetch;
  bus.write({.address = base});
  require(bus.sample_program(base) == 0 && !fetch.peek(base) && !restored.peek(base) &&
              fetch.sampled(0) == unknown && restored.sampled(0) == unknown,
          "Later known-zero RAM writes cannot resolve captured or copied unknown F2 samples");
  refuses([&] { static_cast<void>(restored.consume(base)); }, error);
  restored.redirect(base, ordinary_omr);
  capture(restored, bus);
  const auto fresh = restored.sampled(0);
  require(fresh && fresh->sample.data == 0 && fresh->occurrence > unknown->occurrence &&
              restored.consume(base) == 0 && fetch.sampled(0) == unknown,
          "Only a new actual F1/F2 occurrence can see zero after the write");
}

void strict_extension_and_map() {
  DspBus bus;
  bus.write({.address = base, .value = immediate_omr});
  DspProgramFetch fetch;
  fetch.start(base, ordinary_omr);
  capture(fetch, bus);
  capture(fetch, bus);
  const auto opcode = fetch.queued(0);
  require(opcode.has_value(), "Own two-word opcode is an actual known F2 receipt");
  const auto before = fetch.view();
  refuses([&] { static_cast<void>(fetch.instruction(*opcode, 2)); },
          "Unknown DSP program sample read P:" + hex(base + 1, 6) + " pc=" + hex(base, 6) +
              " omr=" + hex(ordinary_omr, 6));
  require(fetch.view() == before, "An unknown used extension cannot consume its opcode");
  refuses([&] { static_cast<void>(bus.sample_program(unmapped)); },
          "Unmapped DSP memory read at P:" + hex(unmapped, 6));
  DspProgramFetch missing;
  missing.start(unmapped, ordinary_omr);
  missing.issue();
  refuses([&] { missing.capture(bus); },
          "Native F2 read P:" + hex(unmapped, 6) + " pc=" + hex(unmapped, 6) +
              " omr=" + hex(ordinary_omr, 6));
  require(missing.view().used == 0 && missing.view().pending_address == unmapped,
          "Unsupported accesses never become unknown mapped-memory samples");
}
}

int main() {
  try {
    unknown_and_fresh();
    strict_extension_and_map();
    std::cout
        << "PASS mapped unknown/known-zero,old and fresh occurrences,copy,extension,map guards\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
