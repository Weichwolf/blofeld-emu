// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_loop_fetch.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

namespace blofeld::test::loop_identity {
namespace {
constexpr unsigned setup = 0x100;
constexpr unsigned body = setup + 2;
constexpr unsigned forever_do = 0x000203;
constexpr unsigned omr = 0x305;
constexpr unsigned status = 0xc00300;
constexpr unsigned loop_status = status | 0x18000;
constexpr unsigned count = 7;
constexpr unsigned last = 0x321;
constexpr std::size_t workers = 2;
constexpr std::size_t iterations = 1024;

void require(const bool value, const std::string &message) {
  if (!value) { throw Fault(message); }
}

DspFetchedWord queued(const DspProgramFetch &fetch, const std::size_t position) {
  const auto result = fetch.queued(position);
  if (!result) { throw Fault("Expected a real captured F2 word in the identity test"); }
  return *result;
}

void read(DspProgramFetch &fetch, DspBus &bus) {
  fetch.issue();
  fetch.capture(bus);
}

DspLoopArchitecture native_result() {
  // Independent control endpoint already checked against actual native FOREVER.
  // This test checks host identity concurrency, not instruction or elapsed timing.
  return {.pc = body, .omr = omr, .la = body, .lc = count, .sp = 2, .sc = 2, .status = loop_status};
}

void install(DspBus &bus) {
  bus.write({.address = setup, .value = forever_do});
  bus.write({.address = setup + 1, .value = body});
  bus.write({.address = body, .value = 0});
}

DspProgramFetch prepare(DspBus &bus) {
  DspProgramFetch fetch;
  fetch.start(setup, omr);
  read(fetch, bus);
  read(fetch, bus);
  const auto opcode = queued(fetch, 0);
  const auto extension = queued(fetch, 1);
  const DspLoopOpening opening{
      .word = opcode.word,
      .extension = extension.word,
      .before = {.pc = setup, .omr = omr, .la = last, .lc = count, .status = status},
      .after = native_result()};
  require(fetch.consume(setup) == forever_do && fetch.consume(setup + 1) == body,
          "Both actual P words must precede the identity test's loop context");
  fetch.open_loop(opening, opcode, extension);
  require(fetch.confirm_opening(native_result(), false), "Loop identity fixture opening differs");
  read(fetch, bus);
  return fetch;
}

void concurrent_copies() {
  std::array<DspBus, workers> buses;
  for (auto &bus : buses) { install(bus); }
  const auto source = prepare(buses[0]);
  const auto prior = queued(source, 0).occurrence;
  std::array contexts{source, source};
  std::array<std::array<std::uint64_t, iterations>, workers> results{};
  std::array<std::exception_ptr, workers> failures{};
  std::array<std::jthread, workers> threads;
  for (std::size_t slot = 0; slot < workers; ++slot) {
    threads.at(slot) = std::jthread([&, slot] {
      try {
        auto &fetch = contexts.at(slot);
        auto &bus = buses.at(slot);
        for (std::size_t index = 0; index < iterations; ++index) {
          read(fetch, bus);
          const auto fresh = queued(fetch, 1);
          require(fresh.word.address == body && fresh.word.value == 0 && fresh.occurrence > prior,
                  "Each owned frontend must capture an actual fresh same-PC word");
          const auto selected = fetch.instruction(fresh, 1);
          require(selected.opcode == fresh && !selected.extension,
                  "Exact instruction selection must retain this copy's fresh same-PC occurrence");
          const auto older = fetch.instruction(queued(fetch, 0), 1);
          older.check_front(fetch);
          results.at(slot).at(index) = fresh.occurrence;
          require(fetch.consume(body) == 0, "Each copy must consume its actual older F2 word");
          older.check_consumed(fetch.consumed_pair(), omr);
          require(fetch.confirm_loop(native_result(), true),
                  "Each thread must confirm only its own ordered loop-end continuation");
        }
      } catch (...) { failures.at(slot) = std::current_exception(); }
    });
  }
  for (auto &thread : threads) { thread.join(); }
  for (const auto &failure : failures) {
    if (failure) { std::rethrow_exception(failure); }
  }
  std::array<std::uint64_t, workers * iterations> all{};
  for (std::size_t slot = 0; slot < workers; ++slot) {
    const auto &row = results.at(slot);
    require(std::ranges::is_sorted(row) && std::ranges::adjacent_find(row) == row.end(),
            "A single frontend's actual F2 identities must remain strictly ordered");
    std::ranges::copy(row, all.begin() + static_cast<std::ptrdiff_t>(slot * iterations));
  }
  std::ranges::sort(all);
  require(std::ranges::adjacent_find(all) == all.end(),
          "Concurrent copied frontends must never alias newly captured F2 identities");
}
}
}

int main() {
  try {
    blofeld::test::loop_identity::concurrent_copies();
    std::cout << "Two owned frontend copies captured 2048 unique F2 identities with exact receipt "
                 "checks\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
