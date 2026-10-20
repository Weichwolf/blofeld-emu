// SPDX-License-Identifier: GPL-3.0-only
#include "bank_fixture.hpp"
#include "blofeld/bus.hpp"
#include "blofeld/dsp_dma.hpp"
#include "dma_fixture.hpp"
#include "esai_fixture.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace blofeld::test::banks {
namespace {
using esai::rejects;
using esai::require;
enum class Failure : std::uint8_t { none, unknown, standard, non_standard };

struct Engine {
  struct Write {
    DspDmaAddress address;
    unsigned value;
  };

  DspDma dma;
  std::vector<DspDmaPhase> phases;
  std::vector<DspDmaAddress> reads;
  std::vector<Write> writes;
  unsigned sample = first;
  DspDmaGrant decision = DspDmaGrant::available;
  Failure failure = Failure::none;

  std::optional<DspDmaWord> step(const Cycle cycle) {
    return dma.advance(inputs(cycle),
                       {.read =
                            [this](const DspDmaAddress address) {
                              reads.push_back(address);
                              return sample;
                            },
                        .write =
                            [this](const DspDmaAddress address, const std::uint32_t value) {
                              writes.push_back({.address = address, .value = value});
                            },
                        .admit =
                            [this](const DspDmaPhase &phase) {
                              phases.push_back(phase);
                              if (failure == Failure::unknown) { return DspDmaGrant::unknown; }
                              if (failure == Failure::standard) {
                                throw Fault("Own admission failure");
                              }
                              if (failure == Failure::non_standard) { throw 7; }
                              return decision;
                            }});
  }
};

void denied_and_global_wait() {
  Engine test;
  static_cast<void>(test.step({.index = 0}));
  transfer::configure(test.dma, transfer::word_copy);
  test.decision = DspDmaGrant::wait;
  require(!test.step({.index = 1, .requests = 1}) && test.reads.empty() &&
              test.phases.back().kind == read_phase && !test.phases.back().value,
          "Admission sees the newly sampled rising request before any source read");
  test.decision = DspDmaGrant::available;
  static_cast<void>(test.step({.index = 2, .requests = 1}));
  test.sample = second;
  test.decision = DspDmaGrant::wait;
  require(!test.step({.index = 3, .requests = 1}) && test.reads.size() == 1 &&
              test.writes.empty() && test.phases.back().kind == write_phase &&
              test.phases.back().value == first,
          "Denied destination access retains the already captured source word");
  test.decision = DspDmaGrant::available;
  require(!test.step({.index = 4, .requests = 1, .grant = DspDmaGrant::wait}) &&
              test.phases.size() == 4 && test.phases.back().kind == write_phase &&
              test.writes.empty(),
          "Global wait still exposes the held phase but cannot perform admitted I/O");
  const auto word = test.step({.index = 5, .requests = 1});
  require(word && word->value == first && test.reads.size() == 1 && test.writes.size() == 1 &&
              test.writes.front().address ==
                  DspDmaAddress{.space = DspDmaSpace::y, .address = transfer::destination} &&
              test.dma.read(transfer::status) == transfer::all_done,
          "The same held request needs no second edge and writes only the captured value");
}

void failure_recovery() {
  for (const auto failure : {Failure::unknown, Failure::standard, Failure::non_standard}) {
    Engine test;
    static_cast<void>(test.step({.index = 0}));
    transfer::configure(test.dma, transfer::word_copy);
    test.failure = failure;
    const auto source_error =
        rejects([&] { static_cast<void>(test.step({.index = 1, .requests = 1})); });
    require(source_error.contains("cycle=1 channel=0") && source_error.contains("0x000660") &&
                test.reads.empty() && test.writes.empty() &&
                test.dma.read(transfer::control0) == transfer::word_copy,
            "Admission failure reports the selected channel and commits no source I/O/state");
    test.failure = Failure::none;
    static_cast<void>(test.step({.index = 1, .requests = 1}));
    test.sample = second;
    test.failure = failure;
    const auto write_error =
        rejects([&] { static_cast<void>(test.step({.index = 2, .requests = 1})); });
    require(write_error.contains("cycle=2 channel=0") && test.reads.size() == 1 &&
                test.writes.empty() && test.phases.back().value == first,
            "Rejected held-write admission does not reread source or consume its cycle");
    test.failure = Failure::none;
    const auto word = test.step({.index = 2, .requests = 1});
    require(word && word->value == first && test.writes.size() == 1,
            "Retrying the uncommitted cycle retains the independent original held value");
  }
}

void priority_and_disable() {
  Engine test;
  static_cast<void>(test.step({.index = 0}));
  transfer::configure(test.dma, transfer::word_copy);
  constexpr unsigned high_irq_b = 0x8e0a44; // DE,DTM1,DRS1,priority3,fixedX->fixedY.
  test.dma.write(transfer::source0 - 4, transfer::ring_start + 1);
  test.dma.write(transfer::destination0 - 4, transfer::destination);
  test.dma.write(transfer::counter0 - 4, 0);
  test.dma.write(transfer::control0 - 4, high_irq_b);
  static_cast<void>(test.step({.index = 1, .requests = 1}));
  static_cast<void>(test.step({.index = 2, .requests = 3, .grant = DspDmaGrant::wait}));
  require(test.phases.back().channel == 0 && test.phases.back().kind == write_phase &&
              test.reads.size() == 1,
          "A new higher-priority edge cannot replace a lower-priority held word");
  const auto low = test.step({.index = 3, .requests = 3});
  require(low && low->channel == 0, "The original destination phase completes first");
  static_cast<void>(test.step({.index = 4, .requests = 3}));
  const auto high = test.step({.index = 5, .requests = 3});
  require(high && high->channel == 1 && test.reads.size() == 2 &&
              test.reads.back().address == transfer::ring_start + 1,
          "The sampled high-priority request survives waiting and owns the next whole word");

  Engine stopped;
  static_cast<void>(stopped.step({.index = 0}));
  transfer::configure(stopped.dma, transfer::word_copy);
  stopped.decision = DspDmaGrant::wait;
  static_cast<void>(stopped.step({.index = 1, .requests = 1}));
  stopped.dma.write(transfer::control0, transfer::word_copy & ~transfer::enable);
  stopped.decision = DspDmaGrant::available;
  static_cast<void>(stopped.step({.index = 2}));
  const auto last = stopped.step({.index = 3});
  require(last && last->value == first && stopped.reads.size() == 1 &&
              stopped.dma.read(transfer::status) == transfer::all_done,
          "APR23 word-mode DE clear retains an already captured but not-yet-admitted request");
}

void copied_admission() {
  DspDma dma;
  static_cast<void>(dma.advance(inputs({.index = 0}), {}));
  transfer::configure(dma, transfer::word_copy);
  unsigned calls = 0;
  unsigned stored = 0;
  auto held = std::make_shared<unsigned>(0);
  const std::weak_ptr<unsigned> weak = held;
  DspDmaIo io{.read = [](const DspDmaAddress) { return first; },
              .write = [&](const DspDmaAddress, const std::uint32_t value) { stored = value; }};
  io.admit = [owned = held, &io, &calls](const DspDmaPhase &phase) {
    ++*owned;
    ++calls;
    io.admit = {};
    require(phase.kind == read_phase && !phase.value,
            "A copied source admission has no fabricated source value");
    return DspDmaGrant::available;
  };
  held.reset();
  static_cast<void>(dma.advance(inputs({.index = 1, .requests = 1}), io));
  require(calls == 1 && weak.expired(),
          "The running callback survives clearing and is not retained");
  const auto word = dma.advance(inputs({.index = 2, .requests = 1}), io);
  require(word && stored == first && calls == 1,
          "A later held write retains no obsolete admission callback");
}
}

void phase_checks() {
  denied_and_global_wait();
  failure_recovery();
  priority_and_disable();
  copied_admission();
}
}
