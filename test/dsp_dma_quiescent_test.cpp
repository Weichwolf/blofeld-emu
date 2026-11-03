// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "dsp/dsp_dma.hpp"
#include "dsp/dsp_dma_registers.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <string>

namespace {
// DSP56371 DMA register blocks and FM mode fields. DTM1 is one request per
// word, IRQA, X-to-Y with fixed addresses; DTM3 requests a software block.
constexpr unsigned source_register = 0xffffef;
constexpr unsigned destination_register = 0xffffee;
constexpr unsigned counter_register = 0xffffed;
constexpr unsigned control_register = 0xffffec;
constexpr unsigned status_register = 0xfffff4;
constexpr unsigned all_done = 0x3f;
constexpr unsigned word_copy = 0x880244;
constexpr unsigned software_copy = 0x9802d4;
constexpr unsigned source_address = 0x660;
constexpr unsigned destination_address = 0x900;
constexpr unsigned sample_value = 0x123456;
constexpr std::uint64_t distant_cycle = std::uint64_t{1} << 40;
std::uint64_t assertions = 0;

void require(const bool condition, const std::string &message) {
  ++assertions;
  if (!condition) { throw blofeld::Fault(message); }
}

void rejects(const auto &operation) {
  bool failed = false;
  try {
    operation();
  } catch (const blofeld::Fault &error) {
    failed = std::string(error.what()).contains("DSP DMA quiescent span at X:0xfffff4");
  }
  require(failed, "An invalid idle span must fail visibly in the actual controller");
}

blofeld::DspDmaInputs levels(const std::uint64_t cycle) {
  return {.cycle = cycle,
          .known_requests = std::numeric_limits<std::uint32_t>::max(),
          .timing = blofeld::DspDmaTiming::reference_sampled_cycles,
          .grant = blofeld::DspDmaGrant::available};
}

struct Configuration {
  unsigned channel;
  unsigned control;
};

struct Device {
  blofeld::DspDma dma;
  unsigned reads = 0, writes = 0;

  [[nodiscard]] std::optional<blofeld::DspDmaWord> tick(const blofeld::DspDmaInputs inputs) {
    return dma.advance(
        inputs,
        {.read =
             [this](const blofeld::DspDmaAddress address) {
               require(address.space == blofeld::DspDmaSpace::x &&
                           address.address == source_address,
                       "The actual source phase uses X:0x660");
               ++reads;
               return sample_value;
             },
         .write =
             [this](const blofeld::DspDmaAddress address, const unsigned value) {
               require(address.space == blofeld::DspDmaSpace::y &&
                           address.address == destination_address && value == sample_value,
                       "The actual destination phase writes the original 24-bit word");
               ++writes;
             }});
  }

  void configure(const Configuration configuration) {
    const auto [channel, control] = configuration;
    const auto offset = channel * 4;
    dma.write(source_register - offset, source_address);
    dma.write(destination_register - offset, destination_address);
    dma.write(counter_register - offset, 0);
    dma.write(control_register - offset, control);
  }

  void unchanged_rejection(const blofeld::DspDmaInputs inputs) {
    const auto cycle = dma.last_cycle();
    const auto status = dma.read(status_register);
    const auto state = dma.quiescent();
    rejects([&] { dma.advance_quiescent(inputs); });
    require(dma.last_cycle() == cycle && dma.read(status_register) == status &&
                dma.quiescent() == state && reads == 0 && writes == 0,
            "Rejected spans preserve cycle, channel state and external bus ownership");
  }
};

void test_reset_baseline_and_limit() {
  Device device;
  require(device.dma.quiescent(), "Reset has no active DMA transition");
  device.unchanged_rejection(levels(0));
  require(!device.tick(levels(0)), "The first real sample establishes its baseline");
  device.dma.advance_quiescent(levels(distant_cycle));
  require(device.dma.last_cycle() == distant_cycle &&
              device.dma.read(status_register) == all_done && device.reads == 0 &&
              device.writes == 0,
          "A large idle span only advances the genuine cycle cursor");
  auto bad = levels(distant_cycle + 1);
  bad.requests = 1;
  device.unchanged_rejection(bad);
  bad = levels(distant_cycle + 1);
  bad.known_requests = 0;
  device.unchanged_rejection(bad);
  bad = levels(distant_cycle + 1);
  bad.timing = blofeld::DspDmaTiming::unqualified;
  device.unchanged_rejection(bad);
  bad = levels(distant_cycle + 1);
  bad.grant = blofeld::DspDmaGrant::unknown;
  device.unchanged_rejection(bad);
  device.unchanged_rejection(levels(distant_cycle));
  device.unchanged_rejection(levels(distant_cycle - 1));
  constexpr auto last = std::numeric_limits<std::uint64_t>::max();
  device.dma.advance_quiescent(levels(last));
  require(device.dma.last_cycle() == last, "The last representable cycle is consumed exactly");
  device.unchanged_rejection(levels(0));
  device.unchanged_rejection(levels(last));
}

void test_waiting_channel_and_transfer(const unsigned channel) {
  Device device;
  require(!device.tick(levels(0)), "Establish known-low request levels before enabling");
  device.configure({.channel = channel, .control = word_copy | blofeld::dma::interrupt});
  require(!device.dma.quiescent(), "Enable settling and first sampling remain real transitions");
  device.unchanged_rejection(levels(distant_cycle));
  for (unsigned cycle = 1; cycle <= blofeld::dma::settling_cycles; ++cycle) {
    require(!device.tick(levels(cycle)), "Low request levels cannot transfer a word");
    if (cycle != blofeld::dma::settling_cycles) {
      device.unchanged_rejection(levels(distant_cycle));
    }
  }
  const auto bit = 1U << channel;
  require(device.dma.active() && device.dma.quiescent() &&
              device.dma.read(status_register) == (all_done & ~bit),
          "A settled enabled channel can wait quiescently with DTD still clear");
  device.dma.advance_quiescent(levels(distant_cycle));
  require(device.dma.read(status_register) == (all_done & ~bit) && device.reads == 0 &&
              device.writes == 0 && device.dma.interrupt_sources() == 0,
          "The idle span fabricates neither transfer, completion nor interrupt");
  auto request = levels(distant_cycle + 1);
  request.requests = 1;
  require(!device.tick(request) && device.reads == 1 && device.writes == 0 &&
              !device.dma.quiescent(),
          "The next real rising request starts only the actual source phase");
  const auto before = device.dma.last_cycle();
  rejects([&] { device.dma.advance_quiescent(levels(distant_cycle + 2)); });
  require(device.dma.last_cycle() == before && device.writes == 0,
          "An owned source word cannot disappear in an idle span");
  ++request.cycle;
  const auto word = device.tick(request);
  require(word && word->channel == channel && word->value == sample_value && device.writes == 1 &&
              device.dma.read(status_register) == all_done &&
              device.dma.interrupt_sources() == bit && device.dma.quiescent(),
          "The consecutive destination cycle commits the genuine word and interrupt");
  request.cycle += distant_cycle;
  device.dma.advance_quiescent(request);
  require(device.dma.interrupt_sources() == bit && device.reads == 1 && device.writes == 1,
          "A pending masked IRQ does not block a genuinely idle span or retrigger a word");
}

void test_pending_and_first_sample() {
  Device device;
  require(!device.tick(levels(0)), "Software-copy setup begins on an actual sampled boundary");
  device.configure({.channel = 0, .control = software_copy});
  require(!device.dma.quiescent(), "A pending software request must reach its actual phase");
  device.unchanged_rejection(levels(distant_cycle));
  auto blocked = levels(1);
  blocked.grant = blofeld::DspDmaGrant::wait;
  require(!device.tick(blocked), "Explicit wait keeps the pending source unowned");
  device.unchanged_rejection(levels(distant_cycle));
}
}

int main() {
  try {
    test_reset_baseline_and_limit();
    for (unsigned channel = 0; channel < 6; ++channel) {
      test_waiting_channel_and_transfer(channel);
    }
    test_pending_and_first_sample();
    std::cout << "DMA quiescent spans: " << assertions << " assertions passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
