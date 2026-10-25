// SPDX-License-Identifier: GPL-3.0-only
#include "common/bus.hpp"
#include "firmware/firmware.hpp"
#include "original_gpio_midi_fixture.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>
#include <string_view>

namespace blofeld::test::gpio_midi {
namespace {
constexpr unsigned channels = 16;
constexpr unsigned byte_maximum = 255;
constexpr unsigned frame_source_bytes = 128;
constexpr unsigned usb_source_bytes = 160;
constexpr unsigned expected_messages = 48;
constexpr unsigned pitch_status = 0xe0;
constexpr unsigned clock_byte = 0xf8;
constexpr unsigned center = 8192;
constexpr unsigned highest = 16383;
constexpr unsigned scale = 1024;
constexpr unsigned data_mask = 127;
constexpr unsigned word_mask = 0xffffff;
constexpr unsigned pitch_header = 0x0e;
constexpr unsigned clock_header = 0x0f;
constexpr unsigned unused_padding = 0xa5;
constexpr unsigned max_usb_count = 124;
constexpr unsigned channel_selector = 5;

void append(Source &source, const unsigned value) {
  require(value <= byte_maximum, "Own valid MIDI values fit a byte");
  source.bytes.push_back(static_cast<std::uint8_t>(value));
}

void packet(Source &source, const std::array<unsigned, 4> bytes) {
  for (const auto value : bytes) { source.storage.push_back(static_cast<std::uint8_t>(value)); }
}
}

Source make_source(const Scenario scenario) {
  Source source;
  const auto padding = scenario.transport == Transport::endpoint ? 0U : unused_padding;
  for (unsigned channel = 0; channel < channels; ++channel) {
    if (scenario.transport == Transport::frame) { append(source, pitch_status | channel); }
    for (const auto value : {0U, center, highest}) {
      const auto lsb = value & data_mask;
      const auto msb = value >> 7U;
      if (scenario.transport != Transport::frame) {
        append(source, pitch_status | channel);
        packet(source, {pitch_header, pitch_status | channel, lsb, msb});
      }
      append(source, lsb);
      append(source, msb);
      if (scenario.channel == 0 || scenario.channel == channel + 1) {
        const auto centered = static_cast<std::int32_t>(value) - static_cast<std::int32_t>(center);
        source.pitches.push_back(
            static_cast<unsigned>(centered * static_cast<std::int32_t>(scale)) & word_mask);
      }
    }
    append(source, clock_byte);
    if (scenario.transport != Transport::frame) {
      packet(source, {clock_header, clock_byte, padding, padding});
    }
  }
  if (scenario.transport == Transport::frame) {
    source.storage = source.bytes;
    source.counts.push_back(static_cast<std::uint8_t>(source.bytes.size()));
  } else {
    for (auto remaining = static_cast<unsigned>(source.storage.size()); remaining > 0;) {
      const auto count = remaining > max_usb_count ? max_usb_count : remaining;
      source.counts.push_back(static_cast<std::uint8_t>(count));
      remaining -= count;
    }
  }
  require(source.bytes.size() == (scenario.transport == Transport::frame ? frame_source_bytes
                                                                         : usb_source_bytes) &&
              source.pitches.size() == (scenario.channel == 0 ? expected_messages : 3U),
          "The independent source matrix has exact valid-message/running-status/clock coverage");
  return source;
}

namespace {
void run_case(const Firmware &firmware, const Scenario scenario, std::ostream &trace) {
  Fixture fixture(firmware, scenario);
  const bool checkpoint = !scenario.inverted && scenario.ratio == 4 && scenario.channel == 0;
  if (checkpoint) {
    while (!fixture.checkpointable()) {
      require(!fixture.complete(), "Native in-flight IRQ/partial-message snapshot must be reached");
      fixture.step();
    }
    const auto saved = fixture.save();
    fixture.run();
    const auto expected = fixture.result();
    fixture.restore(saved);
    fixture.run();
    require(fixture.result() == expected,
            "Both native owner snapshots replay all watched RAM, whole CPU/debug state and "
            "bookkeeping exactly");
  } else {
    fixture.run();
  }
  const auto &stats = fixture.stats();
  const auto result = fixture.result();
  trace << "CASE transport=" << static_cast<unsigned>(scenario.transport)
        << " inverted=" << scenario.inverted << " ratio=" << scenario.ratio
        << " channel=" << scenario.channel << " bytes=" << stats.received
        << " consumed=" << stats.consumed << " irqs=" << stats.irqs << " rtis=" << stats.returns
        << " clocks=" << stats.timestamps << " callbacks=" << stats.callbacks
        << " pitches=" << stats.pitches << " producer-steps=" << stats.producer_steps
        << " mcu-steps=" << result.peer.steps << " transitions=" << result.peer.transitions
        << " preemptions=" << stats.preemptions << " replay=" << checkpoint;
  if (scenario.transport == Transport::endpoint) {
    trace << " usb-packets=" << result.peer.calls << " usb-irqs=" << result.peer.usb[3]
          << " usb-naks=" << result.peer.usb[4] << " mcu-frame-reuses=" << result.peer.usb[5];
  }
  trace << '\n';
}

void endpoint_profiles(const Firmware &firmware, std::ostream &trace) {
  for (const auto inverted : {false, true}) {
    for (const auto ratio : {1U, 2U, 4U}) {
      for (const auto channel : {0U, channel_selector}) {
        run_case(firmware,
                 {.transport = Transport::endpoint,
                  .inverted = inverted,
                  .ratio = ratio,
                  .channel = channel},
                 trace);
      }
    }
  }
  trace << "RESULT endpoint-profiles=12 joint-replays=1\n";
}

void slow_profile_rejects(const Firmware &firmware, std::ostream &trace) {
  try {
    Fixture fixture(firmware,
                    {.transport = Transport::frame, .inverted = false, .ratio = 1, .channel = 0});
    fixture.run();
  } catch (const Fault &error) {
    require(
        std::string(error.what()).contains("Actual GPIO ring store must equal"),
        "The deliberate slow-profile control must reject exact data loss, not an unrelated fault");
    trace << "REJECT slow-frame-profile=1 exact-data-mismatch=1\n";
    return;
  }
  throw Fault("Unqualified one-to-one frame profile must not silently claim lossless delivery");
}
}
}

int main(const int count, char **arguments) {
  using namespace blofeld::test::gpio_midi;
  try {
    const bool endpoint = count == 4 && std::string_view(arguments[3]) == "endpoint";
    require(count == 3 || endpoint,
            "Usage: original-gpio-midi firmware.zip private.trace [endpoint]");
    const auto firmware = blofeld::Firmware::load(std::filesystem::path(arguments[1]));
    std::ofstream trace(arguments[2]);
    require(trace.is_open(), "Cannot create private GPIO/MIDI integration trace");
    trace.exceptions(std::ios::failbit | std::ios::badbit);
    if (endpoint) {
      endpoint_profiles(firmware, trace);
      return 0;
    }
    for (const auto transport : {Transport::frame, Transport::usb}) {
      for (const auto inverted : {false, true}) {
        for (const auto ratio : {1U, 2U, 4U}) {
          if (transport == Transport::frame && ratio == 1) { continue; }
          for (const auto channel : {0U, channel_selector}) {
            run_case(
                firmware,
                {.transport = transport, .inverted = inverted, .ratio = ratio, .channel = channel},
                trace);
          }
        }
      }
    }
    slow_profile_rejects(firmware, trace);
    trace << "RESULT profiles=20 replays=2 slow-rejections=1\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
