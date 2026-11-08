// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dma.hpp"
#include "dsp/esai.hpp"

#include <array>
#include <cstddef>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Dma;
using blofeld::vi::dsp::Esai;
using dsp56300::Word;
constexpr Word tx0 = 0xffff80, tx1 = 0xffff81;
constexpr Word status_address = 0xffff91, skip_address = 0xffff92;
constexpr Word control_address = 0xffff95, clock_address = 0xffff96;
constexpr Word mask_low = 0xffff99, mask_high = 0xffff9a;
constexpr Word clock_config = 0x640301, tx_format = 0x7d00, active_format = 0x7d01;
constexpr Word empty = 0x8000, underrun = 0x4000, frame_sync = 0x2000;
constexpr Word even_empty = 0x10000, odd_empty = 0x20000;
constexpr Word first = 0x123456, second = 0xfedcba, mask_half = 0xffff;
constexpr unsigned sample_bits = 24;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void put(Esai &esai, const Esai::Write access) {
  require(esai.write(access), "Write an implemented ESAI register");
}

Word read(Esai &esai, const Word address) {
  Word value = 0;
  require(esai.read(address, value), "Read a defined ESAI register");
  return value;
}

Esai configured(const Word control = active_format) {
  Esai esai;
  put(esai, {.address = clock_address, .value = clock_config});
  put(esai, {.address = control_address, .value = control});
  esai.connected(true);
  return esai;
}

void reset_format_and_data() {
  Esai esai;
  Word value = 0;
  require(read(esai, clock_address) == 0 && read(esai, status_address) == 0 &&
              read(esai, mask_low) == mask_half && read(esai, mask_high) == mask_half &&
              !esai.read(tx0, value),
          "Reset clears status/control, enables all slots and keeps TX data write-only");
  put(esai, {.address = clock_address, .value = clock_config});
  put(esai, {.address = control_address, .value = tx_format});
  put(esai, {.address = tx0, .value = first});
  put(esai, {.address = control_address, .value = active_format});
  require(!esai.clock_slot() && !esai.take_request(), "Disconnected ports produce no clock event");
  esai.connected(true);
  const auto format = esai.format();
  require(format && format->word_bits == sample_bits && format->slots == 2 &&
              format->slot_ticks == 128,
          "The original configuration selects 24-bit words, two slots and HCKT/128 slots");
  const auto left = esai.clock_slot();
  require(left && left->index == 0 && left->enabled == 1 && left->known == 1 &&
              left->samples[0] == first && esai.take_request() && !esai.take_request(),
          "A real slot transfers the primed word and publishes one DMA request");
  require(read(esai, status_address) == (frame_sync | empty | even_empty),
          "First-slot sync and empty flags describe the actual transfer");
  put(esai, {.address = tx0, .value = second});
  require((read(esai, status_address) & empty) == 0, "Refill clears actual empty ownership");
  const auto right = esai.clock_slot();
  require(right && right->index == 1 && right->samples[0] == second && right->known == 1 &&
              read(esai, status_address) == (empty | odd_empty),
          "The second slot retains all signed 24-bit payload bits");
  const auto repeated = esai.clock_slot();
  require(repeated && repeated->known == 1 && repeated->samples[0] == second,
          "An underrun repeats actual retained data");
  put(esai, {.address = tx0, .value = first});
  require((read(esai, status_address) & underrun) != 0,
          "A data write alone cannot clear an unobserved underrun");
  put(esai, {.address = tx0, .value = second});
  require((read(esai, status_address) & underrun) == 0, "Observed underrun clears on real refill");
}

void unknown_mask_and_reset() {
  auto esai = configured();
  const auto unknown = esai.clock_slot();
  require(unknown && unknown->known == 0 && unknown->enabled == 1 &&
              (read(esai, status_address) & underrun) != 0,
          "Unwritten registers never become valid silent samples");
  put(esai, {.address = tx0, .value = first});
  static_cast<void>(esai.take_request());
  put(esai, {.address = mask_low, .value = 1});
  const auto old_frame = esai.clock_slot();
  require(old_frame && old_frame->enabled == 1, "A mask write cannot alter the current frame");
  put(esai, {.address = tx0, .value = second});
  const auto enabled = esai.clock_slot();
  const auto disabled = esai.clock_slot();
  require(enabled && enabled->index == 0 && enabled->known == 1 && disabled &&
              disabled->index == 1 && disabled->enabled == 0 && disabled->known == 0,
          "The next frame applies its slot mask without manufacturing disabled-slot audio");
  put(esai, {.address = skip_address});
  const auto skipped = esai.clock_slot();
  require(skipped && skipped->enabled == 0, "TSR disables the next available slot");
  esai.connected(false);
  require(read(esai, status_address) == 0 && read(esai, clock_address) == clock_config &&
              !esai.clock_slot() && !esai.take_request(),
          "Individual reset clears activity and requests while retaining configuration");
  esai.reset();
  require(read(esai, mask_low) == mask_half && !esai.format(),
          "Hardware reset restores mask/control");
}

void widths_and_multiple_transmitters() {
  struct Case {
    Word code, expected;
    unsigned ticks;
  };

  constexpr std::array cases{Case{.code = 1, .expected = 0x123000, .ticks = 48},
                             Case{.code = 2, .expected = 0x123400, .ticks = 64},
                             Case{.code = 30, .expected = first, .ticks = 96}};
  constexpr Word network_enable = 0x101;
  for (const auto item : cases) {
    auto esai = configured((item.code << 10U) | network_enable);
    put(esai, {.address = tx0, .value = first});
    const auto format = esai.format();
    const auto slot = esai.clock_slot();
    require(format && format->slot_ticks == item.ticks && slot && slot->known == 1 &&
                slot->samples[0] == item.expected,
            "Word length determines fractional precision and clock division independently");
  }
  auto esai = configured(tx_format | 3);
  put(esai, {.address = tx0, .value = first});
  put(esai, {.address = tx1, .value = second});
  const auto slot = esai.clock_slot();
  require(slot && slot->known == 3 && slot->samples[0] == first && slot->samples[1] == second,
          "Enabled transmitters retain their independent words");
  put(esai, {.address = tx0, .value = second});
  require((read(esai, status_address) & empty) != 0, "One refill cannot satisfy two enabled lanes");
  put(esai, {.address = tx1, .value = first});
  require((read(esai, status_address) & empty) == 0, "Every enabled lane must be refilled");
  constexpr Word reserved_length = 11;
  auto invalid = configured((reserved_length << 10U) | network_enable);
  require(!invalid.clock_slot() && invalid.error() != nullptr && !invalid.take_request(),
          "An unsupported format fails without fabricating a clock or DMA event");
}

void dma_stereo_words() {
  auto esai = configured();
  Dma dma;
  constexpr Word buffer_base = 0x660;
  constexpr Word negative_bit = 0x800000;
  std::array<Word, 64> buffer{};
  for (std::size_t index = 0; index < buffer.size(); ++index) {
    buffer[index] = (static_cast<Word>(index + 1) << 16U) | ((index & 1) != 0 ? negative_bit : 0);
  }
  constexpr std::array configuration{Dma::Write{.address = 0xfffff3, .value = 1},
                                     Dma::Write{.address = 0xfffff2, .value = 0xffffc1},
                                     Dma::Write{.address = 0xffffef, .value = buffer_base + 1},
                                     Dma::Write{.address = 0xffffee, .value = tx0},
                                     Dma::Write{.address = 0xffffed, .value = 0xfc0},
                                     Dma::Write{.address = 0xffffec, .value = 0xaab604}};
  for (const auto access : configuration) {
    require(dma.write(access), "Configure real DMA state");
  }
  put(esai, {.address = tx0, .value = buffer[0]});
  constexpr unsigned dma_request = 22;
  for (unsigned word = 0; word < 32; ++word) {
    const auto slot = esai.clock_slot();
    require(slot && slot->index == word % 2 && slot->known == 1 &&
                slot->samples[0] == buffer[word] && esai.take_request(),
            "Actual memory words arrive in complete stereo slot order");
    require(dma.trigger(dma_request) &&
                dma.service(
                    0,
                    [&](const Dma::Address source, Word &value) {
                      if (source.space != dsp56300::Space::x || source.address < buffer_base ||
                          source.address - buffer_base >= buffer.size()) {
                        return false;
                      }
                      value = buffer[source.address - buffer_base];
                      return true;
                    },
                    [&](const Dma::Address target, const Word value) {
                      return target.space == dsp56300::Space::y && target.address == tx0 &&
                             esai.write({.address = target.address, .value = value});
                    }),
            "The real empty event triggers DMA and refills the actual transmit register");
  }
}
}

int main() try {
  reset_format_and_data();
  unknown_mask_and_reset();
  widths_and_multiple_transmitters();
  dma_stereo_words();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
