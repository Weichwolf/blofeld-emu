// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/channels.hpp"
#include "dsp/io.hpp"
#include "dsp/ports.hpp"
#include "io/coupling.hpp"
#include "io/peer_port.hpp"
#include "storage/media_image.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <limits>
#include <optional>

namespace blofeld::vi::dsp {

std::optional<Io::MediaResult> Io::copy_media_run(MediaRun run) noexcept {
  using dsp56300::Space;
  using dsp56300::Word;
  if (run.words == 0 || run.words > MediaImage::page_bytes / 2 || !memory_.direct()) { return {}; }
  const auto now = device_cycles();
  run.words = media_run_words(run, now);
  if (run.words == 0 || !media_ports_are_quiet()) { return {}; }
  const auto destination = media_run_target(run);
  if (!destination) { return {}; }

  constexpr Word data_address = 0xfffff5;
  const auto initial = controller_.output();
  Word f = 0;
  Word direction = 0;
  Word function = 0;
  constexpr Word byte = 0xff;
  if (!initial || !ports_.read(Space::y, data_address, f) || (f & byte) != *initial ||
      !ports_.read(Space::y, data_address + 1, direction) ||
      !ports_.read(Space::y, data_address + 2, function) ||
      (function & ~direction & byte) != byte) {
    return {};
  }

  auto projected = clock_;
  const auto last_strobe = now + (std::uint64_t{run.words} * run.turn_cycles) - run.strobe_tail;
  if (peer_ != nullptr && (!projected.advance(last_strobe) || !event_time(projected))) {
    return {};
  }

  // The complete destination, event horizon and old latch are established.
  // A rejected media range changes neither its latch nor the destination.
  const auto consumed = controller_.read_run(run.words * 2);
  if (consumed.empty()) { return {}; }
  const auto output = destination->words;
  output.front() = (Word{*initial} << 16U) | (Word{consumed.front()} << 8U);
  for (std::size_t index = 1; index < output.size(); ++index) {
    const auto offset = index * 4;
    output[index] = (Word{consumed[offset - 2]} << 16U) | (Word{consumed[offset]} << 8U);
  }
  std::ranges::fill(destination->known, 1);
  ports_.input(Port::f, {.mask = byte, .known = byte, .value = consumed[consumed.size() - 2]});
  if (peer_ != nullptr) { clock_ = projected; }
  return MediaResult{.word = output.back(), .words = run.words};
}

std::optional<Io::MediaTarget> Io::media_run_target(const MediaRun run) noexcept {
  for (const auto &page : memory_.windows(dsp56300::Space::y)) {
    if (!page.writable || page.aliases_program || run.destination < page.address) { continue; }
    const auto offset = run.destination - page.address;
    if (offset >= page.words.size() || run.words > page.words.size() - offset) { continue; }
    return MediaTarget{.words = page.words.subspan(offset, run.words),
                       .known =
                           page.known.empty() ? page.known : page.known.subspan(offset, run.words)};
  }
  return {};
}

std::uint32_t Io::media_run_words(const MediaRun run, const std::uint64_t now) const noexcept {
  if (run.strobe_tail >= run.turn_cycles || stop_.stop_requested() || reset_.pending(epoch_) ||
      peer_interrupt() || (peer_ == nullptr) != (coupling_ == nullptr)) {
    return 0;
  }
  const auto cycles = std::uint64_t{run.words} * run.turn_cycles;
  if (cycles > std::numeric_limits<std::uint64_t>::max() - now) { return 0; }
  auto limit = deadline();
  if (peer_ != nullptr && (!limit || peer_read_until_ < *limit)) { limit = peer_read_until_; }
  if (!limit || now + cycles < *limit) { return run.words; }
  if (*limit <= now) { return 0; }

  constexpr std::uint32_t minimum_short_run = 8;
  const auto available = (*limit - now - 1U) / run.turn_cycles;
  // The grant above excludes the complete request. Available therefore fits in words.
  return available >= minimum_short_run ? static_cast<std::uint32_t>(available) : 0;
}

bool Io::media_ports_are_quiet() const noexcept {
  if (esai_1_.connected() != (ports_.peripheral(Port::e) != 0) ||
      dax_.connected() != (ports_.peripheral(Port::d) != 0)) {
    return false;
  }
  constexpr dsp56300::Word port_bits = 0xfff;
  constexpr dsp56300::Word control_mask = 0xf00;
  constexpr dsp56300::Word inactive_strobes = 0x300;
  dsp56300::Word previous = 0;
  const auto c = ports_.output(Port::c);
  if ((c.mask & control_mask) != control_mask || c.known != port_bits ||
      (c.value & inactive_strobes) != inactive_strobes ||
      !ports_.read(dsp56300::Space::x, peer_data_address, previous) || previous != c.value) {
    return false;
  }
  if (peer_ == nullptr) { return true; }
  constexpr dsp56300::Word byte = 0xff;
  constexpr dsp56300::Word ready = 1U << 9U;
  const auto e = ports_.output(Port::e);
  const bool known = (e.mask & ~e.known & ready) == 0;
  const bool active = (e.mask & ready) != 0 && (e.value & ready) == 0;
  return known &&
         peer_->unchanged_dsp_writes({.mask = static_cast<std::uint8_t>(c.mask & byte),
                                      .known = static_cast<std::uint8_t>(c.known & byte),
                                      .value = static_cast<std::uint8_t>(c.value & byte)},
                                     active) &&
         coupling_->outputs_unchanged();
}

// Private ABI returns the admitted count and final word. A zero result has no effects.
extern "C" std::uint64_t blofeld_native_media_transfer(dsp56300::Io *owner,
                                                       const dsp56300::PairedPortTransfer *transfer,
                                                       std::uint64_t request) noexcept {
  using dsp56300::Space;
  using dsp56300::Word;
  constexpr Word media_data = 0xfffff5;
  constexpr Word media_control = 0xffffbd;
  constexpr Word read_strobe = 8;
  constexpr Word loop_cycles = 20;
  constexpr std::array<Word, 2> reads{0, 9};
  constexpr std::array<Word, 4> edges{1, 5, 10, 16};
  if (transfer->data_space != static_cast<Word>(Space::y) || transfer->data_address != media_data ||
      transfer->strobe_space != static_cast<Word>(Space::x) ||
      transfer->strobe_address != media_control || transfer->strobe_bit != read_strobe ||
      transfer->cycles != loop_cycles || transfer->read_cycles != reads ||
      transfer->edge_cycles != edges) {
    return 0;
  }
  constexpr unsigned count_shift = 32;
  const auto address = static_cast<Word>(request) & dsp56300::word_mask;
  const auto turns = static_cast<Word>(request >> count_shift);
  const auto result =
      static_cast<Io *>(owner)->copy_media_run({.destination = address,
                                                .words = turns,
                                                .turn_cycles = loop_cycles,
                                                .strobe_tail = loop_cycles - edges.back()});
  return result ? (std::uint64_t{result->words} << count_shift) | result->word : 0;
}
}
