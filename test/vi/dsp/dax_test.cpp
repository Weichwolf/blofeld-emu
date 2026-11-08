// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/dax.hpp"
#include "dsp/dma.hpp"

#include <array>
#include <dsp56300/memory.hpp>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace {
using blofeld::vi::dsp::Dax;
using blofeld::vi::dsp::Dma;
using dsp56300::Word;
constexpr Word control = 0xffffd0, non_audio = 0xffffd1, tx_a = 0xffffd2, tx_b = 0xffffd3;
constexpr Word status = 0xffffd4, metadata = 0x9400, first = 0x123456, second = 0xfedcba;
constexpr Word third = 0x234567, fourth = 0xabcdef, empty = 1, underrun = 2, block_end = 4;
constexpr Word all_control = 0x3f, start_block = 0x20, all_metadata = 0xfc00;
constexpr Word empty_vector = 0x2e, underrun_vector = 0x28, block_vector = 0x2a;
constexpr unsigned block_frames = 192;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void put(Dax &dax, const Dax::Write access) {
  require(dax.write(access), "Write a defined DAX register");
}

Word read(Dax &dax, const Word address) {
  Word value = 0;
  require(dax.read(address, value), "Read a readable DAX register");
  return value;
}

Dax::Frame frame(Dax &dax) {
  require(!dax.clock_subframe(), "Channel A cannot publish a partial frame");
  const auto result = dax.clock_subframe();
  if (!result) { throw std::runtime_error("Channel B must complete one actual frame"); }
  return *result;
}

void reset_and_registers() {
  Dax dax;
  Word value = 0;
  require(read(dax, control) == 0 && read(dax, status) == 0 && !dax.read(non_audio, value) &&
              !dax.read(tx_a, value) && !dax.read(tx_b, value),
          "Control/status reset to zero; data registers remain write-only");
  require(!dax.clock_subframe() && !dax.interrupt() && !dax.take_request(),
          "Disconnected clock events cannot manufacture data or requests");
  put(dax, {.address = control, .value = dsp56300::word_mask});
  require(read(dax, control) == all_control, "Reserved control bits always read zero");
  constexpr std::array<unsigned, 4> ticks{1024, 256, 384, 512};
  for (unsigned source = 0; source < ticks.size(); ++source) {
    put(dax, {.address = control, .value = source << 3U});
    const auto clock = dax.clock();
    require(clock.external == (source != 0) && clock.frame_ticks == ticks[source],
            "Each clock selection reports its documented source and frame divisor");
  }
  put(dax, {.address = non_audio, .value = dsp56300::word_mask});
  dax.reset();
  put(dax, {.address = tx_b, .value = first});
  put(dax, {.address = tx_b, .value = second});
  dax.connected(true);
  const auto output = frame(dax);
  require(
      output.samples == std::array{first, second} && output.known == 3 &&
          output.non_audio == all_metadata && !output.repeated,
      "Both XADR aliases alternate A/B, and reset retains written XNADR with reserved bits zero");
  dax.connected(false);
  require(read(dax, status) == 0 && !dax.clock_subframe(), "Personal reset clears activity");
}

void staging_and_metadata() {
  Dax dax;
  put(dax, {.address = non_audio, .value = metadata});
  put(dax, {.address = tx_a, .value = first});
  put(dax, {.address = tx_a, .value = second});
  put(dax, {.address = tx_a, .value = third});
  dax.connected(true);
  require(!dax.clock_subframe() && read(dax, status) == 0 && !dax.take_request(),
          "XADR can refill A as it uploads, so a primed next word is not empty");
  put(dax, {.address = non_audio, .value = 0});
  put(dax, {.address = tx_b, .value = fourth});
  const auto first_frame = dax.clock_subframe();
  require(first_frame && first_frame->samples == std::array{first, second} &&
              first_frame->non_audio == metadata,
          "A held B word cannot overwrite occupied B; metadata is captured at A upload");
  const auto next = frame(dax);
  require(next.samples == std::array{third, fourth} && next.non_audio == 0 && next.index == 1 &&
              dax.take_request() && !dax.take_request(),
          "The distinct XADR and channel buffers preserve both frames and real refill ownership");
  const auto repeated = frame(dax);
  require(repeated.repeated && repeated.known == 3 && repeated.samples == next.samples,
          "An underrun repeats the actual preceding complete pair");
  put(dax, {.address = tx_a, .value = first});
  put(dax, {.address = tx_b, .value = second});
  require((read(dax, status) & underrun) != 0, "Unobserved underrun survives a complete refill");
  put(dax, {.address = tx_a, .value = third});
  require((read(dax, status) & underrun) != 0, "One post-read write cannot clear an underrun");
  put(dax, {.address = tx_b, .value = fourth});
  require((read(dax, status) & underrun) == 0,
          "Status observation and two channel writes clear XAUR");
}

void blocks_and_interrupts() {
  Dax dax;
  dax.connected(true);
  put(dax, {.address = control, .value = empty | underrun});
  const auto unknown = frame(dax);
  require(unknown.repeated && unknown.known == 0 && !unknown.non_audio &&
              dax.interrupt() == underrun_vector,
          "Unprimed data is unknown and the enabled underrun vector has highest priority");
  static_cast<void>(read(dax, status));
  put(dax, {.address = tx_a, .value = first});
  put(dax, {.address = tx_a, .value = second});
  put(dax, {.address = control, .value = start_block | block_end | empty});
  for (unsigned index = 0; index < block_frames; ++index) {
    require(!dax.clock_subframe() && (read(dax, control) & start_block) == 0,
            "XSB clears at the actual next frame start");
    require(dax.interrupt() == (index + 1 == block_frames ? block_vector : empty_vector),
            "Only the final frame redirects an enabled empty interrupt to the block vector");
    const auto result = dax.clock_subframe();
    require(result && result->index == index && result->known == 3 && !result->repeated,
            "The block index advances once for each complete actual frame");
    put(dax, {.address = tx_a, .value = first});
    put(dax, {.address = tx_a, .value = second});
    require(!dax.interrupt() && read(dax, status) == 0, "Two channel writes clear empty and XBLK");
  }
  require(frame(dax).index == 0, "A complete 192-frame block wraps naturally");
  put(dax, {.address = control, .value = start_block});
  require(frame(dax).index == 0, "XSB starts a new block without changing data ownership");
}

void dma_refill() {
  Dax dax;
  Dma dma;
  constexpr Word source = 0x40;
  constexpr std::array configuration{Dma::Write{.address = 0xffffef, .value = source},
                                     Dma::Write{.address = 0xffffee, .value = tx_a},
                                     Dma::Write{.address = 0xffffed, .value = 1},
                                     Dma::Write{.address = 0xffffec, .value = 0x905250}};
  for (const auto access : configuration) { require(dma.write(access), "Configure DAX DMA line"); }
  put(dax, {.address = tx_a, .value = first});
  put(dax, {.address = tx_a, .value = second});
  dax.connected(true);
  constexpr unsigned request = 10;
  require(!dax.clock_subframe() && dax.take_request() && dma.trigger(request),
          "A real A upload requests the next two-channel DMA line");
  constexpr std::array words{third, fourth};
  require(dma.service(
              0,
              [&](const Dma::Address address, Word &value) {
                if (address.space != dsp56300::Space::x || address.address < source ||
                    address.address - source >= words.size()) {
                  return false;
                }
                value = words[address.address - source];
                return true;
              },
              [&](const Dma::Address address, const Word value) {
                return address.space == dsp56300::Space::x && address.address == tx_a &&
                       dax.write({.address = address.address, .value = value});
              }) &&
              !dma.pending(),
          "Both actual RAM words enter XADR through one real DMA request");
  const auto previous = dax.clock_subframe();
  require(previous && previous->samples == std::array{first, second} && frame(dax).samples == words,
          "DMA refills the next pair without corrupting the frame being transmitted");
}
}

int main() try {
  reset_and_registers();
  staging_and_metadata();
  blocks_and_interrupts();
  dma_refill();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
