// SPDX-License-Identifier: GPL-3.0-only
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/midi_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/execute.hpp"
#include "midi/file.hpp"
#include "storage/flash_image.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {
using blofeld::app::MidiFile;
using blofeld::vi::MidiPort;
using Bytes = std::vector<std::uint8_t>;
constexpr std::array<std::uint8_t, 4> ending{0, 0xff, 0x2f, 0};
constexpr unsigned byte_mask = 0xff;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

template <typename Error> void rejects(auto &&action, const char *message) {
  bool rejected = false;
  try {
    action();
  } catch (const Error &) { rejected = true; }
  require(rejected, message);
}

std::filesystem::path save(const std::filesystem::path &path,
                           const std::vector<Bytes> &tracks,
                           const std::uint8_t format = 0) {
  constexpr std::array<std::uint8_t, 14> header{
      'M', 'T', 'h', 'd', 0, 0, 0, 6, 0, 0, 0, 1, 0, 0x60};
  Bytes bytes(header.begin(), header.end());
  bytes[9] = format;
  bytes[11] = static_cast<std::uint8_t>(tracks.size());
  for (const auto &track : tracks) {
    constexpr std::array<std::uint8_t, 4> signature{'M', 'T', 'r', 'k'};
    bytes.insert(bytes.end(), signature.begin(), signature.end());
    for (const unsigned shift : {24U, 16U, 8U, 0U}) {
      bytes.push_back(static_cast<std::uint8_t>((track.size() >> shift) & byte_mask));
    }
    bytes.insert(bytes.end(), track.begin(), track.end());
  }
  std::ofstream output(path, std::ios::binary);
  output.write(reinterpret_cast<const char *>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  output.close();
  require(output.good(), "Write a complete authored SMF");
  return path;
}

std::unique_ptr<blofeld::vi::FlashImage> echo_image() {
  namespace isa = cpu12::isa;
  constexpr auto program = isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                                        std::array<cpu12::Byte, 2>{0x17, 0},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array<cpu12::Byte, 1>{12},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xc9},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array<cpu12::Byte, 1>{0x2c},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcb},
                                        isa::opcode<isa::Opcode::andcc_immediate>(),
                                        std::array<cpu12::Byte, 1>{0xef},
                                        isa::opcode<isa::Opcode::wai>(),
                                        isa::opcode<isa::Opcode::bra_relative>(),
                                        std::array<cpu12::Byte, 1>{0xfd});
  constexpr auto handler = isa::program(isa::opcode<isa::Opcode::ldaa_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcc},
                                        isa::opcode<isa::Opcode::ldaa_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcf},
                                        isa::opcode<isa::Opcode::staa_direct>(),
                                        std::array<cpu12::Byte, 1>{0xcf},
                                        isa::opcode<isa::Opcode::rti>());
  auto image = std::make_unique<blofeld::vi::FlashImage>();
  image->known.fill(1);
  std::ranges::copy(program, image->bytes.begin());
  constexpr unsigned handler_offset = 0x100;
  constexpr unsigned irq_vector = 0x7fd6;
  std::ranges::copy(handler, image->bytes.begin() + handler_offset);
  constexpr std::uint8_t handler_segment = 0x81;
  image->bytes[irq_vector] = handler_segment;
  image->bytes[irq_vector + 1] = 0;
  image->bytes[image->bytes.size() - 2] = 0x80;
  image->bytes.back() = 0;
  return image;
}

void native_echo(const std::filesystem::path &root) {
  const Bytes track{0, 0x90, 60, 100, 0x60, 0x80, 60, 64, 0, 0xff, 0x2f, 0};
  MidiFile source(save(root / "echo.mid", {track}));
  MidiPort port(MidiPort::InputMode::stream);
  auto firmware = echo_image();
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::ExecutionReport report;
  std::atomic<bool> done{false};
  std::jthread mcu([&](const std::stop_token &stop) {
    try {
      report = blofeld::vi::mcu::execute(
          *firmware, shi, reset, stop, nullptr, nullptr, nullptr, nullptr, &port);
    } catch (...) { report.exception = std::current_exception(); }
    done.store(true, std::memory_order_release);
  });
  constexpr std::uint64_t origin = MidiPort::byte_ticks;
  std::optional<std::uint64_t> end;
  std::exception_ptr failure;
  std::jthread producer([&](const std::stop_token &stop) {
    try {
      end = source.feed(port, origin, stop);
      require(end && port.input_limit() == end,
              "Validated EOF retains its barrier until the application publishes completion");
      require(port.finish_input(),
              "The caller releases input after recording the completion limit");
    } catch (...) { failure = std::current_exception(); }
  });
  constexpr std::array<std::uint8_t, 6> expected{0x90, 60, 100, 0x80, 60, 64};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  std::uint64_t previous = 0;
  for (const auto byte : expected) {
    std::optional<MidiPort::Event> output;
    while (!(output = port.read_output())) {
      if (done.load(std::memory_order_acquire)) {
        output = port.read_output();
        break;
      }
      require(std::chrono::steady_clock::now() < deadline,
              "Native SCI replies must reach their independent consumer");
      std::this_thread::yield();
    }
    require(output && output->byte == byte && output->ticks > previous,
            "SMF bytes pass through real receive interrupts and transmitted-character completions");
    previous = output ? output->ticks : 0;
  }
  producer.join();
  mcu.join();
  if (failure) { std::rethrow_exception(failure); }
  if (report.exception) { std::rethrow_exception(report.exception); }
  constexpr std::uint64_t half_second = MidiPort::reference_hz / 2;
  require(!report.failure && !port.read_output() && port.error() == nullptr &&
              end == origin + half_second + (3 * MidiPort::byte_ticks),
          "The source reports actual wire completion when serialization extends past EOT");
}

void cancelled_source(const std::filesystem::path &root) {
  constexpr std::array<std::uint8_t, 4> prefix{0, 0xf0, 0x90, 0};
  Bytes track(prefix.begin(), prefix.end());
  constexpr std::size_t payload = 2048;
  track.resize(prefix.size() + payload, 1);
  constexpr std::uint8_t terminator = 0xf7;
  track.back() = terminator;
  track.insert(track.end(), ending.begin(), ending.end());
  MidiFile source(save(root / "cancel.mid", {track}));
  MidiPort port(MidiPort::InputMode::stream);
  std::optional<std::uint64_t> end;
  std::exception_ptr failure;
  std::jthread producer([&](const std::stop_token &stop) {
    try {
      end = source.feed(port, 0, stop);
    } catch (...) { failure = std::current_exception(); }
  });
  constexpr auto frontier = MidiPort::capacity * MidiPort::byte_ticks;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (port.input_limit() != frontier) {
    require(std::chrono::steady_clock::now() < deadline,
            "A file producer reaches bounded backpressure");
    std::this_thread::yield();
  }
  producer.request_stop();
  producer.join();
  if (failure) { std::rethrow_exception(failure); }
  require(!end && port.input_limit() == frontier && port.time() == 0 && port.error() == nullptr,
          "Cancellation does not become EOF or manufacture more time");
  rejects<std::logic_error>([&] { static_cast<void>(source.feed(port, 0, {})); },
                            "A cancelled file cannot resume after consuming a rejected chunk");
}

void failures(const std::filesystem::path &root) {
  const std::vector<Bytes> cases{{0, 0x90, 60, 1, 0, 0xff, 1, 0x7f, 1},
                                 {0, 0xf7, 2, 0xf0, 1, 0, 0xff, 0x2f, 0},
                                 {0x60, 0x90, 60, 1, 0, 0xff, 0x2f, 0}};
  for (std::size_t index = 0; index < cases.size(); ++index) {
    MidiFile source(save(root / ("bad-" + std::to_string(index) + ".mid"), {cases[index]}));
    MidiPort port(MidiPort::InputMode::stream);
    const auto origin = index == 2 ? std::numeric_limits<std::uint64_t>::max() : 0;
    rejects<std::runtime_error>(
        [&] { static_cast<void>(source.feed(port, origin, {})); },
        "Bad source framing, unfinished destination SysEx and origin overflow fail");
    require(port.error() != nullptr && !port.await_input(0, {}) && !port.await_output({}),
            "Source failure wakes both native execution and reply consumers");
  }
  const Bytes first{0, 0xf0, 1, 1, 0x60, 0xf7, 1, 0xf7, 0, 0xff, 0x2f, 0};
  const Bytes second{0x30, 0x90, 60, 1, 0, 0xff, 0x2f, 0};
  MidiFile collision(save(root / "collision.mid", {first, second}, 1));
  MidiPort port(MidiPort::InputMode::stream);
  rejects<std::runtime_error>(
      [&] { static_cast<void>(collision.feed(port, 0, {})); },
      "Merging tracks must not splice a channel message into another track's SysEx");
  const Bytes empty(ending.begin(), ending.end());
  MidiFile live(save(root / "live.mid", {empty}));
  MidiPort input;
  rejects<std::logic_error>([&] { static_cast<void>(live.feed(input, 0, {})); },
                            "A file cannot accidentally use an unbounded live input");
  require(input.error() == nullptr, "Invalid composition does not alter a live endpoint");
}
}

int main(const int argc, char **argv) try {
  require(argc == 2, "Supply a build artifact directory");
  const auto root = std::filesystem::path(argv[1]) / std::to_string(::getpid());
  require(std::filesystem::create_directories(root), "Create owned unit directory");
  native_echo(root);
  cancelled_source(root);
  failures(root);
  std::filesystem::remove_all(root);
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
