// SPDX-License-Identifier: GPL-3.0-only
#include "io/shi.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
constexpr std::uint32_t control = 0xffff91, clock_register = 0xffff90, receive = 0xffff94;
constexpr std::uint32_t receive_ready = 0x20000, receive_full = 0x80000, overrun = 0x100000;
constexpr std::uint32_t fifo24 = 0x29, single24 = 9, reset_status = 0x8200;
constexpr std::uint32_t transmit = 0xffff93, transmit_empty = 0x8000, underrun = 0x4000;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

std::uint32_t status(blofeld::vi::Shi &shi) {
  std::uint32_t value = 0;
  require(shi.read(control, value), "Read the actual status register");
  return value;
}

void fifo() {
  blofeld::vi::Shi shi;
  std::uint32_t word = 0;
  require(status(shi) == reset_status, "Reset sets only HIDLE and HTDE");
  require(!shi.transfer(1).accepted && !shi.read(receive, word),
          "Disabled or empty receive produces no data");
  require(!shi.write({.address = control, .value = 3}), "Unsupported I2C operation is explicit");
  require(shi.write({.address = control, .value = fifo24}),
          "Enable 24-bit SPI with a ten-word FIFO");
  require(shi.select(true).has_value(), "Select the active SPI receive port");
  for (unsigned index = 0; index < 10; ++index) {
    require(shi.transfer(index).accepted, "Accept a real complete word");
  }
  require((status(shi) & receive_full) != 0, "The tenth word fills the FIFO");
  require(shi.transfer(10).accepted, "An overflowing SPI transfer completes on the transport");
  require((status(shi) & overrun) != 0, "The eleventh word sets HROE");
  for (unsigned index = 0; index < 10; ++index) {
    require(shi.read(receive, word) && word == index,
            "Overflow drops new data, preserves FIFO order");
  }
  require((status(shi) & (receive_ready | receive_full | overrun)) == 0,
          "HCSR then HRX clears the observed overrun and the FIFO drains");
  require(!shi.read(receive, word), "An empty FIFO cannot manufacture another word");
  static_cast<void>(shi.select(false));
  require(shi.write({.address = control, .value = 0}) && status(shi) == reset_status,
          "Disable resets FIFO and status");
}

void error_clear() {
  blofeld::vi::Shi shi;
  std::uint32_t word = 0;
  require(shi.write({.address = control, .value = single24}), "Enable single-word receive");
  static_cast<void>(shi.select(true));
  require(shi.transfer(1).accepted && shi.transfer(2).accepted,
          "Receive and overrun a one-word FIFO");
  require(shi.read(receive, word) && word == 1, "First read retains the original word");
  require((status(shi) & overrun) != 0, "HRX alone does not clear an unobserved HROE");
  require(shi.transfer(3).accepted && shi.read(receive, word) && word == 3,
          "Read after observed HROE");
  require((status(shi) & overrun) == 0, "Observed error clears only with the HRX read");
  require(shi.write({.address = control, .value = 1}), "Select eight-bit words");
  constexpr std::uint32_t byte = 0x53;
  constexpr std::uint32_t aligned_byte = 0x530000;
  require(shi.transfer(byte).accepted && shi.read(receive, word) && word == aligned_byte,
          "Eight-bit receive occupies the most significant byte");
  constexpr std::uint32_t reserved_clock_bits = 0xffffff;
  constexpr std::uint32_t implemented_clock_bits = 0x7ff;
  require(shi.write({.address = clock_register, .value = reserved_clock_bits}) &&
              shi.read(clock_register, word) && word == implemented_clock_bits,
          "Reserved HCKR bits read zero");
}

void peripheral_reset() {
  blofeld::vi::Shi shi;
  std::uint32_t value = 0;
  require(shi.read(clock_register, value) && value == 1, "CPHA resets set");
  require(shi.write({.address = control, .value = single24}) &&
              shi.write({.address = clock_register, .value = 3}),
          "Configure a non-default receive mode");
  static_cast<void>(shi.select(true));
  require(shi.transfer(1).accepted && shi.transfer(2).accepted && (status(shi) & overrun) != 0,
          "Populate the FIFO and its observed error before reset");
  shi.reset();
  require(status(shi) == reset_status && !shi.read(receive, value) &&
              shi.read(clock_register, value) && value == 1,
          "Reset restores registers, empties the FIFO and removes observed errors");
  require(shi.write({.address = control, .value = single24}) && shi.transfer(3).accepted &&
              shi.read(receive, value) && value == 3,
          "Reset preserves external selection but never preserves stale receive data");
}

void transmit_pipeline() {
  blofeld::vi::Shi shi;
  constexpr std::uint32_t first = 0xa591e7;
  constexpr std::uint32_t overwritten = 0x123456;
  constexpr std::uint32_t replacement = 0x654321;
  require(shi.write({.address = control, .value = fifo24}) &&
              shi.write({.address = transmit, .value = first}) &&
              (status(shi) & transmit_empty) != 0,
          "The first HTX write primes the empty shift register and frees HTX");
  require(shi.write({.address = transmit, .value = overwritten}) &&
              (status(shi) & transmit_empty) == 0 &&
              shi.write({.address = transmit, .value = replacement}),
          "An unpolled third write overwrites only the pending HTX word");
  static_cast<void>(shi.select(true));
  const auto one = shi.transfer(1);
  const auto two = shi.transfer(2);
  require(one.accepted && one.output == first && two.accepted && two.output == replacement,
          "Actual transfers deliver IOSR then the surviving HTX word in order");
  const auto empty = shi.transfer(3);
  require(empty.accepted && !empty.output, "Underrun never manufactures a transmitted byte");
  require(shi.write({.address = transmit, .value = first}) && (status(shi) & underrun) != 0,
          "HTX alone cannot clear an unobserved underrun");
  require(shi.write({.address = transmit, .value = replacement}) && (status(shi) & underrun) == 0,
          "Observed HCSR underrun clears on the following actual HTX write");
  shi.reset();
  require(status(shi) == reset_status && shi.write({.address = control, .value = single24}),
          "Reset restores status and allows the original mode to be re-enabled");
  const auto after_reset = shi.transfer(4);
  require(after_reset.accepted && !after_reset.output,
          "Reset discards both transmit stages rather than replaying stale output");
}

void transmit_widths() {
  struct Case {
    std::uint32_t mode, input, received, output;
  };

  constexpr std::array cases{
      Case{.mode = 1, .input = 0x53, .received = 0x530000, .output = 0xa5},
      Case{.mode = 5, .input = 0x5391, .received = 0x539100, .output = 0xa591},
      Case{.mode = 9, .input = 0x5391e7, .received = 0x5391e7, .output = 0xa591e7}};
  constexpr std::uint32_t sent = 0xa591e7;
  for (const auto &item : cases) {
    blofeld::vi::Shi shi;
    require(shi.write({.address = control, .value = item.mode}) &&
                shi.write({.address = transmit, .value = sent}),
            "Configure and prime a supported word size");
    static_cast<void>(shi.select(true));
    const auto transferred = shi.transfer(item.input);
    std::uint32_t received = 0;
    require(transferred.accepted && transferred.output == item.output &&
                shi.read(receive, received) && received == item.received,
            "Transmit uses the high byte(s); receive is high-aligned independently");
  }
}

void phase_zero_transmit() {
  blofeld::vi::Shi shi;
  constexpr std::uint32_t first = 0xa591e7;
  constexpr std::uint32_t second = 0x123456;
  require(shi.write({.address = clock_register, .value = 0}) &&
              shi.write({.address = control, .value = single24}),
          "Configure phase zero");
  static_cast<void>(shi.select(true));
  require(shi.write({.address = transmit, .value = first}) && (status(shi) & transmit_empty) == 0,
          "Phase zero cannot load IOSR while selected");
  static_cast<void>(shi.select(false));
  require((status(shi) & transmit_empty) != 0, "Deselection loads the pending shift word");
  static_cast<void>(shi.select(true));
  const auto transferred = shi.transfer(1);
  require(transferred.accepted && transferred.output == first && !shi.transfer(2).accepted,
          "Phase zero requires a selection boundary between complete words");
  require(shi.write({.address = transmit, .value = second}) && (status(shi) & transmit_empty) == 0,
          "The next phase-zero word stays in HTX until deselection");
  static_cast<void>(shi.select(false));
  static_cast<void>(shi.select(true));
  require(shi.transfer(3).output == second, "The next selection transmits the actual pending word");
}

void concurrent_fifo() {
  blofeld::vi::Shi shi;
  require(shi.write({.address = control, .value = fifo24}), "Enable shared FIFO");
  constexpr unsigned words = 64;
  std::atomic<bool> failed{false};
  std::jthread producer([&] {
    static_cast<void>(shi.select(true));
    for (unsigned index = 0; index < words; ++index) {
      if (!shi.submit(index, {}).accepted) { failed.store(true, std::memory_order_release); }
    }
    static_cast<void>(shi.select(false));
  });
  for (unsigned index = 0; index < words; ++index) {
    while ((status(shi) & receive_ready) == 0) {
      require(!failed.load(std::memory_order_acquire), "Producer accepts the bounded input");
      std::this_thread::yield();
    }
    std::uint32_t word = 0;
    require(shi.read(receive, word) && word == index, "Concurrent owners preserve word ordering");
  }
  producer.join();
  require(!failed.load(std::memory_order_acquire) && (status(shi) & overrun) == 0,
          "A longer VM stream applies real capacity backpressure without losing words");
}

void cancel_submission() {
  blofeld::vi::Shi shi;
  require(shi.write({.address = control, .value = fifo24}), "Configure bounded submission");
  static_cast<void>(shi.select(true));
  constexpr unsigned capacity = 10;
  for (unsigned index = 0; index < capacity; ++index) {
    require(shi.transfer(index).accepted, "Fill the receive FIFO before queuing more work");
  }
  constexpr std::uint32_t too_wide = 0x1000000;
  require(!shi.submit(too_wide, {}).accepted,
          "Invalid words fail immediately even when a full FIFO cannot make progress");
  std::atomic<bool> accepted{true};
  std::jthread producer([&](const std::stop_token &stop) {
    accepted.store(shi.submit(capacity, stop).accepted, std::memory_order_release);
  });
  producer.request_stop();
  producer.join();
  require(!accepted.load(std::memory_order_acquire) && (status(shi) & overrun) == 0,
          "Stop cancels a full-FIFO submission without publishing or overwriting data");
  std::jthread reset_producer(
      [&] { accepted.store(shi.submit(capacity, {}).accepted, std::memory_order_release); });
  shi.reset();
  reset_producer.join();
  require(!accepted.load(std::memory_order_acquire) && status(shi) == reset_status,
          "Receiver reset wakes a pending submission without committing stale data");
}

void concurrent_transmit() {
  blofeld::vi::Shi shi;
  require(shi.write({.address = control, .value = fifo24}), "Enable full-duplex word transfers");
  constexpr unsigned words = 64;
  constexpr std::uint32_t output_base = 0xa50000;
  std::atomic<unsigned> turn{0};
  std::atomic<bool> failed{false};
  std::jthread external([&] {
    static_cast<void>(shi.select(true));
    for (unsigned index = 0; index < words; ++index) {
      turn.wait(index * 2, std::memory_order_acquire);
      const auto transferred = shi.transfer(index);
      if (!transferred.accepted || transferred.output != output_base + index) {
        failed.store(true, std::memory_order_release);
      }
      turn.store((index * 2) + 2, std::memory_order_release);
      turn.notify_one();
    }
    static_cast<void>(shi.select(false));
  });
  for (unsigned index = 0; index < words; ++index) {
    if (!shi.write({.address = transmit, .value = output_base + index})) {
      failed.store(true, std::memory_order_release);
    }
    const auto waiting = (index * 2) + 1;
    turn.store(waiting, std::memory_order_release);
    turn.notify_one();
    turn.wait(waiting, std::memory_order_acquire);
    std::uint32_t received = 0;
    if (!shi.read(receive, received) || received != index) {
      failed.store(true, std::memory_order_release);
    }
  }
  external.join();
  require(!failed.load(std::memory_order_acquire),
          "Distinct execution owners retain every transmitted and received word");
}
}

int main() try {
  fifo();
  error_clear();
  peripheral_reset();
  transmit_pipeline();
  transmit_widths();
  phase_zero_transmit();
  concurrent_fifo();
  cancel_submission();
  concurrent_transmit();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
