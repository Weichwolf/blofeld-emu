// SPDX-License-Identifier: GPL-3.0-only
#include "io/shi.hpp"

#include <atomic>
#include <cstdint>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
constexpr std::uint32_t control = 0xffff91, clock_register = 0xffff90, receive = 0xffff94;
constexpr std::uint32_t receive_ready = 0x20000, receive_full = 0x80000, overrun = 0x100000;
constexpr std::uint32_t fifo24 = 0x29, single24 = 9, reset_status = 0x8200;

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
  require(!shi.receive(1) && !shi.read(receive, word),
          "Disabled or empty receive produces no data");
  require(!shi.write({.address = control, .value = 3}), "Unsupported I2C operation is explicit");
  require(shi.write({.address = control, .value = fifo24}),
          "Enable 24-bit SPI with a ten-word FIFO");
  require(shi.select(true).has_value(), "Select the active SPI receive port");
  for (unsigned index = 0; index < 10; ++index) {
    require(shi.receive(index), "Accept a real complete word");
  }
  require((status(shi) & receive_full) != 0, "The tenth word fills the FIFO");
  require(shi.receive(10), "An overflowing SPI transfer completes on the transport");
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
  require(shi.receive(1) && shi.receive(2), "Receive and overrun a one-word FIFO");
  require(shi.read(receive, word) && word == 1, "First read retains the original word");
  require((status(shi) & overrun) != 0, "HRX alone does not clear an unobserved HROE");
  require(shi.receive(3) && shi.read(receive, word) && word == 3, "Read after observed HROE");
  require((status(shi) & overrun) == 0, "Observed error clears only with the HRX read");
  require(shi.write({.address = control, .value = 1}), "Select eight-bit words");
  constexpr std::uint32_t byte = 0x53;
  constexpr std::uint32_t aligned_byte = 0x530000;
  require(shi.receive(byte) && shi.read(receive, word) && word == aligned_byte,
          "Eight-bit receive occupies the most significant byte");
  constexpr std::uint32_t reserved_clock_bits = 0xffffff;
  constexpr std::uint32_t implemented_clock_bits = 0x7ff;
  require(shi.write({.address = clock_register, .value = reserved_clock_bits}) &&
              shi.read(clock_register, word) && word == implemented_clock_bits,
          "Reserved HCKR bits read zero");
}

void concurrent_fifo() {
  blofeld::vi::Shi shi;
  require(shi.write({.address = control, .value = fifo24}), "Enable shared FIFO");
  std::atomic<bool> failed{false};
  std::jthread producer([&] {
    static_cast<void>(shi.select(true));
    for (unsigned index = 0; index < 10; ++index) {
      if (!shi.receive(index)) { failed.store(true, std::memory_order_release); }
    }
    static_cast<void>(shi.select(false));
  });
  for (unsigned index = 0; index < 10; ++index) {
    while ((status(shi) & receive_ready) == 0) {
      require(!failed.load(std::memory_order_acquire), "Producer accepts the bounded input");
      std::this_thread::yield();
    }
    std::uint32_t word = 0;
    require(shi.read(receive, word) && word == index, "Concurrent owners preserve word ordering");
  }
  producer.join();
  require(!failed.load(std::memory_order_acquire), "All concurrent input was accepted");
}
}

int main() try {
  fifo();
  error_clear();
  concurrent_fifo();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
