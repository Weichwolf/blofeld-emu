// SPDX-License-Identifier: GPL-3.0-only
#include "io/dsp_reset.hpp"

#include <atomic>
#include <exception>
#include <iostream>
#include <latch>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void epochs() {
  blofeld::vi::DspReset reset;
  require(reset.state().epoch == 0 && !reset.state().held && !reset.pending(0),
          "Cold reset starts released at epoch zero");
  require(reset.assert_reset() && reset.assert_reset(), "Assertion is idempotent while held");
  require(reset.state().epoch == 1 && reset.state().held && reset.pending(0),
          "A held assertion advances the epoch exactly once");
  require(!reset.acknowledge(2) && reset.acknowledge(1), "Only requested epochs can be applied");
  require(reset.release({}) && !reset.state().held && reset.state().epoch == 1,
          "Release preserves the acknowledged epoch");
  require(reset.pending(0) && !reset.pending(1), "An unobserved complete reset cannot disappear");
  require(!reset.acknowledge(0), "Acknowledgement cannot move backwards");
  require(reset.assert_reset() && reset.state().epoch == 2 && reset.acknowledge(2) &&
              reset.release({}),
          "The next reset has independent ownership and release");
}

void release_handshake() {
  blofeld::vi::DspReset reset;
  require(reset.assert_reset(), "Request a real reset");
  std::latch entered(1);
  std::atomic<bool> released{false};
  std::jthread mcu([&](const std::stop_token &stop) {
    entered.count_down();
    released.store(reset.release(stop), std::memory_order_release);
  });
  entered.wait();
  require(reset.state().held && !released.load(std::memory_order_acquire),
          "An unapplied reset cannot be released");
  require(reset.acknowledge(1), "DSP owner retires the old execution and applies reset");
  mcu.join();
  require(released.load(std::memory_order_acquire) && !reset.state().held,
          "Acknowledgement permits the waiting MCU release");
}

void cancellation() {
  blofeld::vi::DspReset reset;
  require(reset.assert_reset(), "Hold reset for cancellation");
  std::latch entered(2);
  std::atomic<bool> released{true};
  std::atomic<bool> resumed{true};
  std::jthread mcu([&](const std::stop_token &stop) {
    entered.count_down();
    released.store(reset.release(stop), std::memory_order_release);
  });
  std::jthread dsp([&](const std::stop_token &stop) {
    entered.count_down();
    resumed.store(reset.wait_release(stop).has_value(), std::memory_order_release);
  });
  entered.wait();
  mcu.request_stop();
  dsp.request_stop();
  mcu.join();
  dsp.join();
  require(!released.load(std::memory_order_acquire) && !resumed.load(std::memory_order_acquire) &&
              reset.state().held,
          "Shutdown cancels both waits without inventing a release");
}
}

int main() try {
  epochs();
  release_handshake();
  cancellation();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
