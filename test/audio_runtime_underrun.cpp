// SPDX-License-Identifier: GPL-3.0-only
#include "audio/audio_runtime.hpp"
#include "audio_runtime_fixture.hpp"
#include "common/bus.hpp"
#include "serial_pcm_fixture.hpp"

#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <sys/poll.h>
#include <sys/syscall.h>
#include <system_error>
#include <thread>
#include <unistd.h>

namespace blofeld::test::audio_runtime {
namespace {
using ThreadId = decltype(::getpid());
// Set before creating the worker, never changed while it exists. The signal
// handler uses only POSIX async-signal-safe I/O, preserving interrupted errno.
int notice_output = -1;
int release_input = -1;
constexpr int signal_failure = 90;

void park_worker(int /*signal*/) noexcept {
  const auto saved = errno;
  char token = 'r';
  if (::write(notice_output, &token, 1) != 1) { ::_exit(signal_failure); }
  auto count = ::read(release_input, &token, 1);
  while (count < 0 && errno == EINTR) { count = ::read(release_input, &token, 1); }
  if (count != 1) { ::_exit(signal_failure); }
  errno = saved;
}

class Pipe {
public:
  Pipe() {
    if (::pipe(descriptors_.data()) != 0) {
      throw Fault("Cannot create an owned test parking pipe");
    }
  }

  ~Pipe() {
    (void)::close(descriptors_[0]);
    (void)::close(descriptors_[1]);
  }

  Pipe(const Pipe &) = delete;
  Pipe &operator=(const Pipe &) = delete;

  [[nodiscard]] int input() const { return descriptors_[0]; }

  [[nodiscard]] int output() const { return descriptors_[1]; }

private:
  std::array<int, 2> descriptors_{};
};

class Parker {
public:
  Parker() {
    notice_output = notice_.output();
    release_input = release_.input();
    previous_ = std::signal(SIGINT, park_worker);
    if (previous_ == SIG_ERR) { throw Fault("Cannot install the owned test parking signal"); }
  }

  ~Parker() {
    if (std::signal(SIGINT, previous_) == SIG_ERR) { std::terminate(); }
  }

  Parker(const Parker &) = delete;
  Parker &operator=(const Parker &) = delete;

  void release() const noexcept {
    const char token = 'g';
    if (::write(release_.output(), &token, 1) != 1) { std::terminate(); }
  }

  void park(const ThreadId thread) const {
    pcm::require(::syscall(SYS_tgkill, ::getpid(), thread, SIGINT) == 0,
                 "The test can park only its own live audio worker");
    pollfd ready{.fd = notice_.input(), .events = POLLIN, .revents = 0};
    constexpr int timeout_ms = 2000;
    pcm::require(::poll(&ready, 1, timeout_ms) == 1 && (ready.revents & POLLIN) != 0,
                 "The actual worker must acknowledge entering the parking handler");
    char token = 0;
    pcm::require(::read(notice_.input(), &token, 1) == 1 && token == 'r',
                 "The worker parking acknowledgement is complete");
  }

private:
  Pipe notice_;
  Pipe release_;
  using Handler = void (*)(int);
  Handler previous_ = nullptr;
};

struct Release {
  ~Release() { parker.release(); }

  const Parker &parker;
};

std::string read_line(const std::filesystem::path &path) {
  std::ifstream input(path);
  std::string value;
  std::getline(input, value);
  return value;
}

ThreadId sleeping_worker(AudioRuntime &output) {
  const std::filesystem::path tasks("/proc/self/task");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
  for (;;) {
    output.check();
    pcm::require(std::chrono::steady_clock::now() < deadline,
                 "Audio worker must reach its idle wait");
    for (const auto &task : std::filesystem::directory_iterator(tasks)) {
      if (read_line(task.path() / "comm") != "blofeld-audio") { continue; }
      if (!output.status().waiting_for_source) { continue; }
      const auto name = task.path().filename().string();
      ThreadId result = 0;
      const auto converted = std::from_chars(name.data(), name.data() + name.size(), result);
      pcm::require(converted.ec == std::errc{} && converted.ptr == name.data() + name.size() &&
                       result > 0,
                   "The named worker has an actual numeric task identity");
      return result;
    }
    std::this_thread::yield();
  }
}
}

void closed_underrun(const std::string &server) {
  Parker parker;
  AudioRuntime output(options(server));
  const Release cleanup{parker};
  constexpr unsigned initial = 4096;
  for (unsigned index = 0; index < initial; ++index) { output.accept(frame_at(index)); }
  wait_for([&] { return output.status().state == AudioRuntimeState::playing; }, output);
  wait_for([&] { return output.status().written == initial; }, output);
  parker.park(sleeping_worker(output));
  // The server drains independently while the actual worker cannot dispatch
  // the notification. Publish the last frame and closure before releasing it.
  constexpr std::chrono::milliseconds starve{150};
  constexpr std::chrono::milliseconds close_first{10};
  std::this_thread::sleep_for(starve);
  output.accept(frame_at(initial));
  const std::jthread resume([&] {
    std::this_thread::sleep_for(close_first);
    parker.release();
  });
  const auto failure = pcm::rejects([&] { output.finish(); });
  constexpr std::int64_t expected_byte = 32768;
  pcm::require(
      failure.find("underrun") != std::string::npos &&
          failure.find("byte=" + std::to_string(expected_byte)) != std::string::npos &&
          output.status().state == AudioRuntimeState::failed && output.status().underflows != 0 &&
          output.status().accepted == initial + 1,
      "A delayed underrun before the final frame cannot become a successful closed stream");
}
}
