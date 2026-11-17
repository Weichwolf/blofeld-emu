// SPDX-License-Identifier: GPL-3.0-only
extern "C" {
#include <signal.h>
}

#include "memory/faults.hpp"

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <system_error>
#if defined(__linux__)
#include <bits/types/siginfo_t.h>
#include <bits/types/sigset_t.h>
#include <sys/ucontext.h>
#else
#include <sys/signal.h>
#include <ucontext.h>
#endif
#include <unistd.h>

namespace blofeld::host::memory {
static_assert(std::atomic<Faults *>::is_always_lock_free);
std::atomic<Faults *> Faults::active_{nullptr};

Faults::Faults(const Claim claim) : claim_(claim) {
  if (claim == nullptr || active_.load(std::memory_order_relaxed) != nullptr) {
    throw std::invalid_argument("Memory fault ownership requires one nonnull process scope");
  }
  for (std::size_t index = 0; index < signals.size(); ++index) {
    if (sigaction(signals[index], nullptr, &previous_[index]) != 0) {
      throw std::system_error(errno, std::generic_category(), "Read memory fault action");
    }
  }
  active_.store(this, std::memory_order_release);
  for (std::size_t index = 0; index < signals.size(); ++index) {
    auto action = previous_[index];
    action.sa_sigaction = handle;
    action.sa_flags = SA_SIGINFO | (action.sa_flags & (SA_ONSTACK | SA_RESTART | SA_NODEFER));
    if (sigaction(signals[index], &action, nullptr) != 0) {
      const auto reason = errno;
      while (index != 0) {
        --index;
        sigaction(signals[index], &previous_[index], nullptr);
      }
      active_.store(nullptr, std::memory_order_release);
      throw std::system_error(reason, std::generic_category(), "Install memory fault action");
    }
  }
}

Faults::~Faults() {
  for (std::size_t index = 0; index < signals.size(); ++index) {
    sigaction(signals[index], &previous_[index], nullptr);
  }
  active_.store(nullptr, std::memory_order_release);
}

void Faults::handle(const int signal, siginfo_t *info, void *context) noexcept {
  const auto *state = static_cast<ucontext_t *>(context);
#if defined(__linux__) && defined(__x86_64__)
  const auto pc = static_cast<std::uintptr_t>(state->uc_mcontext.gregs[REG_RIP]);
  const bool writing = (state->uc_mcontext.gregs[REG_ERR] & 2) != 0;
#elif defined(__APPLE__) && defined(__aarch64__)
  const auto pc =
      static_cast<std::uintptr_t>(__darwin_arm_thread_state64_get_pc(state->uc_mcontext->__ss));
  const bool writing = (state->uc_mcontext->__es.__esr & (1U << 6)) != 0;
#elif defined(__APPLE__) && defined(__x86_64__)
  const auto pc = static_cast<std::uintptr_t>(state->uc_mcontext->__ss.__rip);
  const bool writing = (state->uc_mcontext->__es.__err & 2) != 0;
#else
#error "Memory fault context is not implemented for this native host"
#endif
  const auto *owner = active_.load(std::memory_order_acquire);
  const auto reason = errno;
  sigset_t handler_mask{};
  if (pthread_sigmask(SIG_SETMASK, &state->uc_sigmask, &handler_mask) != 0) { _exit(2); }
  if (owner->claim_(reinterpret_cast<std::uintptr_t>(info->si_addr), writing, pc)) { _exit(3); }
  if (pthread_sigmask(SIG_SETMASK, &handler_mask, nullptr) != 0) { _exit(4); }
  errno = reason;
  const auto &prior = owner->previous_[signal == SIGSEGV ? 0U : 1U];
  if (prior.sa_handler == SIG_IGN) { return; }
  if (prior.sa_handler == SIG_DFL) {
    sigaction(signal, &prior, nullptr);
    static_cast<void>(raise(signal));
  } else if ((prior.sa_flags & SA_SIGINFO) != 0) {
    prior.sa_sigaction(signal, info, context);
  } else {
    prior.sa_handler(signal);
  }
}
}
