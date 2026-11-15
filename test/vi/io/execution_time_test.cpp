// SPDX-License-Identifier: GPL-3.0-only
#include "io/execution_time.hpp"
#include "io/peer_port.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using blofeld::vi::ExecutionTime;

void require(const bool condition, const char *message) {
  if (!condition) { throw std::runtime_error(message); }
}

void cancellation_tokens(const bool source) {
  ExecutionTime time;
  std::stop_source first;
  for (std::uint64_t step = 1; step <= 2; ++step) {
    std::promise<bool> completion;
    auto result = completion.get_future();
    std::jthread owner([&] {
      completion.set_value(source ? time.dsp_limit(step - 1, first.get_token()).has_value()
                                  : time.wait_for_dsp(step, first.get_token()));
    });
    const auto blocked = result.wait_for(std::chrono::milliseconds(10));
    std::optional<ExecutionTime::DspGrant> supplied;
    std::optional<std::jthread> publisher;
    if (source) {
      static_cast<void>(time.publish_mcu(step));
    } else {
      publisher.emplace([&] { supplied = time.dsp_limit(step, {}); });
    }
    const auto ready = result.wait_for(std::chrono::seconds(2));
    const bool published = time.publish_mcu(source ? step : step + 1);
    if (ready != std::future_status::ready) {
      static_cast<void>(first.request_stop());
      time.hold_dsp();
    }
    owner.join();
    if (publisher) { publisher->join(); }
    require(blocked == std::future_status::timeout && ready == std::future_status::ready &&
                result.get() && published &&
                (source || supplied == ExecutionTime::DspGrant{.until = step + 1}),
            "One cancellation token permits consecutive real progress waits");
  }
  std::stop_source next;
  std::promise<bool> completion;
  auto result = completion.get_future();
  std::jthread owner([&] {
    completion.set_value(source ? time.dsp_limit(2, next.get_token()).has_value()
                                : time.wait_for_dsp(3, next.get_token()));
  });
  const auto blocked = result.wait_for(std::chrono::milliseconds(10));
  static_cast<void>(first.request_stop());
  const auto old_cancelled = result.wait_for(std::chrono::milliseconds(10));
  static_cast<void>(next.request_stop());
  const auto cancelled = result.wait_for(std::chrono::seconds(2));
  if (cancelled != std::future_status::ready) { time.hold_dsp(); }
  owner.join();
  const bool advanced = result.get();
  const bool published = time.publish_mcu(3);
  require(blocked == std::future_status::timeout && old_cancelled == std::future_status::timeout &&
              cancelled == std::future_status::ready && !advanced && published &&
              time.dsp_limit(2, {}) == ExecutionTime::DspGrant{.until = 3},
          "A replacement owner token cancels without old-token cancellation or invented time");
}

void cancellation_lifetime() {
  std::stop_source stop;
  {
    ExecutionTime time;
    std::promise<bool> completion;
    auto result = completion.get_future();
    std::jthread owner([&] { completion.set_value(time.wait_for_dsp(1, stop.get_token())); });
    const auto blocked = result.wait_for(std::chrono::milliseconds(10));
    std::jthread source([&] { static_cast<void>(time.dsp_limit(1, {})); });
    const auto ready = result.wait_for(std::chrono::seconds(2));
    if (ready != std::future_status::ready) { time.hold_dsp(); }
    static_cast<void>(time.publish_mcu(2));
    owner.join();
    source.join();
    require(blocked == std::future_status::timeout && ready == std::future_status::ready &&
                result.get(),
            "Complete a genuinely blocked wait before destroying the coordinator");
  }
  require(stop.request_stop(), "Destroying the coordinator detaches its cancellation callback");
}

void forecast_overlap(const std::uint64_t source_bound) {
  ExecutionTime time;
  constexpr std::uint64_t receiver_bound = 200;

  struct Bound {
    std::uint64_t until;
    unsigned queries = 0;
  } bound{.until = source_bound};

  const ExecutionTime::DspForecast forecast{.owner = &bound, .query = [](void *owner) {
                                              auto &value = *static_cast<Bound *>(owner);
                                              ++value.queries;
                                              return value.until;
                                            }};
  std::promise<std::optional<std::uint64_t>> completion;
  auto receiver = completion.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    completion.set_value(time.idle_mcu(0, receiver_bound, stop, ExecutionTime::McuWork::running));
  });
  std::optional<ExecutionTime::DspGrant> grant;
  std::jthread dsp([&](const std::stop_token &stop) {
    grant = time.dsp_limit(0, stop, ExecutionTime::DspBoundary::exact, forecast);
  });
  dsp.join();
  const auto ready = receiver.wait_for(std::chrono::seconds(2));
  mcu.request_stop();
  mcu.join();
  const auto until = source_bound < receiver_bound ? source_bound : receiver_bound;
  require(ready == std::future_status::ready && receiver.get() == until && grant &&
              grant->until == until && bound.queries == 1,
          "Both owners receive the joint no-event bound before source work completes");

  std::promise<bool> transaction;
  auto admitted = transaction.get_future();
  std::jthread writer(
      [&](const std::stop_token &stop) { transaction.set_value(time.enter_mcu(until, stop)); });
  require(time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = until},
          "The paused source still reports its original actual clock");
  const auto premature = admitted.wait_for(std::chrono::milliseconds(10));
  std::jthread source([&](const std::stop_token &stop) {
    static_cast<void>(time.dsp_limit(until, stop, ExecutionTime::DspBoundary::exact, forecast));
  });
  const auto settled = admitted.wait_for(std::chrono::seconds(2));
  writer.request_stop();
  writer.join();
  time.leave_mcu();
  source.request_stop();
  source.join();
  require(premature == std::future_status::timeout && settled == std::future_status::ready &&
              admitted.get() && bound.queries == 1,
          "A no-event bound cannot replace actual source completion at a shared operation");
}

void cancelled_forecast() {
  ExecutionTime time;
  std::stop_source cancelled;
  std::promise<std::optional<std::uint64_t>> completion;
  auto receiver = completion.get_future();
  constexpr std::uint64_t receiver_until = 200;
  std::jthread mcu([&](const std::stop_token &stop) {
    completion.set_value(time.idle_mcu(0, receiver_until, stop, ExecutionTime::McuWork::running));
  });
  std::optional<ExecutionTime::DspGrant> grant;
  std::jthread dsp([&] {
    grant =
        time.dsp_limit(0,
                       cancelled.get_token(),
                       ExecutionTime::DspBoundary::exact,
                       {.owner = &cancelled, .query = [](void *owner) {
                          static_cast<void>(static_cast<std::stop_source *>(owner)->request_stop());
                          constexpr std::uint64_t until = 80;
                          return until;
                        }});
  });
  dsp.join();
  const auto advanced = receiver.wait_for(std::chrono::milliseconds(10));
  mcu.request_stop();
  mcu.join();
  require(!grant && advanced == std::future_status::timeout && !receiver.get(),
          "A cancelled source query cannot grant either owner future execution");
}

void first_wait() {
  constexpr std::uint64_t later_tick = 100;
  ExecutionTime time;
  require(!time.mcu_wait_time(), "Starting execution does not fabricate an application wait");
  require(time.publish_mcu(later_tick), "Ordinary MCU progress is independent of WAI");
  require(!time.mcu_wait_time(), "Advancing the native owner alone is no boot-completion signal");
  std::jthread mcu([&] {
    time.publish_mcu_wait(0);
    time.publish_mcu_wait(later_tick);
  });
  mcu.join();
  require(time.mcu_wait_time() == 0, "Zero is a valid first wait and later waits retain its time");
  time.hold_dsp();
  time.release_dsp(later_tick);
  require(time.mcu_wait_time() == 0, "Resetting the DSP cannot rewrite the MCU lifecycle");
}

void ordered_transactions() {
  ExecutionTime time;
  std::uint64_t data = 0;
  std::atomic<bool> done{false};
  bool ordered = true;
  constexpr std::uint64_t count = 64;
  constexpr std::uint64_t spacing = 100;
  std::jthread dsp([&](const std::stop_token &stop) {
    std::uint64_t now = 0;
    for (std::uint64_t expected = 0; expected <= count; ++expected) {
      const auto limit = time.dsp_limit(now, stop);
      if (!limit) {
        ordered = false;
        break;
      }
      ordered = ordered && limit->until == (expected + 1) * spacing && data == expected;
      now = limit->until;
    }
    done.store(true);
    done.notify_one();
  });
  for (std::uint64_t value = 1; value <= count; ++value) {
    require(time.enter_mcu(value * spacing, {}), "Admit a real shared-register transaction");
    require(data == value - 1, "The previous phase remains until its producer changes it");
    data = value;
    time.leave_mcu();
  }
  require(time.publish_mcu((count + 1) * spacing), "Publish completed local work without waiting");
  done.wait(false);
  dsp.join();
  require(ordered, "Native grants observe every committed phase in device-time order");
  require(!time.publish_mcu(0), "A published device frontier cannot move backwards");
}

void reset_and_cancellation() {
  ExecutionTime time;
  time.hold_dsp();
  constexpr std::uint64_t reset_time = 120;
  require(time.enter_mcu(reset_time, {}), "A held DSP cannot block MCU reset setup");
  require(!time.settle_dsp(reset_time, {}),
          "Settlement cannot overlap an active shared transaction even during reset");
  time.leave_mcu();
  require(!time.dsp_limit(0, {}), "A reset interrupts a DSP grant wait");
  time.release_dsp(reset_time);
  require(time.dsp_origin() == reset_time, "A fresh DSP epoch uses the MCU reset-release time");
  bool cancelled = false;
  std::jthread dsp(
      [&](const std::stop_token &stop) { cancelled = !time.dsp_limit(reset_time, stop); });
  dsp.request_stop();
  dsp.join();
  require(cancelled, "Shutdown cancels a DSP waiting for a future device event");
  // libc++ mutates the source on request_stop(); libstdc++ accepts a const source.
#if defined(_LIBCPP_VERSION)
  std::stop_source stop;
#else
  const std::stop_source stop;
#endif
  stop.request_stop();
  require(!time.enter_mcu(reset_time, stop.get_token()),
          "Cancellation never grants a shared access or advances the source");
}

void publication_wakeups() {
  ExecutionTime time;
  constexpr std::uint64_t count = 512;
  constexpr std::uint64_t spacing = 100;
  std::atomic<std::uint64_t> acknowledged{0};
  bool ordered = true;
  std::jthread dsp([&](const std::stop_token &stop) {
    std::uint64_t now = 0;
    for (std::uint64_t expected = 1; expected <= count; ++expected) {
      const auto next = time.dsp_limit(now, stop);
      if (!next) {
        ordered = false;
        return;
      }
      ordered = ordered && next->until == expected * spacing;
      now = next->until;
      acknowledged.store(expected, std::memory_order_release);
      acknowledged.notify_one();
    }
  });
  for (std::uint64_t value = 1; value <= count; ++value) {
    require(time.publish_mcu(value * spacing) && time.publish_mcu(value * spacing),
            "A new frontier and its repeated publication retain actual device time");
    auto observed = acknowledged.load(std::memory_order_acquire);
    while (observed < value) {
      acknowledged.wait(observed, std::memory_order_acquire);
      observed = acknowledged.load(std::memory_order_acquire);
    }
  }
  dsp.join();
  require(ordered && acknowledged.load() == count,
          "Publication racing wait registration cannot lose a wakeup or reorder a grant");
  require(!time.publish_mcu(0), "Rejected backwards progress retains the final frontier");
}

void repeated_timestamp_transactions() {
  ExecutionTime time;
  constexpr std::uint64_t timestamp = 100;
  constexpr std::uint64_t count = 1024;
  bool completed = false;
  std::jthread dsp([&](const std::stop_token &stop) {
    completed = time.dsp_limit(timestamp, stop) == ExecutionTime::DspGrant{.until = timestamp + 1};
  });
  for (std::uint64_t index = 0; index < count; ++index) {
    require(time.enter_mcu(timestamp, {}), "Each same-time transaction receives its own grant");
    require(!time.enter_mcu(timestamp, {}), "An outstanding transaction cannot be entered twice");
    time.leave_mcu();
  }
  require(time.publish_mcu(timestamp + 1), "Only genuine device progress advances the frontier");
  dsp.join();
  require(completed, "Reused timestamps cannot hide a request or reuse a stale grant");
  bool backwards = false;
  try {
    static_cast<void>(time.dsp_limit(timestamp - 1, {}));
  } catch (const std::logic_error &) { backwards = true; }
  require(backwards, "Atomic grants still reject backwards DSP progress");
}

void cancelled_transactions() {
  constexpr std::uint64_t timestamp = 100;
  ExecutionTime time;
  bool accepted = true;
  std::jthread mcu(
      [&](const std::stop_token &stop) { accepted = time.enter_mcu(timestamp, stop); });
  require(time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = timestamp},
          "The DSP sees publication while the MCU awaits actual boundary admission");
  mcu.request_stop();
  mcu.join();
  require(!accepted, "Cancellation withdraws an ungranted transaction");
  time.hold_dsp();
  require(time.enter_mcu(timestamp, {}), "Cancellation releases ownership for reset setup");
  time.leave_mcu();
  time.release_dsp(timestamp);

  bool cancelled = false;
  std::jthread dsp(
      [&](const std::stop_token &stop) { cancelled = !time.dsp_limit(timestamp, stop); });
  require(time.enter_mcu(timestamp, {}), "A real grant precedes cancellation of its DSP waiter");
  dsp.request_stop();
  dsp.join();
  require(cancelled, "Shutdown wakes a DSP even while the MCU still holds its granted access");
  time.leave_mcu();
}

void unmask_after_earlier_dsp_work() {
  ExecutionTime time;
  blofeld::vi::PeerPort peer;
  constexpr std::uint64_t earlier = 50;
  constexpr std::uint64_t unmask = 100;
  require(time.publish_mcu(earlier) &&
              time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = earlier},
          "The DSP owns an earlier interval while the MCU remains masked");
  std::promise<void> published;
  auto announced = published.get_future();
  auto mcu = std::async(std::launch::async, [&] {
    const bool valid = time.publish_mcu(unmask);
    published.set_value();
    return valid && time.wait_for_dsp(unmask, {}) && peer.mcu_interrupt_ready(true) &&
           time.publish_mcu(unmask + 1);
  });
  announced.wait();
  require(mcu.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout &&
              !peer.mcu_interrupt_ready(),
          "A grant is not completed DSP work and cannot publish a future unmask");
  // A pulse at ticks25..30 belongs to the DSP grant, before the unmask at100.
  require(peer.dsp_ready(true) && peer.dsp_ready(false), "Execute the earlier masked pulse");
  require(time.dsp_limit(earlier, {}) == ExecutionTime::DspGrant{.until = unmask} &&
              time.dsp_limit(unmask, {}) == ExecutionTime::DspGrant{.until = unmask + 1},
          "Completed DSP progress releases the unmask without a fabricated clock advance");
  require(mcu.get() && peer.mcu_interrupt_ready() && !peer.mcu_interrupt(),
          "The new MCU window cannot retain a pulse from an earlier masked interval");
}

void cancelled_dsp_prefix() {
  ExecutionTime time;
  constexpr std::uint64_t timestamp = 100;
  bool accepted = true;
  std::jthread mcu(
      [&](const std::stop_token &stop) { accepted = time.wait_for_dsp(timestamp, stop); });
  require(time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = timestamp},
          "The waiting MCU publishes actual completed work");
  mcu.request_stop();
  mcu.join();
  require(!accepted, "Shutdown cancels an uncompleted DSP prefix");
  time.hold_dsp();
  require(time.wait_for_dsp(timestamp, {}), "A held DSP has no earlier notification to produce");
  time.release_dsp(timestamp);
  require(time.wait_for_dsp(timestamp, {}), "A released epoch retains its actual origin");
  bool backwards = false;
  try {
    static_cast<void>(time.wait_for_dsp(timestamp - 1, {}));
  } catch (const std::logic_error &) { backwards = true; }
  require(backwards, "An admission barrier rejects backwards MCU progress");
}

void conditional_idle_wake() {
  constexpr std::uint64_t now = 100;
  constexpr std::uint64_t wake = 500;
  constexpr std::uint64_t deadline = 1000;
  ExecutionTime time;
  require(time.publish_mcu(now) && time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = now},
          "Only actual MCU work establishes the initial completed frontier");
  std::promise<std::optional<std::uint64_t>> completion;
  auto result = completion.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    try {
      completion.set_value(time.wait_for_dsp(now, stop) ? time.idle_mcu(now, deadline, stop)
                                                        : std::nullopt);
    } catch (...) { completion.set_exception(std::current_exception()); }
  });
  require(time.dsp_limit(now, {}) == ExecutionTime::DspGrant{.until = deadline} &&
              result.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout,
          "An idle execution grant is neither completed DSP work nor elapsed MCU time");
  require(time.wake_mcu(wake) && !time.wake_mcu(wake + 1),
          "The first actual source event ends the conditional window exactly once");
  require(result.get() == wake, "The waiting MCU resumes at the real source timestamp");
  mcu.join();
  require(time.publish_mcu(now), "Lending time never advances the completed MCU frontier");
  std::promise<std::optional<ExecutionTime::DspGrant>> source;
  auto next = source.get_future();
  std::jthread dsp(
      [&](const std::stop_token &stop) { source.set_value(time.dsp_limit(wake, stop)); });
  require(next.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout,
          "After a wake the source cannot reuse its old long execution window");
  require(time.publish_mcu(wake + 1) && next.get() == ExecutionTime::DspGrant{.until = wake + 1},
          "Only new actual MCU progress releases the source again");
  dsp.join();
}

void conditional_idle_expiry() {
  constexpr std::uint64_t now = 100;
  constexpr std::uint64_t deadline = 1000;
  ExecutionTime time;
  require(time.publish_mcu(now) && time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = now},
          "Establish the original completed prefix");
  std::promise<std::optional<std::uint64_t>> completion;
  auto result = completion.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    const auto resumed =
        time.wait_for_dsp(now, stop) ? time.idle_mcu(now, deadline, stop) : std::nullopt;
    completion.set_value(resumed);
    if (resumed) { static_cast<void>(time.publish_mcu(deadline + 1)); }
  });
  require(time.dsp_limit(now, {}) == ExecutionTime::DspGrant{.until = deadline} &&
              !time.wake_mcu(deadline),
          "The local deadline is outside the exclusive earlier-event window");
  require(time.dsp_limit(deadline, {}) == ExecutionTime::DspGrant{.until = deadline + 1} &&
              result.get() == deadline,
          "An event-free completed source prefix releases the real local device deadline");
  mcu.join();
}

void conditional_idle_cancellation() {
  constexpr std::uint64_t deadline = 1000;
  ExecutionTime time;
  std::optional<std::uint64_t> resumed = deadline;
  std::jthread mcu(
      [&](const std::stop_token &stop) { resumed = time.idle_mcu(0, deadline, stop); });
  require(time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = deadline},
          "The source observes the outstanding idle window");
  mcu.request_stop();
  mcu.join();
  require(!resumed && !time.wake_mcu(1) && time.publish_mcu(0),
          "Cancellation withdraws the lease without inventing a wake or completed time");
  time.hold_dsp();
  require(time.idle_mcu(0, deadline, {}) == deadline,
          "A held source cannot produce an earlier peer event");
  time.release_dsp(0);
}

void repeated_idle_generations() {
  constexpr std::uint64_t count = 512;
  constexpr std::uint64_t spacing = 100;
  constexpr std::uint64_t wake_offset = spacing / 2;
  ExecutionTime time;
  bool ordered = true;
  std::jthread mcu([&](const std::stop_token &stop) {
    std::uint64_t now = 0;
    for (std::uint64_t index = 1; index <= count; ++index) {
      if (!time.wait_for_dsp(now, stop)) { return; }
      const auto next = now + (index % 2 == 0 ? spacing : wake_offset);
      const auto wake = time.idle_mcu(now, now + spacing, stop);
      if (!wake || *wake != next) {
        ordered = false;
        return;
      }
      now = *wake;
    }
    ordered = time.publish_mcu(now + 1);
  });
  std::uint64_t ticks = 0;
  for (std::uint64_t index = 1; index <= count; ++index) {
    require(time.dsp_limit(ticks, {}) == ExecutionTime::DspGrant{.until = ticks + spacing},
            "Every fresh generation lends only its own conditional horizon");
    ticks += index % 2 == 0 ? spacing : wake_offset;
    if (index % 2 != 0) {
      require(time.wake_mcu(ticks), "Only the current generation accepts its real source event");
    }
  }
  require(time.dsp_limit(ticks, {}) == ExecutionTime::DspGrant{.until = ticks + 1},
          "The last wake cannot leak a retired lease into subsequent source execution");
  mcu.join();
  require(ordered, "Wait registration and repeated generations preserve every exact wake");
}

void current_tick_settlement() {
  constexpr std::uint64_t now = 100;
  constexpr std::uint64_t later = 120;
  ExecutionTime time;
  std::promise<bool> completed;
  auto settled = completed.get_future();
  std::jthread mcu(
      [&](const std::stop_token &stop) { completed.set_value(time.settle_dsp(now, stop)); });
  require(time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = now} &&
              time.dsp_limit(now, {}) ==
                  ExecutionTime::DspGrant{.until = now + 1,
                                          .kind = ExecutionTime::DspGrant::Kind::current} &&
              settled.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout,
          "A current-instruction permission is not completed source work");
  std::promise<std::optional<ExecutionTime::DspGrant>> advanced;
  auto resumed = advanced.get_future();
  std::jthread dsp(
      [&](const std::stop_token &stop) { advanced.set_value(time.dsp_limit(now + 1, stop)); });
  const auto finished = settled.wait_for(std::chrono::seconds(1));
  const bool unchanged = time.publish_mcu(now);
  const bool parked =
      resumed.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout;
  const bool published = time.publish_mcu(later);
  const auto source_done = resumed.wait_for(std::chrono::seconds(1));
  mcu.request_stop();
  dsp.request_stop();
  mcu.join();
  dsp.join();
  require(
      finished == std::future_status::ready && settled.get() && unchanged && parked && published &&
          source_done == std::future_status::ready &&
          resumed.get() == ExecutionTime::DspGrant{.until = later},
      "Strict source settlement leaves MCU time unchanged and cannot be reused as a future grant");
  require(!time.settle_dsp(now, {}) &&
              !time.settle_dsp(std::numeric_limits<std::uint64_t>::max(), {}),
          "Settlement refuses backwards time and an unrepresentable exclusive endpoint");
}

void source_knowledge(const ExecutionTime::DspBoundary boundary) {
  constexpr std::uint64_t now = 100;
  ExecutionTime time;
  std::promise<std::optional<ExecutionTime::DspGrant>> completion;
  auto resumed = completion.get_future();
  std::jthread source([&](const std::stop_token &stop) {
    completion.set_value(time.dsp_limit(now, stop, boundary));
  });
  const bool settled = time.settle_dsp(now, {});
  const auto parked = resumed.wait_for(std::chrono::milliseconds(10));
  const bool actual = time.publish_mcu(now);
  const bool next = time.publish_mcu(now + 1);
  const auto continued = resumed.wait_for(std::chrono::seconds(1));
  source.request_stop();
  source.join();
  require(settled && actual && parked == std::future_status::timeout && next &&
              continued == std::future_status::ready &&
              resumed.get() == ExecutionTime::DspGrant{.until = now + 1},
          "Source knowledge settles this instant without permission for a later opcode");
}

void cancelled_settlement() {
  constexpr std::uint64_t now = 100;
  ExecutionTime time;
  bool accepted = true;
  std::jthread mcu([&](const std::stop_token &stop) { accepted = time.settle_dsp(now, stop); });
  require(time.dsp_limit(0, {}) == ExecutionTime::DspGrant{.until = now} &&
              time.dsp_limit(now, {}) ==
                  ExecutionTime::DspGrant{.until = now + 1,
                                          .kind = ExecutionTime::DspGrant::Kind::current},
          "The source observes a current-tick permission before cancellation");
  mcu.request_stop();
  mcu.join();
  std::promise<std::optional<ExecutionTime::DspGrant>> completion;
  auto resumed = completion.get_future();
  std::jthread dsp(
      [&](const std::stop_token &stop) { completion.set_value(time.dsp_limit(now, stop)); });
  const auto waiting = resumed.wait_for(std::chrono::milliseconds(10));
  dsp.request_stop();
  dsp.join();
  require(!accepted && time.publish_mcu(now) && waiting == std::future_status::timeout &&
              !resumed.get(),
          "Cancellation revokes settlement permission without fabricating source completion or a "
          "transaction");
  time.hold_dsp();
  require(time.settle_dsp(now, {}),
          "A held execution owner needs no permission to run another instruction");
}

}

int main() try {
  cancellation_tokens(false);
  cancellation_tokens(true);
  cancellation_lifetime();
  constexpr std::uint64_t source_first = 80;
  constexpr std::uint64_t receiver_first = 400;
  forecast_overlap(source_first);
  forecast_overlap(receiver_first);
  cancelled_forecast();
  current_tick_settlement();
  source_knowledge(ExecutionTime::DspBoundary::fractional);
  source_knowledge(ExecutionTime::DspBoundary::quiescent);
  cancelled_settlement();
  first_wait();
  ordered_transactions();
  reset_and_cancellation();
  publication_wakeups();
  repeated_timestamp_transactions();
  cancelled_transactions();
  unmask_after_earlier_dsp_work();
  cancelled_dsp_prefix();
  conditional_idle_wake();
  conditional_idle_expiry();
  conditional_idle_cancellation();
  repeated_idle_generations();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
