// SPDX-License-Identifier: GPL-3.0-only
#include "midi/input.hpp"
#include "midi/ump.hpp"

#include <CoreFoundation/CFBase.h>
#include <CoreFoundation/CFRunLoop.h>
#include <CoreFoundation/CFString.h>
#include <CoreMIDI/MIDIServices.h>
#include <MacTypes.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <mach/mach_time.h>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace blofeld::host::midi {
namespace {
constexpr std::size_t capacity = 1024;
static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
static_assert(std::atomic<unsigned>::is_always_lock_free);
static_assert(std::atomic<const char *>::is_always_lock_free);

void require(const OSStatus status, const char *operation) {
  if (status != noErr) {
    throw std::runtime_error(std::string(operation) + ": OSStatus=" + std::to_string(status));
  }
}

std::int32_t identifier(const MIDIEndpointRef endpoint) {
  SInt32 result = 0;
  require(MIDIObjectGetIntegerProperty(endpoint, kMIDIPropertyUniqueID, &result),
          "Query MIDI source identifier");
  return result;
}

std::string name(const MIDIEndpointRef endpoint) {
  CFStringRef value = nullptr;
  require(MIDIObjectGetStringProperty(endpoint, kMIDIPropertyDisplayName, &value),
          "Query MIDI source name");
  if (value == nullptr) { throw std::runtime_error("Missing MIDI source name"); }
  constexpr std::size_t maximum = 4096;
  std::array<char, maximum> text{};
  const auto converted = CFStringGetCString(
      value, text.data(), static_cast<CFIndex>(text.size()), kCFStringEncodingUTF8);
  CFRelease(value);
  if (converted == 0) { throw std::runtime_error("MIDI source name exceeds UTF-8 bounds"); }
  return text.data();
}

mach_timebase_info_data_t host_timebase() {
  mach_timebase_info_data_t result{};
  require(mach_timebase_info(&result), "Query MIDI host timebase");
  if (result.numer == 0 || result.denom == 0) {
    throw std::runtime_error("Invalid MIDI host timebase");
  }
  return result;
}

std::uint64_t nanoseconds(const std::uint64_t ticks, const mach_timebase_info_data_t timebase) {
  const auto whole = ticks / timebase.denom;
  const auto fraction = (ticks % timebase.denom) * timebase.numer / timebase.denom;
  if (whole > (std::numeric_limits<std::uint64_t>::max() - fraction) / timebase.numer) {
    throw std::overflow_error("Native MIDI timestamp exceeds nanosecond range");
  }
  return (whole * timebase.numer) + fraction;
}

struct Client {
  Client() = default;
  Client(const Client &) = delete;
  Client &operator=(const Client &) = delete;
  MIDIClientRef value = 0;

  ~Client() {
    if (value != 0) { MIDIClientDispose(value); }
  }
};

class Callbacks {
public:
  class Call {
  public:
    explicit Call(Callbacks &owner) : owner_(owner) {
      owner_.count_.fetch_add(1, std::memory_order_acq_rel);
    }

    ~Call() { owner_.count_.fetch_sub(1, std::memory_order_release); }

    Call(const Call &) = delete;
    Call &operator=(const Call &) = delete;

    [[nodiscard]] bool active() const noexcept {
      return owner_.enabled_.load(std::memory_order_acquire);
    }

  private:
    Callbacks &owner_;
  };

  ~Callbacks() {
    close();
    while (count_.load(std::memory_order_acquire) != 0) { std::this_thread::yield(); }
  }

  void open() noexcept { enabled_.store(true, std::memory_order_release); }

  void close() noexcept { enabled_.store(false, std::memory_order_release); }

private:
  std::atomic<unsigned> count_{0};
  std::atomic<bool> enabled_{false};
};

class Events {
public:
  Events() {
    CFRunLoopSourceContext context{};
    context.info = this;
    context.perform = [](void *owner) { CFRunLoopStop(static_cast<Events *>(owner)->loop_); };
    source_ = CFRunLoopSourceCreate(kCFAllocatorDefault, 0, &context);
    if (source_ == nullptr) { throw std::runtime_error("Cannot create MIDI owner wake source"); }
    CFRunLoopAddSource(loop_, source_, kCFRunLoopDefaultMode);
  }

  ~Events() {
    CFRunLoopRemoveSource(loop_, source_, kCFRunLoopDefaultMode);
    CFRelease(source_);
  }

  Events(const Events &) = delete;
  Events &operator=(const Events &) = delete;

  void run(const std::stop_token &stop) {
    const std::stop_callback cancelled(stop, [this] {
      CFRunLoopSourceSignal(source_);
      CFRunLoopWakeUp(loop_);
    });
    if (!stop.stop_requested()) { CFRunLoopRun(); }
    if (!stop.stop_requested()) {
      throw std::runtime_error("Native MIDI notification runloop stopped unexpectedly");
    }
  }

private:
  CFRunLoopRef loop_ = CFRunLoopGetCurrent();
  CFRunLoopSourceRef source_ = nullptr;
};

void runtime() {
  struct Server {
    enum class Phase : std::uint8_t { opening, ready, failed };
    std::atomic<Phase> phase{Phase::opening};
    std::exception_ptr error;
    std::jthread owner;

    Server() {
      owner = std::jthread([this](const std::stop_token &stop) {
        try {
          Events events;
          Client client;
          require(
              MIDIClientCreate(CFSTR("blofeld-emu MIDI runtime"), nullptr, nullptr, &client.value),
              "Open native MIDI runtime");
          phase.store(Phase::ready, std::memory_order_release);
          phase.notify_all();
          events.run(stop);
        } catch (...) {
          error = std::current_exception();
          phase.store(Phase::failed, std::memory_order_release);
          phase.notify_all();
        }
      });
      phase.wait(Phase::opening, std::memory_order_acquire);
      check();
    }

    void check() const {
      if (phase.load(std::memory_order_acquire) == Phase::failed) { std::rethrow_exception(error); }
    }
  };

  // Keep the first client and its notification runloop alive until process exit.
  static const Server server;
  server.check();
}

}

struct Input::Impl {
  const Route route;
  Ump decoder;
  std::array<Packet, capacity> packets{};
  std::atomic<std::uint64_t> head{0}, tail{0}, next_check{0};
  std::atomic<const char *> error{nullptr};
  std::atomic_flag receiving = ATOMIC_FLAG_INIT;
  std::uint64_t previous = 0;
  const mach_timebase_info_data_t timebase = host_timebase();
  std::atomic<MIDIEndpointRef> endpoint{0};
  MIDIPortRef port = 0;
  Callbacks callbacks;
  Client client;

  explicit Impl(const Route selected) : route(selected), decoder(selected.group) {
    runtime();
    callbacks.open();
    auto *self = this;
    require(MIDIClientCreateWithBlock(
                CFSTR("blofeld-emu MIDI"),
                &client.value,
                ^(const MIDINotification *notice) { self->notification(notice); }),
            "Create native MIDI client");
    MIDIObjectRef selected_object = 0;
    MIDIObjectType type = kMIDIObjectType_Other;
    require(MIDIObjectFindByUniqueID(route.source, &selected_object, &type),
            "Find selected native MIDI source");
    if (selected_object == 0 || type != kMIDIObjectType_Source) {
      throw std::runtime_error("Selected MIDI identifier is not an available source");
    }
    endpoint.store(selected_object, std::memory_order_release);
    require(MIDIInputPortCreateWithProtocol(client.value,
                                            CFSTR("DIN input"),
                                            kMIDIProtocol_1_0,
                                            &port,
                                            ^(const MIDIEventList *events, void *)__attribute__((
                                                nonblocking)) { self->receive(events); }),
            "Create native MIDI input port");
    require(MIDIPortConnectSource(port, endpoint.load(std::memory_order_acquire), nullptr),
            "Connect selected MIDI source");
  }

  ~Impl() {
    callbacks.close();
    if (port != 0) {
      MIDIPortDisconnectSource(port, endpoint.load(std::memory_order_acquire));
      MIDIPortDispose(port);
    }
    // Dispose callbacks before their storage and decoder are destroyed.
    if (client.value != 0) {
      MIDIClientDispose(client.value);
      client.value = 0;
    }
  }

  void fail(const char *reason) noexcept {
    const char *empty = nullptr;
    error.compare_exchange_strong(empty, reason, std::memory_order_release);
  }

  void notification(const MIDINotification *notice) noexcept {
    const Callbacks::Call call(callbacks);
    if (!call.active()) { return; }
    if (notice != nullptr && notice->messageID == kMIDIMsgObjectRemoved &&
        notice->messageSize >= sizeof(MIDIObjectAddRemoveNotification)) {
      const auto *removed = reinterpret_cast<const MIDIObjectAddRemoveNotification *>(notice);
      if (removed->child == endpoint.load(std::memory_order_acquire)) {
        fail("Selected MIDI source disconnected");
      }
    } else if (notice != nullptr && notice->messageID == kMIDIMsgIOError) {
      fail("Native MIDI transport I/O failure");
    }
  }

  void check_source() noexcept {
    const auto now = mach_absolute_time();
    auto next = next_check.load(std::memory_order_relaxed);
    constexpr std::uint64_t interval_ns = 50000000;
    const auto interval = interval_ns * timebase.denom / timebase.numer;
    if (now < next || now > std::numeric_limits<std::uint64_t>::max() - interval ||
        !next_check.compare_exchange_strong(next, now + interval, std::memory_order_relaxed)) {
      return;
    }
    SInt32 actual = 0;
    const auto status = MIDIObjectGetIntegerProperty(
        endpoint.load(std::memory_order_acquire), kMIDIPropertyUniqueID, &actual);
    if (status != noErr || actual != route.source) { fail("Selected MIDI source disconnected"); }
  }

  void push(const Packet packet) noexcept {
    const auto at = head.load(std::memory_order_relaxed);
    if (packet.timestamp == 0 || packet.timestamp < previous) {
      fail("Native MIDI source timestamp is missing or moved backwards");
    } else if (at == std::numeric_limits<std::uint64_t>::max() ||
               at - tail.load(std::memory_order_acquire) == capacity) {
      fail("Native MIDI input queue overflow");
    } else {
      packets[at % capacity] = packet;
      previous = packet.timestamp;
      head.store(at + 1, std::memory_order_release);
    }
  }

  void receive(const MIDIEventList *events) noexcept [[clang::nonblocking]] {
    const Callbacks::Call call(callbacks);
    if (!call.active()) { return; }
    if (error.load(std::memory_order_acquire) != nullptr) { return; }
    if (receiving.test_and_set(std::memory_order_acquire)) {
      fail("Concurrent callbacks on the selected MIDI source");
      return;
    }
    if (events == nullptr || events->protocol != kMIDIProtocol_1_0) {
      fail("Unexpected native MIDI input protocol");
    } else {
      const auto *packet = &events->packet[0];
      for (UInt32 index = 0;
           index < events->numPackets && error.load(std::memory_order_acquire) == nullptr;
           ++index) {
        const std::span words(packet->words, packet->wordCount);
        std::size_t offset = 0;
        while (offset < words.size() && error.load(std::memory_order_acquire) == nullptr) {
          const std::size_t count = words[offset] >> 28U == 3 ? 2 : 1;
          const auto message = decoder.decode(
              words.subspan(offset, std::min(count, words.size() - offset)), packet->timeStamp);
          if (decoder.error() != nullptr) {
            fail(decoder.error());
          } else if (message && message->size != 0) {
            push(*message);
          }
          offset += count;
        }
        packet = MIDIEventPacketNext(packet);
      }
    }
    receiving.clear(std::memory_order_release);
  }
};

std::vector<Source> Input::sources() {
  runtime();
  constexpr unsigned attempts = 3;
  for (unsigned attempt = 0; attempt < attempts; ++attempt) {
    try {
      std::vector<Source> result;
      const auto count = MIDIGetNumberOfSources();
      for (ItemCount index = 0; index < count; ++index) {
        const auto endpoint = MIDIGetSource(index);
        if (endpoint == 0) { throw std::runtime_error("MIDI sources changed during discovery"); }
        result.push_back({.id = identifier(endpoint), .name = name(endpoint)});
      }
      return result;
    } catch (const std::runtime_error &) {
      if (attempt + 1 == attempts) { throw; }
    }
  }
  throw std::runtime_error("Cannot obtain a stable native MIDI source list");
}

std::uint64_t Input::now() {
  return nanoseconds(mach_absolute_time(), host_timebase());
}

Input::Input(const Route route) : impl_(std::make_unique<Impl>(route)) {}

Input::~Input() = default;

void Input::check() const {
  runtime();
  impl_->check_source();
  if (const auto *error = impl_->error.load(std::memory_order_acquire)) {
    throw std::runtime_error("MIDI source=" + std::to_string(impl_->route.source) +
                             " group=" + std::to_string(impl_->route.group) + ": " + error);
  }
}

std::optional<Packet> Input::read() {
  check();
  const auto at = impl_->tail.load(std::memory_order_relaxed);
  if (at == impl_->head.load(std::memory_order_acquire)) { return {}; }
  auto packet = impl_->packets[at % capacity];
  impl_->tail.store(at + 1, std::memory_order_release);
  packet.timestamp = nanoseconds(packet.timestamp, impl_->timebase);
  return packet;
}
}
