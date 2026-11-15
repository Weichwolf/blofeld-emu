// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/io.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"
#include "mcu/devices.hpp"
#include "mcu/execute.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/program.hpp"
#include "support/mcu.hpp"

#include <array>
#include <chrono>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstddef>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <format>
#include <future>
#include <iostream>
#include <memory>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

namespace {
namespace isa = cpu12::isa;
enum class Loop : std::uint8_t { branch, ram, timer, mixed, flags, strobe, latch, input, counted };
constexpr std::uint64_t frontier = 200000, returned = 78, timer_started = 15;
constexpr std::uint64_t first_wait = 250;
constexpr unsigned initial_retired = 19;

struct Prelude {
  unsigned cycles = 0, instructions = 0;
};

constexpr Prelude prelude(const Loop loop) {
  if (loop == Loop::strobe) { return {.cycles = 1 + 3 + 3, .instructions = 3}; }
  return loop == Loop::latch ? Prelude{.cycles = 1 + 3 + 1 + 3, .instructions = 4} : Prelude{};
}

constexpr cpu12::Byte all_bits = 0xff, output_data = 0xa5, interrupt_masks = 0x90;
constexpr cpu12::Address port_q = 0x260, direction_q = port_q + 2;
constexpr cpu12::Address handler_entry = 0x8100, marker = 0x3000;

struct Fixture {
  blofeld::test::McuImage image;
  std::shared_ptr<const blofeld::vi::mcu::Program> program;
  Loop loop;

  explicit Fixture(const Loop body) : loop(body) {
    constexpr auto setup = isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                                        std::array<cpu12::Byte, 2>{0x1f, 0xff},
                                        isa::opcode<isa::Opcode::clr_extended>(),
                                        std::array<cpu12::Byte, 2>{0x18, 0},
                                        isa::opcode<isa::Opcode::ldx_immediate>(),
                                        std::array<cpu12::Byte, 2>{0x18, 1},
                                        isa::opcode<isa::Opcode::ldab_immediate>(),
                                        std::array<cpu12::Byte, 1>{0x80},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0x4d},
                                        isa::opcode<isa::Opcode::stab_direct>(),
                                        std::array<cpu12::Byte, 1>{0x46},
                                        isa::opcode<isa::Opcode::andcc_immediate>(),
                                        std::array<cpu12::Byte, 1>{0xbf},
                                        isa::wai);
    std::vector<cpu12::Byte> code;
    if (body == Loop::strobe) {
      constexpr auto inactive = isa::program(isa::opcode<isa::Opcode::ldaa_immediate>(),
                                             std::array<cpu12::Byte, 1>{1},
                                             isa::opcode<isa::Opcode::staa_extended>(),
                                             std::array<cpu12::Byte, 2>{2, 0x50},
                                             isa::opcode<isa::Opcode::staa_extended>(),
                                             std::array<cpu12::Byte, 2>{2, 0x52});
      code.insert(code.end(), inactive.begin(), inactive.end());
    }
    if (body == Loop::latch) {
      constexpr auto output = isa::program(isa::opcode<isa::Opcode::ldaa_immediate>(),
                                           std::array<cpu12::Byte, 1>{output_data},
                                           isa::opcode<isa::Opcode::staa_extended>(),
                                           std::array<cpu12::Byte, 2>{2, 0x60},
                                           isa::opcode<isa::Opcode::ldab_immediate>(),
                                           std::array<cpu12::Byte, 1>{all_bits},
                                           isa::opcode<isa::Opcode::stab_extended>(),
                                           std::array<cpu12::Byte, 2>{2, 0x62});
      code.insert(code.end(), output.begin(), output.end());
    }
    code.insert(code.end(), setup.begin(), setup.end());
    const auto loop_start = code.size();
    constexpr cpu12::Byte counter_low = 0x45;
    if (body == Loop::counted) {
      constexpr auto delay = isa::program(isa::opcode<isa::Opcode::dbne_relative>(),
                                          std::array<cpu12::Byte, 2>{0x31, 0xfd},
                                          isa::opcode<isa::Opcode::ldab_extended>(),
                                          std::array<cpu12::Byte, 2>{2, 0x60},
                                          isa::opcode<isa::Opcode::ldab_immediate>(),
                                          std::array<cpu12::Byte, 1>{0x80});
      code.insert(code.end(), delay.begin(), delay.end());
    }
    if (body == Loop::ram || body == Loop::mixed) {
      code.push_back(isa::opcode<isa::Opcode::staa_indexed>()[0]);
      code.push_back(0);
    }
    if (body == Loop::timer || body == Loop::mixed) {
      code.push_back(isa::opcode<isa::Opcode::ldab_direct>()[0]);
      code.push_back(counter_low);
    }
    if (body == Loop::latch || body == Loop::input) {
      constexpr auto input = isa::program(isa::opcode<isa::Opcode::ldab_extended>(),
                                          std::array<cpu12::Byte, 2>{2, 0x60});
      code.insert(code.end(), input.begin(), input.end());
    }
    if (body == Loop::flags) {
      const auto acknowledge = isa::program(isa::opcode<isa::Opcode::bset_direct>(),
                                            std::array<cpu12::Byte, 2>{0x37, 0x80});
      code.insert(code.end(), acknowledge.begin(), acknowledge.end());
    }
    if (body == Loop::strobe) {
      constexpr auto inactive = isa::program(isa::opcode<isa::Opcode::staa_extended>(),
                                             std::array<cpu12::Byte, 2>{2, 0x50},
                                             isa::opcode<isa::Opcode::clr_extended>(),
                                             std::array<cpu12::Byte, 2>{2, 0x70},
                                             isa::opcode<isa::Opcode::staa_extended>(),
                                             std::array<cpu12::Byte, 2>{2, 0x40});
      code.insert(code.end(), inactive.begin(), inactive.end());
    }
    code.push_back(isa::opcode<isa::Opcode::bra_relative>()[0]);
    constexpr unsigned byte_values = 256;
    const auto loop_bytes = code.size() - loop_start + 1;
    code.push_back(static_cast<cpu12::Byte>(byte_values - loop_bytes));
    constexpr auto handler = isa::program(isa::opcode<isa::Opcode::inc_extended>(),
                                          std::array<cpu12::Byte, 2>{0x18, 0},
                                          isa::opcode<isa::Opcode::ldab_direct>(),
                                          std::array<cpu12::Byte, 1>{0x45},
                                          isa::opcode<isa::Opcode::ldaa_extended>(),
                                          std::array<cpu12::Byte, 2>{0x18, 0},
                                          isa::opcode<isa::Opcode::cmpa_immediate>(),
                                          std::array<cpu12::Byte, 1>{1},
                                          isa::opcode<isa::Opcode::beq_relative>(),
                                          std::array<cpu12::Byte, 1>{3},
                                          isa::opcode<isa::Opcode::stab_extended>(),
                                          std::array<cpu12::Byte, 2>{0x30, 0},
                                          isa::rti);
    image.load(0x8000, code);
    image.load(handler_entry, handler);
    image->bytes[blofeld::vi::FlashImage::byte_count - 12] =
        static_cast<cpu12::Byte>(handler_entry >> 8U);
    image->bytes[blofeld::vi::FlashImage::byte_count - 11] =
        static_cast<cpu12::Byte>(handler_entry);
    image->bytes[blofeld::vi::FlashImage::byte_count - 2] = 0x80;
    image->bytes.back() = 0;
    program = image.program();
  }
};

struct Expected {
  std::uint64_t boundary = returned, retired = initial_retired, timer = 0;
};

Expected expected(const Loop loop, const std::uint64_t tick) {
  const auto setup = prelude(loop);
  const auto start = returned + setup.cycles;
  const bool reuse = tick <= start * 10;
  Expected result{.boundary = start, .retired = initial_retired + setup.instructions};
  if (!reuse) {
    const auto elapsed = ((tick + 9) / 10) - start;
    if (loop == Loop::counted) {
      constexpr unsigned operations = 128;
      constexpr unsigned cost = 3;
      constexpr unsigned delay = operations * cost;
      constexpr unsigned period = delay + 3 + 1 + 3;
      const auto turns = (elapsed - 1) / period;
      const auto phase = (elapsed - 1) % period;
      result.boundary += turns * period;
      result.retired += turns * (operations + 3);
      if (phase < delay) {
        const auto count = (phase / cost) + 1;
        result.boundary += count * cost;
        result.retired += count;
      } else {
        const auto tail = phase - delay;
        const unsigned step = tail < 3 ? 1U : 2U + unsigned{tail >= 4};
        constexpr std::array<unsigned, 3> boundaries{3, 4, 7};
        result.boundary += delay + boundaries[step - 1];
        result.retired += operations + step;
      }
    } else if (loop == Loop::branch) {
      const auto branches = (elapsed + 2) / 3;
      result.boundary += branches * 3;
      result.retired += branches;
    } else if (loop == Loop::mixed) {
      constexpr std::array<unsigned, 3> boundaries{2, 5, 8};
      constexpr unsigned period = 8;
      const auto phase = (elapsed - 1) % period;
      const auto instruction = unsigned{phase >= boundaries[0]} + unsigned{phase >= boundaries[1]};
      const auto turns = (elapsed - 1) / period;
      result.boundary += (turns * period) + boundaries[instruction];
      result.retired += (turns * boundaries.size()) + instruction + 1;
    } else if (loop == Loop::strobe) {
      constexpr unsigned cost = 3;
      constexpr unsigned operations = 4;
      constexpr unsigned period = cost * operations;
      const auto turns = (elapsed - 1) / period;
      const auto step = (((elapsed - 1) % period) / cost) + 1;
      result.boundary += turns * period + step * cost;
      result.retired += turns * operations + step;
    } else {
      const auto access = 2U + unsigned{loop != Loop::ram} + unsigned{loop == Loop::flags};
      const auto period = access + 3;
      const auto turns = (elapsed - 1) / period;
      const bool first = (elapsed - 1) % period < access;
      result.boundary += (turns * period) + (first ? access : period);
      result.retired += (turns * 2) + (first ? 1 : 2);
    }
  }
  constexpr unsigned byte_mask = 0xff;
  result.timer = (result.boundary + (reuse ? 6 : 9) + 4 - timer_started - setup.cycles) & byte_mask;
  return result;
}

struct Source {
  blofeld::vi::PeerPort &peer;
  blofeld::vi::ExecutionTime &time;
  blofeld::vi::MidiPort &midi;
  std::uint64_t tick;
  std::chrono::milliseconds delay;
  std::promise<bool> publication;
  std::uint64_t grant = 0;
  std::uint64_t receiver = 0;
  bool woke = false;
  std::uint64_t offset = 0;

  void run(const std::stop_token &stop) {
    const std::array<std::uint64_t, 3> events{500 + offset, 501 + offset, tick};
    std::size_t next = 0;
    std::uint64_t now = 0;
    while (const auto limit = time.dsp_limit(now, stop)) {
      if (next == events.size() || limit->until <= events[next]) {
        now = limit->until;
        continue;
      }
      now = events[next];
      if (next == 2) {
        grant = limit->until;
        std::this_thread::sleep_for(delay);
        receiver = midi.time();
      }
      const bool active = next != 1;
      const bool accepted = peer.dsp_ready(active);
      const bool wake = active && time.wake_mcu(now);
      if (next == 2) {
        woke = wake;
        publication.set_value(accepted);
      }
      ++next;
    }
  }
};

void run(Fixture &fixture, const std::uint64_t tick, const std::chrono::milliseconds delay) {
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::PeerPort peer;
  blofeld::vi::ExecutionTime time;
  blofeld::vi::MidiPort midi(blofeld::vi::MidiPort::InputMode::stream);
  if (!midi.advance_input(frontier)) { throw std::runtime_error("Missing input knowledge"); }
  std::promise<blofeld::vi::ExecutionReport> completion;
  auto done = completion.get_future();
  Source source{.peer = peer,
                .time = time,
                .midi = midi,
                .tick = tick,
                .delay = delay,
                .publication = {},
                .offset = std::uint64_t{prelude(fixture.loop).cycles} * 10};
  auto posted = source.publication.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    try {
      completion.set_value(blofeld::vi::mcu::execute(*fixture.image,
                                                     *fixture.program,
                                                     shi,
                                                     reset,
                                                     stop,
                                                     nullptr,
                                                     nullptr,
                                                     &peer,
                                                     &time,
                                                     &midi));
    } catch (...) { completion.set_exception(std::current_exception()); }
  });
  std::jthread dsp([&](const std::stop_token &stop) { source.run(stop); });
  const auto guard = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (posted.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
         done.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
         std::chrono::steady_clock::now() < guard) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const bool announced = posted.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  const bool published = announced && posted.get();
  const bool extended = midi.advance_input(frontier * 2);
  const auto ended = done.wait_for(std::chrono::seconds(2));
  mcu.request_stop();
  dsp.request_stop();
  mcu.join();
  dsp.join();
  const auto result = done.get();
  if (result.exception) { std::rethrow_exception(result.exception); }
  const auto oracle = expected(fixture.loop, tick);
  const auto first = returned + prelude(fixture.loop).cycles +
                     (fixture.loop == Loop::ram || fixture.loop == Loop::mixed
                          ? 2U
                          : 3U + unsigned{fixture.loop == Loop::flags});
  const bool distant = fixture.loop != Loop::flags && fixture.loop != Loop::strobe &&
                       fixture.loop != Loop::latch && fixture.loop != Loop::input &&
                       fixture.loop != Loop::counted && tick > (first + 3) * 10;
  constexpr std::uint64_t counter_span = std::uint64_t{128 - 1} * 3 * 10;
  const bool counted = fixture.loop == Loop::counted && tick >= 5000;
  if (!published || !extended || ended != std::future_status::ready ||
      time.mcu_wait_time() != first_wait + (std::uint64_t{prelude(fixture.loop).cycles} * 10) ||
      result.instructions != oracle.retired || !result.failure ||
      result.failure->operation != blofeld::vi::Access::write ||
      result.failure->address != marker || result.failure->value != oracle.timer ||
      source.receiver > oracle.boundary * 10 ||
      (distant && (source.receiver != first * 10 || source.grant != frontier || !source.woke)) ||
      (counted && (source.grant < source.receiver + counter_span || !source.woke))) {
    throw std::runtime_error(std::format("Native horizon: loop={} tick={} delay={} timer={}/{} "
                                         "retired={}/{} receiver={} grant={}",
                                         static_cast<unsigned>(fixture.loop),
                                         tick,
                                         delay.count(),
                                         result.failure ? result.failure->value : 0,
                                         oracle.timer,
                                         result.instructions,
                                         oracle.retired,
                                         source.receiver,
                                         source.grant));
  }
}

struct NativePorts {
  blofeld::vi::PeerPort peer;
  blofeld::test::McuImage image;
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  const std::stop_token stop;
  blofeld::vi::mcu::Memory memory{*image};
  blofeld::vi::mcu::Devices devices{stop};
  blofeld::vi::mcu::Io io{memory, *image, devices, shi, reset, stop, nullptr, nullptr, &peer};
  cpu12::Io callbacks = io.callbacks();
  std::vector<cpu12::Ram> grants;
  cpu12::Core core{cpu12::Memory::external(), &callbacks};

  explicit NativePorts(const std::span<const cpu12::CodeSpan> code) {
    if (code.empty()) { throw std::runtime_error("Missing native port fixture code"); }
    for (const auto span : code) { image.load(span.address, span.words); }
    const auto program = image.program();
    grants = memory.regions();
    core.set_memory(cpu12::Memory::mapped(grants));
    io.bind(core);
    const auto admitted = program->admit(memory, code.front().address);
    program->install(core, admitted);
  }

  void write(const cpu12::ByteWrite access) const {
    if (!callbacks.write(callbacks.owner, access)) {
      throw std::runtime_error("Native port setup");
    }
  }
};

void input_source(blofeld::vi::PeerPort &peer,
                  blofeld::vi::ExecutionTime &time,
                  const std::span<const blofeld::vi::PeerPort::Sample> samples,
                  const std::stop_token &stop) {
  std::size_t next = 0;
  std::uint64_t now = 0;
  while (const auto limit = time.dsp_limit(now, stop)) {
    if (next < samples.size() && next < limit->until) {
      now = next;
      peer.dsp_data(samples[next++]);
    } else {
      now = limit->until;
    }
  }
}

void settled_reads(const bool rmw, const cpu12::Byte known) {
  constexpr cpu12::Address entry = 0x8000;
  constexpr cpu12::Byte current = 0xa5;
  constexpr cpu12::Byte future = 0x5a;
  constexpr cpu12::Byte masks = 0xd0;
  constexpr unsigned byte_bits = 8;
  constexpr auto increment =
      isa::program(isa::opcode<isa::Opcode::inc_extended>(), std::array<cpu12::Byte, 2>{2, 0x60});
  constexpr auto load =
      isa::program(isa::opcode<isa::Opcode::ldaa_extended>(), std::array<cpu12::Byte, 2>{2, 0x60});
  const auto bytes = rmw ? increment : load;
  const std::array code{cpu12::CodeSpan{.address = entry, .words = bytes}};
  const std::array samples{
      blofeld::vi::PeerPort::Sample{.mask = all_bits, .known = all_bits, .value = current},
      blofeld::vi::PeerPort::Sample{.mask = all_bits, .known = known, .value = future}};
  NativePorts fixture(code);
  blofeld::vi::ExecutionTime time;
  fixture.devices.time = &time;
  std::jthread dsp(
      [&](const std::stop_token &stop) { input_source(fixture.peer, time, samples, stop); });
  bool exact = true;
  for (const auto value : {current, future}) {
    fixture.core.set_registers({.pc = entry, .ccr = masks});
    const auto before = fixture.core.state();
    const auto progress = fixture.core.progress();
    const auto result = fixture.core.step();
    if (value == future && known != all_bits) {
      exact = exact && result.fault == cpu12::Fault::memory_read && result.retired == 0 &&
              result.address == port_q && result.pc == entry &&
              fixture.core.registers() == before.registers && fixture.core.progress() == progress;
    } else {
      const auto output = rmw ? unsigned{fixture.peer.mcu_data().value}
                              : unsigned{fixture.core.registers().d} >> byte_bits;
      exact = exact && result.fault == cpu12::Fault::none && result.retired == 1 &&
              output == unsigned{value} + unsigned{rmw} &&
              fixture.core.progress().estimated_cycles == progress.estimated_cycles + (rmw ? 4 : 3);
    }
  }
  dsp.request_stop();
  dsp.join();
  if (!exact) {
    throw std::runtime_error("Native input/RMW sees same-tick source data, excludes future data "
                             "and preserves unknown faults");
  }
}

void local_outputs() {
  constexpr cpu12::Address entry = 0x8000;
  constexpr cpu12::Address other = 0x8100;
  constexpr std::array ports{cpu12::Address{0x240}, cpu12::Address{0x270}};
  constexpr unsigned byte_values = 256;
  constexpr unsigned byte_bits = 8;
  constexpr unsigned cycles = 3;
  constexpr auto led =
      isa::program(isa::opcode<isa::Opcode::staa_extended>(), std::array<cpu12::Byte, 2>{2, 0x40});
  constexpr auto scan =
      isa::program(isa::opcode<isa::Opcode::staa_extended>(), std::array<cpu12::Byte, 2>{2, 0x70});
  const std::array code{cpu12::CodeSpan{.address = entry, .words = led},
                        cpu12::CodeSpan{.address = other, .words = scan}};
  NativePorts fixture(code);
  fixture.core.isolate_accesses(std::array{
      cpu12::LocalAccess{.first = ports[0], .size = 1, .role = cpu12::LocalAccess::Role::write},
      cpu12::LocalAccess{.first = ports[1], .size = 1, .role = cpu12::LocalAccess::Role::write}});
  for (const auto port : ports) {
    if (!fixture.callbacks.write(
            fixture.callbacks.owner,
            {.address = static_cast<cpu12::Address>(port + 2), .value = all_bits})) {
      throw std::runtime_error("Local output directions");
    }
    const auto pc = port == ports[0] ? entry : other;
    const auto mask = port == ports[0] ? 0x0fU : 0xffU;
    for (unsigned value = 0; value < byte_values; ++value) {
      fixture.core.set_registers({.pc = pc, .d = static_cast<cpu12::Address>(value << byte_bits)});
      if (fixture.core.channel_prefix() != 0 ||
          fixture.core.channel_prefix(cpu12::PrefixScope::foreground) != cycles) {
        throw std::runtime_error("Unconfigured unmasked IRQ paths remain uncertified");
      }
      fixture.core.set_registers({.pc = pc,
                                  .d = static_cast<cpu12::Address>(value << byte_bits),
                                  .ccr = interrupt_masks | 0x40});
      const auto time = fixture.core.progress();
      const auto data = fixture.peer.mcu_data();
      cpu12::Byte readback = 0;
      if (fixture.core.channel_prefix() != cycles ||
          fixture.core.run_until(time.estimated_cycles + cycles).retired != 1 ||
          !fixture.callbacks.read(fixture.callbacks.owner, port, readback) ||
          readback != (value & mask) || fixture.peer.mcu_data() != data ||
          fixture.peer.dsp_interrupt(false) || !fixture.io.complete_events() ||
          fixture.devices.clock.cycles() != time.estimated_cycles + cycles ||
          fixture.devices.clock.bus_period() != 10) {
        throw std::runtime_error(
            std::format("Local output port=${:04x} value={} events={} foreground={} CCR=${:02x} "
                        "cycles={} clock={} period={} readback={}",
                        port,
                        value,
                        fixture.core.channel_prefix(),
                        fixture.core.channel_prefix(cpu12::PrefixScope::foreground),
                        fixture.core.registers().ccr,
                        fixture.core.progress().estimated_cycles,
                        fixture.devices.clock.cycles(),
                        fixture.devices.clock.bus_period(),
                        readback));
      }
    }
  }
}

void neutral_stores(const cpu12::Byte direction) {
  constexpr cpu12::Address entry = 0x8000;
  constexpr cpu12::Address port_j = 0x250;
  constexpr unsigned byte_values = 256;
  constexpr unsigned byte_bits = 8;
  constexpr unsigned cycles = 3;
  constexpr auto store =
      isa::program(isa::opcode<isa::Opcode::staa_extended>(), std::array<cpu12::Byte, 2>{2, 0x50});
  const std::array code{cpu12::CodeSpan{.address = entry, .words = store}};
  NativePorts fixture(code);
  fixture.write({.address = port_q, .value = output_data});
  fixture.write({.address = direction_q, .value = all_bits});
  for (unsigned value = 0; value < byte_values; ++value) {
    fixture.write({.address = port_j, .value = 1});
    fixture.write({.address = port_j + 2, .value = direction});
    fixture.core.set_registers({.pc = entry,
                                .d = static_cast<cpu12::Address>(value << byte_bits),
                                .ccr = interrupt_masks});
    const auto before = fixture.core.state();
    const auto time = fixture.core.progress();
    const auto data = fixture.peer.mcu_data();
    const auto clock = fixture.devices.clock.ticks();
    const auto writes = fixture.io.isolated_writes();
    const bool neutral = direction == 0 || (value & 1) != 0;
    if (!writes ||
        fixture.core.channel_prefix(cpu12::PrefixScope::foreground,
                                    {.ranges = {}, .writes = *writes}) != (neutral ? cycles : 0) ||
        fixture.core.channel_prefix() != 0 || fixture.core.state() != before ||
        fixture.core.progress() != time || fixture.peer.mcu_data() != data ||
        fixture.peer.dsp_interrupt(false) || fixture.devices.clock.ticks() != clock) {
      throw std::runtime_error("Only a proved inactive strobe preserves the channel");
    }
    const auto result = fixture.core.run_until(time.estimated_cycles + cycles);
    if (result.fault != cpu12::Fault::none || result.retired != 1 ||
        fixture.core.registers().pc != entry + store.size() ||
        fixture.core.progress().estimated_cycles != time.estimated_cycles + cycles ||
        fixture.peer.dsp_interrupt(false).has_value() == neutral ||
        fixture.peer.mcu_data() != data || !fixture.io.complete_events() ||
        (!neutral && fixture.io.isolated_writes())) {
      throw std::runtime_error("Every native port write retains its actual request/data effects");
    }
  }
  fixture.peer.dsp_data({.mask = all_bits, .known = all_bits, .value = 0x0d});
  if (!fixture.peer.dsp_ready(true) || !fixture.io.isolated_writes()) {
    throw std::runtime_error("Incoming report state does not revoke an inactive outgoing strobe");
  }
}

void retained_reads(NativePorts &fixture,
                    const cpu12::Address entry,
                    const std::span<const cpu12::Byte> expected) {
  constexpr unsigned cycles = 3;
  constexpr unsigned byte_bits = 8;
  constexpr cpu12::Byte masks = 0xd0;
  fixture.write({.address = direction_q, .value = 0});
  static_cast<void>(fixture.peer.sample_mcu_data(fixture.devices.clock.ticks()));
  for (const auto value : expected) {
    fixture.core.set_registers({.pc = entry, .ccr = masks});
    const auto time = fixture.core.progress();
    const auto result = fixture.core.run_until(time.estimated_cycles + cycles);
    if (fixture.io.isolated_reads() || result.fault != cpu12::Fault::none || result.retired != 1 ||
        fixture.core.registers().d != (unsigned{value} << byte_bits) ||
        !fixture.io.complete_events()) {
      throw std::runtime_error("Own-latch reads never consume or reorder retained response bytes");
    }
  }
  fixture.peer.dsp_data({.mask = all_bits, .known = 0, .value = all_bits});
  fixture.core.set_registers({.pc = entry, .ccr = masks});
  const auto before = fixture.core.state();
  const auto time = fixture.core.progress();
  const auto result = fixture.core.run_until(time.estimated_cycles + cycles);
  if (result.fault != cpu12::Fault::memory_read || result.retired != 0 ||
      result.address != port_q || result.pc != entry ||
      fixture.core.registers() != before.registers || fixture.core.progress() != time) {
    throw std::runtime_error("A real unknown input after the reply retains its exact native fault");
  }
}

void latch_reads() {
  constexpr cpu12::Address entry = 0x8000;
  constexpr cpu12::Address pin = 0x8100;
  constexpr cpu12::Address word_entry = 0x8200;
  constexpr cpu12::Address rmw = 0x8300;
  constexpr cpu12::Byte masks = 0xd0;
  constexpr unsigned byte_bits = 8;
  constexpr unsigned cycles = 3;
  constexpr std::array<cpu12::Byte, 16> reply{
      0, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
  constexpr auto data =
      isa::program(isa::opcode<isa::Opcode::ldaa_extended>(), std::array<cpu12::Byte, 2>{2, 0x60});
  constexpr auto input =
      isa::program(isa::opcode<isa::Opcode::ldab_extended>(), std::array<cpu12::Byte, 2>{2, 0x61});
  constexpr auto word_read =
      isa::program(isa::opcode<isa::Opcode::ldd_extended>(), std::array<cpu12::Byte, 2>{2, 0x60});
  constexpr auto read_modify_write =
      isa::program(isa::opcode<isa::Opcode::inc_extended>(), std::array<cpu12::Byte, 2>{2, 0x60});
  const std::array code{cpu12::CodeSpan{.address = entry, .words = data},
                        cpu12::CodeSpan{.address = pin, .words = input},
                        cpu12::CodeSpan{.address = word_entry, .words = word_read},
                        cpu12::CodeSpan{.address = rmw, .words = read_modify_write}};
  NativePorts fixture(code);
  const auto publish = [&](const cpu12::ByteWrite access) {
    fixture.write(access);
    static_cast<void>(fixture.peer.sample_mcu_data(fixture.devices.clock.ticks()));
  };
  fixture.peer.dsp_data({.mask = all_bits, .known = all_bits, .value = 0x0d});
  if (!fixture.peer.dsp_ready(true) || !fixture.peer.dsp_ready(false) ||
      !fixture.peer.mcu_strobe(true) || !fixture.peer.mcu_strobe(false)) {
    throw std::runtime_error("Announce the actual retained reply");
  }
  for (const auto value : reply) {
    if (!fixture.peer.dsp_write({.mask = all_bits, .known = all_bits, .value = value}, 0)) {
      throw std::runtime_error("Produce every real reply byte before reading any");
    }
  }
  publish({.address = direction_q, .value = all_bits});
  for (unsigned value = 0; value <= all_bits; ++value) {
    publish({.address = port_q, .value = static_cast<cpu12::Byte>(value)});
    for (const auto known : {cpu12::Byte{0}, all_bits}) {
      fixture.peer.dsp_data(
          {.mask = all_bits, .known = known, .value = static_cast<cpu12::Byte>(value ^ all_bits)});
      fixture.core.set_registers({.pc = entry, .ccr = masks});
      const auto before = fixture.core.state();
      const auto time = fixture.core.progress();
      const auto output = fixture.peer.mcu_data();
      const auto read = fixture.io.isolated_reads();
      if (!read ||
          fixture.core.channel_prefix(cpu12::PrefixScope::foreground,
                                      {.ranges = std::span(&*read, 1), .writes = {}}) != cycles ||
          fixture.core.channel_prefix() != 0 || fixture.core.state() != before ||
          fixture.core.progress() != time || fixture.peer.mcu_data() != output) {
        throw std::runtime_error("Only the complete output latch is a total local read");
      }
      const auto result = fixture.core.run_until(time.estimated_cycles + cycles);
      if (result.fault != cpu12::Fault::none || result.retired != 1 ||
          fixture.core.registers().d != (value << byte_bits) || fixture.peer.mcu_data() != output ||
          !fixture.io.complete_events()) {
        throw std::runtime_error(
            "Native PTQ reads preserve all own bytes despite unknown driven inputs");
      }
    }
  }
  const auto read = fixture.io.isolated_reads();
  for (const auto pc : {pin, word_entry, rmw}) {
    fixture.core.set_registers({.pc = pc, .ccr = masks});
    if (!read || fixture.core.channel_prefix(cpu12::PrefixScope::foreground,
                                             {.ranges = std::span(&*read, 1), .writes = {}}) != 0) {
      throw std::runtime_error("PTIQ, spanning words and RMW never inherit the latch-read proof");
    }
  }
  for (unsigned direction = 0; direction <= all_bits; ++direction) {
    publish({.address = direction_q, .value = static_cast<cpu12::Byte>(direction)});
    if (fixture.io.isolated_reads().has_value() != (direction == all_bits)) {
      throw std::runtime_error("Every partial ownership pattern revokes the local read");
    }
  }
  retained_reads(fixture, entry, reply);
}

constexpr dsp56300::Word native_boot = blofeld::vi::dsp::BootRom::main_first;
constexpr dsp56300::Word native_port = 0xffff9d;
constexpr dsp56300::Word native_ready = 0x200;
constexpr std::uint64_t native_return = 790;

constexpr unsigned native_first = 690, native_withdrawn = 700, repeat_cycles = 5;
constexpr std::uint64_t native_top = 3125, native_bottom = 4263;

constexpr auto native_peer_code() {
  constexpr dsp56300::Word store_x0 = 0x4c7000;
  constexpr dsp56300::Word store_x1 = 0x4d7000;
  constexpr dsp56300::Word jump = 0x0af080;
  constexpr dsp56300::Word repeat_r0 = 0x06d020;
  constexpr dsp56300::Word repeat_r1 = 0x06d120;
  constexpr dsp56300::Word repeat_r2 = 0x06d220;
  return std::array{repeat_r0,
                    dsp56300::Word{0},
                    store_x0,
                    native_port,
                    repeat_r1,
                    dsp56300::Word{0},
                    store_x1,
                    native_port,
                    repeat_r2,
                    dsp56300::Word{0},
                    store_x0,
                    native_port,
                    jump,
                    native_boot + 12};
}

blofeld::vi::dsp::Program native_peer_program() {
  constexpr auto words = native_peer_code();
  const std::array spans{dsp56300::CodeSpan{.address = native_boot, .words = words}};
  constexpr std::array entries{native_boot};
  const std::array variants{
      dsp56300::CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array choices{
      blofeld::vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  return blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices, {}, {}, {});
}

void native_peer(const blofeld::vi::dsp::Program &program,
                 const unsigned cycle,
                 const std::chrono::milliseconds delay) {
  namespace vi = blofeld::vi;
  constexpr auto boot = native_boot;
  constexpr auto port_e = native_port;
  constexpr auto ready = native_ready;
  constexpr unsigned word_bits = 24;
  constexpr auto words = native_peer_code();
  Fixture fixture(Loop::branch);
  vi::Shi shi;
  vi::DspReset reset;
  vi::PeerPort peer;
  vi::ExecutionTime time;
  vi::MidiPort midi(vi::MidiPort::InputMode::stream);
  if (!midi.advance_input(frontier)) { throw std::runtime_error("Known native-peer silence"); }
  std::stop_source cancellation;
  const auto stop = cancellation.get_token();
  vi::dsp::Io io({.main = words, .eeprom = {}},
                 shi,
                 reset,
                 stop,
                 nullptr,
                 {},
                 &peer,
                 &time,
                 nullptr,
                 nullptr,
                 &program);
  dsp56300::Core core(dsp56300::Memory::external(), &io);
  io.bind(core, 0);
  core.set_registers({.x = std::uint64_t{ready} << word_bits,
                      .r = {native_first - repeat_cycles,
                            native_withdrawn - native_first - 2 - repeat_cycles,
                            cycle - native_withdrawn - 2 - repeat_cycles},
                      .pc = boot});
  if (!io.write(dsp56300::Space::y, port_e, boot, ready) ||
      !io.write(dsp56300::Space::y, port_e + 1, boot, ready) ||
      !io.write(dsp56300::Space::y, port_e + 2, boot, 0)) {
    throw std::runtime_error("Configure actual native E9 output");
  }
  vi::ExecutionReport source_report;
  if (!io.complete_events(source_report)) { throw std::runtime_error("Prepare native source"); }
  io.prepare_code(source_report);
  std::promise<vi::ExecutionReport> completion;
  auto done = completion.get_future();
  std::exception_ptr failure;
  std::jthread source([&] {
    try {
      std::this_thread::sleep_for(delay);
      while (!stop.stop_requested()) {
        const auto result = io.run();
        if (result.fault.kind != dsp56300::FaultKind::none || !io.complete_events(source_report)) {
          throw std::runtime_error("Actual native E9 source failed");
        }
      }
    } catch (...) {
      failure = std::current_exception();
      cancellation.request_stop();
    }
  });
  std::jthread receiver([&] {
    try {
      completion.set_value(vi::mcu::execute(*fixture.image,
                                            *fixture.program,
                                            shi,
                                            reset,
                                            stop,
                                            nullptr,
                                            nullptr,
                                            &peer,
                                            &time,
                                            &midi));
    } catch (...) { completion.set_exception(std::current_exception()); }
  });
  const auto ended = done.wait_for(std::chrono::seconds(2));
  cancellation.request_stop();
  receiver.join();
  source.join();
  if (failure) { std::rethrow_exception(failure); }
  const auto result = done.get();
  if (result.exception) { std::rethrow_exception(result.exception); }
  // The first edge places the native RTI return at790 instead of780.
  constexpr auto shifted_cycles = (native_return - (returned * 10)) / 10;
  const auto tick = ((std::uint64_t{cycle} * native_top) + native_bottom - 1) / native_bottom;
  auto oracle = expected(Loop::branch, tick - (shifted_cycles * 10));
  oracle.timer = (oracle.timer + shifted_cycles) & all_bits;
  if (ended != std::future_status::ready || time.mcu_wait_time() != first_wait ||
      result.instructions != oracle.retired || !result.failure ||
      result.failure->address != marker || result.failure->operation != vi::Access::write ||
      result.failure->value != oracle.timer) {
    throw std::runtime_error(
        std::format("Two native owners retain fractional IRQ time: "
                    "event={} delay={} timer={}/{} retired={}/{} DSP cycles={}",
                    cycle,
                    delay.count(),
                    result.failure ? result.failure->value : 0,
                    oracle.timer,
                    result.instructions,
                    oracle.retired,
                    core.progress().estimated_cycles));
  }
}

void native_clock_return() {
  constexpr cpu12::Address clock_handler = 0x8100;
  constexpr cpu12::Address peer_handler = 0x8200;
  constexpr cpu12::Address helper_entry = 0x8300;
  constexpr std::uint64_t peer_tick = 6000;
  constexpr std::uint64_t expected_timer = 86;
  constexpr std::uint64_t expected_wait = 230;
  constexpr std::uint64_t second_wait = 5660;
  constexpr std::uint64_t expected_instructions = 18;
  constexpr auto main = isa::program(isa::opcode<isa::Opcode::lds_immediate>(),
                                     std::array<cpu12::Byte, 2>{0x1f, 0xff},
                                     isa::opcode<isa::Opcode::ldab_immediate>(),
                                     std::array<cpu12::Byte, 1>{0x80},
                                     isa::opcode<isa::Opcode::stab_direct>(),
                                     std::array<cpu12::Byte, 1>{0x46},
                                     isa::opcode<isa::Opcode::ldaa_immediate>(),
                                     std::array<cpu12::Byte, 1>{0x10},
                                     isa::opcode<isa::Opcode::staa_direct>(),
                                     std::array<cpu12::Byte, 1>{0x3b},
                                     isa::opcode<isa::Opcode::stab_direct>(),
                                     std::array<cpu12::Byte, 1>{0x38},
                                     isa::opcode<isa::Opcode::andcc_immediate>(),
                                     std::array<cpu12::Byte, 1>{0xaf},
                                     isa::wai,
                                     isa::wai);
  constexpr auto clock = isa::program(isa::opcode<isa::Opcode::jsr_extended>(),
                                      std::array<cpu12::Byte, 2>{0x83, 0},
                                      isa::opcode<isa::Opcode::bset_direct>(),
                                      std::array<cpu12::Byte, 2>{0x37, 0x80},
                                      isa::rti);
  constexpr auto helper = isa::program(isa::opcode<isa::Opcode::leas_indexed>(),
                                       std::array<cpu12::Byte, 1>{0x9e},
                                       isa::pshd,
                                       isa::puld,
                                       isa::opcode<isa::Opcode::leas_indexed>(),
                                       std::array<cpu12::Byte, 1>{0x82},
                                       isa::rts);
  constexpr auto handler = isa::program(isa::opcode<isa::Opcode::ldab_direct>(),
                                        std::array<cpu12::Byte, 1>{0x45},
                                        isa::opcode<isa::Opcode::stab_extended>(),
                                        std::array<cpu12::Byte, 2>{0x30, 0});
  blofeld::test::McuImage image;
  image.load(0x8000, main);
  image.load(clock_handler, clock);
  image.load(peer_handler, handler);
  image.load(helper_entry, helper);
  constexpr auto bytes = blofeld::vi::FlashImage::byte_count;
  image->bytes[bytes - 16] = static_cast<cpu12::Byte>(clock_handler >> 8U);
  image->bytes[bytes - 15] = static_cast<cpu12::Byte>(clock_handler);
  image->bytes[bytes - 12] = static_cast<cpu12::Byte>(peer_handler >> 8U);
  image->bytes[bytes - 11] = static_cast<cpu12::Byte>(peer_handler);
  image->bytes[bytes - 2] = static_cast<cpu12::Byte>(0x8000U >> 8U);
  image->bytes.back() = 0;
  const auto program = image.program();
  blofeld::vi::Shi shi;
  blofeld::vi::DspReset reset;
  blofeld::vi::PeerPort peer;
  blofeld::vi::ExecutionTime time;
  blofeld::vi::MidiPort midi(blofeld::vi::MidiPort::InputMode::stream);
  if (!midi.advance_input(frontier)) { throw std::runtime_error("Missing clock input knowledge"); }
  std::promise<blofeld::vi::ExecutionReport> completion;
  auto done = completion.get_future();
  std::jthread mcu([&](const std::stop_token &stop) {
    try {
      completion.set_value(blofeld::vi::mcu::execute(
          *image, *program, shi, reset, stop, nullptr, nullptr, &peer, &time, &midi));
    } catch (...) { completion.set_exception(std::current_exception()); }
  });
  bool posted = false;
  std::uint64_t receiver = 0;
  std::jthread dsp([&](const std::stop_token &stop) {
    std::uint64_t now = 0;
    while (const auto limit = time.dsp_limit(now, stop)) {
      if (!posted && limit->until > peer_tick) {
        now = peer_tick;
        receiver = midi.time();
        posted = peer.dsp_ready(true);
        static_cast<void>(time.wake_mcu(now));
      } else {
        now = limit->until;
      }
    }
  });
  const auto status = done.wait_for(std::chrono::seconds(2));
  mcu.request_stop();
  dsp.request_stop();
  mcu.join();
  dsp.join();
  const auto report = done.get();
  if (report.exception) { std::rethrow_exception(report.exception); }
  if (status != std::future_status::ready || !posted || time.mcu_wait_time() != expected_wait ||
      receiver != second_wait || report.instructions != expected_instructions || !report.failure ||
      report.failure->operation != blofeld::vi::Access::write ||
      report.failure->address != marker || report.failure->value != expected_timer) {
    throw std::runtime_error(
        "Native clock IRQ must acknowledge W1C, return and wake at real XIRQ time");
  }
}
}

int main() try {
  const auto program = native_peer_program();
  for (const unsigned cycle : {1077U, 1078U, 8526U, 8527U}) {
    for (const auto delay : {std::chrono::milliseconds(0), std::chrono::milliseconds(3)}) {
      native_peer(program, cycle, delay);
    }
  }
  native_clock_return();
  for (const bool rmw : {false, true}) {
    for (const auto known : {cpu12::Byte{0}, all_bits}) { settled_reads(rmw, known); }
  }
  local_outputs();
  latch_reads();
  neutral_stores(0);
  neutral_stores(1);
  for (const auto loop : {Loop::branch,
                          Loop::ram,
                          Loop::timer,
                          Loop::mixed,
                          Loop::flags,
                          Loop::strobe,
                          Loop::latch,
                          Loop::input,
                          Loop::counted}) {
    Fixture fixture(loop);
    for (const std::uint64_t tick : {780U,
                                     781U,
                                     799U,
                                     800U,
                                     801U,
                                     809U,
                                     810U,
                                     811U,
                                     829U,
                                     830U,
                                     831U,
                                     839U,
                                     840U,
                                     841U,
                                     5000U,
                                     50000U,
                                     50001U}) {
      for (const auto delay : {std::chrono::milliseconds(0), std::chrono::milliseconds(3)}) {
        run(fixture, tick + (std::uint64_t{prelude(loop).cycles} * 10), delay);
      }
    }
  }
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
