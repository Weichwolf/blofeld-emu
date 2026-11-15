// SPDX-License-Identifier: GPL-3.0-only
#include "aot/dsp.hpp"
#include "dsp/boot_rom.hpp"
#include "dsp/clock.hpp"
#include "dsp/interrupts.hpp"
#include "dsp/io.hpp"
#include "dsp/program.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "isa.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <exception>
#include <format>
#include <future>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

namespace {
using namespace dsp56300;
namespace vi = blofeld::vi;
constexpr Word boot = vi::dsp::BootRom::main_first, vector = 0x54;
constexpr Word nop = static_cast<Word>(isa::Opcode::nop);
constexpr std::array code{
    static_cast<Word>(isa::Opcode::wait), nop, static_cast<Word>(isa::Opcode::stop)};
constexpr std::array vectors{nop, nop};
constexpr std::uint64_t forecast_cycles = 32, input_until = 12;
constexpr auto forecast_code = [] {
  std::array<Word, 65> result{};
  result.back() = static_cast<Word>(isa::Opcode::wait);
  return result;
}();
enum class ForecastInput : std::uint8_t { absent, live, stream };

vi::dsp::Program prepare(const CodeSpan source, const std::span<const Word> entries) {
  const std::array spans{source};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array choices{vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  return blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices, {}, {}, {});
}

bool settle(vi::ExecutionTime &time, const std::uint64_t ticks) {
  std::promise<bool> completed;
  auto settled = completed.get_future();
  std::jthread receiver(
      [&](const std::stop_token &stop) { completed.set_value(time.settle_dsp(ticks, stop)); });
  const auto ready = settled.wait_for(std::chrono::seconds(2));
  receiver.request_stop();
  receiver.join();
  return ready == std::future_status::ready && settled.get();
}

void forecast_boundary(const vi::dsp::Program &program, const ForecastInput input) {
  vi::Shi shi;
  vi::DspReset reset;
  vi::PeerPort peer;
  vi::ExecutionTime time;
  vi::MidiPort midi(input == ForecastInput::stream ? vi::MidiPort::InputMode::stream
                                                   : vi::MidiPort::InputMode::live);
  if (input == ForecastInput::stream && !midi.advance_input(input_until)) {
    throw std::runtime_error("Establish the actual exclusive input horizon");
  }
  const std::stop_token cancellation;
  vi::dsp::Io io({.main = forecast_code, .eeprom = {}},
                 shi,
                 reset,
                 cancellation,
                 nullptr,
                 {},
                 &peer,
                 &time,
                 input == ForecastInput::absent ? nullptr : &midi,
                 nullptr,
                 &program);
  Core core(Memory::external(), &io);
  io.bind(core, 0);
  core.set_registers({.pc = boot});
  constexpr Word control = 0xffff8f;
  constexpr Word load = 0xffff8e;
  constexpr Word compare = 0xffff8d;
  constexpr Word timer_priority = 0x100;
  if (!io.write(Space::program, vector, boot, nop) ||
      !io.write(Space::program, vector + 1, boot, nop) || !io.write(Space::x, load, boot, 0) ||
      !io.write(Space::x, compare, boot, 15) || !io.write(Space::x, control, boot, 7) ||
      !io.write(Space::x, vi::dsp::Interrupts::peripheral_address, boot, timer_priority)) {
    throw std::runtime_error("Configure the genuine future timer event");
  }
  vi::ExecutionReport report;
  if (!io.complete_events(report)) { throw std::runtime_error("Prepare native forecast devices"); }
  io.prepare_code(report);
  std::promise<std::optional<std::uint64_t>> completion;
  auto receiver = completion.get_future();
  constexpr std::uint64_t receiver_until = 1000;
  std::jthread mcu([&](const std::stop_token &stop) {
    completion.set_value(
        time.idle_mcu(0, receiver_until, stop, vi::ExecutionTime::McuWork::running));
  });
  const auto executed = io.run();
  const bool completed = io.complete_events(report);
  const auto ready =
      receiver.wait_for(input == ForecastInput::live ? std::chrono::milliseconds(10)
                                                     : std::chrono::milliseconds(2000));
  mcu.request_stop();
  mcu.join();
  const auto granted = receiver.get();
  constexpr std::uint64_t top = 3125;
  constexpr std::uint64_t bottom = 4263;
  const auto expected = input == ForecastInput::stream
                            ? input_until
                            : ((forecast_cycles * top) + bottom - 1) / bottom;
  const auto steps =
      input == ForecastInput::stream ? ((input_until * bottom) + top - 1) / top : forecast_cycles;
  const bool active = input != ForecastInput::live;
  if (!completed || executed.fault.kind != FaultKind::none || executed.retired != steps ||
      core.progress().estimated_cycles != steps || core.registers().pc != boot + steps ||
      (active && (ready != std::future_status::ready || granted != expected)) ||
      (!active && (ready != std::future_status::timeout || granted))) {
    throw std::runtime_error(std::format("Native source lookahead retains timer/input horizons: "
                                         "input={} grant={} expected={} steps={}/{}",
                                         static_cast<unsigned>(input),
                                         granted.value_or(0),
                                         expected,
                                         executed.retired,
                                         steps));
  }
}

void forecast_boundaries() {
  const std::array spans{CodeSpan{.address = boot, .words = forecast_code},
                         CodeSpan{.address = vector, .words = vectors}};
  constexpr std::array entries{boot};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array choices{vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  constexpr std::array interrupts{InterruptRequest{.vector = vector, .level = 0}};
  const auto program =
      blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices, {}, interrupts, {});
  for (const auto input : {ForecastInput::absent, ForecastInput::live, ForecastInput::stream}) {
    forecast_boundary(program, input);
  }
}

struct FractionalCase {
  unsigned padding;
  Word control;
};

void fractional_port_boundary(const FractionalCase input) {
  const auto padding = input.padding;
  const auto control = input.control;
  constexpr Word data = 0xffffbd;
  constexpr Word direction = 0xffffbe;
  constexpr Word function = 0xffffbf;
  constexpr Word byte_mask = 0xff;
  constexpr Word payload = 0x5a;
  constexpr Word absolute = 0x30;
  constexpr Word store = static_cast<Word>(isa::Opcode::move_x_effective) | (absolute << 8U);
  std::vector<Word> words(padding, nop);
  constexpr Word jump = static_cast<Word>(isa::Opcode::jmp_effective) | (absolute << 8U);
  words.insert(words.end(), {store, data, jump, boot + padding + 2});
  const std::array entries{boot, boot + padding, boot + padding + 2};
  const auto program = prepare({.address = boot, .words = words}, entries);
  vi::Shi shi;
  vi::DspReset reset;
  vi::PeerPort peer;
  vi::ExecutionTime time;
  const std::stop_source cancellation;
  const auto token = cancellation.get_token();
  vi::dsp::Io io({.main = words, .eeprom = {}},
                 shi,
                 reset,
                 token,
                 nullptr,
                 {},
                 &peer,
                 &time,
                 nullptr,
                 nullptr,
                 &program);
  Core core(Memory::external(), &io);
  io.bind(core, 0);
  core.set_registers({.x = payload, .pc = boot});
  if (!io.write(Space::x, data, boot, 0) || !io.write(Space::x, direction, boot, byte_mask) ||
      !io.write(Space::x, function, boot, 0) ||
      !io.write(Space::x, vi::dsp::Clock::address, boot, control)) {
    throw std::runtime_error("Configure actual driven DSP data before native execution");
  }
  vi::ExecutionReport report;
  if (!io.complete_events(report)) { throw std::runtime_error("Prepare actual DSP devices"); }
  io.prepare_code(report);
  constexpr std::uint64_t reference_top = 3125;
  constexpr std::uint64_t reference_bottom = 588;
  vi::dsp::Clock clock;
  if (!clock.write(control)) { throw std::runtime_error("Actual valid test PCTL"); }
  const auto ratio = clock.ratio();
  const auto numerator = std::uint64_t{padding} * reference_top * ratio.denominator;
  const auto denominator = reference_bottom * ratio.numerator;
  const auto now = numerator / denominator;
  if (numerator <= now * denominator || numerator >= (now + 1) * denominator ||
      !clock.advance(padding) || clock.ticks() != now || !clock.fractional()) {
    throw std::runtime_error("The real port-store start is strictly between these MCU ticks");
  }
  if (!time.publish_mcu(now)) { throw std::runtime_error("Initial actual MCU time"); }
  if (now != 0) {
    const auto first = io.run();
    if (first.retired != padding || first.progress.estimated_cycles != padding ||
        first.fault.kind != FaultKind::none || peer.dsp_data().value != 0) {
      throw std::runtime_error(
          "Two native NOPs stop at the real fractional boundary before the port store");
    }
  }
  std::exception_ptr failure;
  std::jthread source([&] {
    try {
      while (!token.stop_requested()) {
        const auto result = io.run();
        if (result.fault.kind != FaultKind::none || !io.complete_events(report)) {
          throw std::runtime_error("Actual native fractional-port source failed");
        }
      }
    } catch (...) { failure = std::current_exception(); }
  });
  const bool ready = settle(time, now);
  const auto observed = peer.dsp_data();
  const bool later = settle(time, now + 2);
  const auto delivered = peer.dsp_data();
  cancellation.request_stop();
  source.join();
  if (failure) { std::rethrow_exception(failure); }
  if (!ready || !later || observed.mask != byte_mask || observed.value != 0 ||
      delivered.mask != byte_mask || delivered.value != payload) {
    throw std::runtime_error(std::format("Current MCU access cannot observe a later fractional DSP "
                                         "write: padding={} MCU={} data={:x}",
                                         padding,
                                         now,
                                         observed.value));
  }
}

void quiescent_boundary(const bool delayed_input) {
  constexpr std::array entries{boot, boot + 1, boot + 2};
  const auto program = prepare({.address = boot, .words = code}, entries);
  vi::Shi shi;
  vi::DspReset reset;
  vi::PeerPort peer;
  vi::ExecutionTime time;
  vi::MidiPort midi(vi::MidiPort::InputMode::stream);
  const std::stop_source cancellation;
  const auto token = cancellation.get_token();
  vi::dsp::Io io({.main = code, .eeprom = {}},
                 shi,
                 reset,
                 token,
                 nullptr,
                 {},
                 &peer,
                 &time,
                 delayed_input ? &midi : nullptr,
                 nullptr,
                 &program);
  Core core(Memory::external(), &io);
  io.bind(core, 0);
  core.set_registers({.pc = boot});
  vi::ExecutionReport report;
  if (!io.complete_events(report)) { throw std::runtime_error("Prepare inactive DSP peripherals"); }
  io.prepare_code(report);
  constexpr std::uint64_t cycles = 4263;
  vi::dsp::Clock clock;
  if (!clock.advance(cycles) || clock.fractional() || !time.publish_mcu(clock.ticks())) {
    throw std::runtime_error("The waiting boundary has exact integer phase");
  }
  if (delayed_input && !midi.advance_input(clock.ticks())) {
    throw std::runtime_error("Declare silence strictly before the source instant");
  }
  const auto waited = io.run();
  if (waited.retired != 1 || waited.state != ExecutionState::waiting ||
      waited.fault.kind != FaultKind::none || !io.complete_events(report)) {
    throw std::runtime_error("Execute the real WAIT before the exact clock boundary");
  }
  const auto idle = io.run();
  if (idle.retired != 0 || idle.fault.kind != FaultKind::none ||
      core.progress().estimated_cycles + io.idle_cycles() != cycles) {
    throw std::runtime_error("Only genuine future permission advances the waiting source");
  }
  const auto before = io.idle_cycles();
  std::exception_ptr failure;
  std::jthread source([&] {
    try {
      while (!token.stop_requested()) {
        const auto result = io.run();
        if (result.fault.kind != FaultKind::none || !io.complete_events(report)) {
          throw std::runtime_error("Waiting source failed");
        }
      }
    } catch (...) { failure = std::current_exception(); }
  });
  std::promise<bool> completion;
  auto finished = completion.get_future();
  std::jthread receiver([&](const std::stop_token &stop) {
    completion.set_value(time.settle_dsp(clock.ticks(), stop));
  });
  const auto before_input = finished.wait_for(std::chrono::milliseconds(10));
  const bool input_known = !delayed_input || midi.advance_input(clock.ticks() + 1);
  const auto ready = finished.wait_for(std::chrono::seconds(2));
  receiver.request_stop();
  receiver.join();
  cancellation.request_stop();
  source.join();
  if (failure) { std::rethrow_exception(failure); }
  if (ready != std::future_status::ready || !finished.get() || !input_known ||
      (delayed_input && before_input != std::future_status::timeout) ||
      io.idle_cycles() != before) {
    throw std::runtime_error(std::format(
        "Current-time WAIT knowledge cannot advance the source clock: before={} after={}",
        before,
        io.idle_cycles()));
  }
}

void waiting_boundary() {
  constexpr std::array entries{boot, boot + 1, boot + 2, vector, vector + 1};
  const std::array spans{CodeSpan{.address = boot, .words = code},
                         CodeSpan{.address = vector, .words = vectors}};
  const std::array variants{
      CodeVariant{.id = 0, .code = spans, .entries = entries, .failed_fetches = {}}};
  const std::array choices{vi::dsp::ProgramChoice{.variant = 0, .resumes = entries, .words = {}}};
  constexpr std::array interrupts{InterruptRequest{.vector = vector, .level = 0}};
  const auto program =
      blofeld::app::aot::prepare_dsp_program({.variants = variants}, choices, {}, interrupts, {});
  vi::Shi shi;
  vi::DspReset reset;
  vi::PeerPort peer;
  vi::ExecutionTime time;
  const std::stop_token cancellation;
  vi::dsp::Io io({.main = code, .eeprom = {}},
                 shi,
                 reset,
                 cancellation,
                 nullptr,
                 {},
                 &peer,
                 &time,
                 nullptr,
                 nullptr,
                 &program);
  Core core(Memory::external(), &io);
  io.bind(core, 0);
  core.set_registers({.pc = boot});
  constexpr Word control = 0xffff8f;
  constexpr Word load = 0xffff8e;
  constexpr Word compare = 0xffff8d;
  constexpr Word timer_priority = 0x100;
  constexpr Word compare_flag = 0x200000;
  const auto write = [&](const Space space, const Word address, const Word value) {
    if (!io.write(space, address, boot, value)) { throw std::runtime_error("DSP timer setup"); }
  };
  write(Space::program, vector, nop);
  write(Space::program, vector + 1, nop);
  write(Space::x, load, 0);
  write(Space::x, compare, 15);
  write(Space::x, control, 7);
  write(Space::x, vi::dsp::Interrupts::peripheral_address, timer_priority);
  vi::ExecutionReport report;
  if (!io.complete_events(report)) { throw std::runtime_error("DSP timer preparation"); }
  io.prepare_code(report);
  constexpr std::uint64_t deadline = 32;
  vi::dsp::Clock clock;
  if (!clock.advance(deadline) || clock.deadline(clock.ticks()) != deadline ||
      !time.publish_mcu(clock.ticks())) {
    throw std::runtime_error("The peer grant must coincide with the real timer compare");
  }
  const auto waited = io.run();
  if (waited.fault.kind != FaultKind::none || waited.retired != 1 ||
      waited.state != ExecutionState::waiting || waited.progress.estimated_cycles != 10 ||
      !io.complete_events(report)) {
    throw std::runtime_error("Execute the actual prepared WAIT before the timer compare");
  }
  const auto before = core.progress();
  const auto bounded = io.run();
  Word flags = 0;
  if (bounded.fault.kind != FaultKind::none || bounded.retired != 0 || core.progress() != before ||
      core.registers().pc != boot + 1 || !io.read(Space::x, control, boot, flags) ||
      (flags & compare_flag) == 0 || before.estimated_cycles + io.idle_cycles() != deadline) {
    throw std::runtime_error(
        "Reaching the peer deadline cannot acknowledge or execute the waiting IRQ");
  }
  if (!time.publish_mcu(clock.ticks() + 1)) { throw std::runtime_error("Next real peer frontier"); }
  const auto entered = io.run();
  if (entered.fault.kind != FaultKind::none || entered.retired != 1 ||
      core.progress().instructions != before.instructions + 1 ||
      !io.read(Space::x, control, boot, flags) || (flags & compare_flag) != 0) {
    throw std::runtime_error(std::format(
        "The retained IRQ executes and acknowledges only under the next grant: fault={} retired={} "
        "instructions={} before={} flags={:x} pc={:x} state={} cycles={} idle={} ticks={}",
        static_cast<unsigned>(entered.fault.kind),
        entered.retired,
        core.progress().instructions,
        before.instructions,
        flags,
        core.registers().pc,
        static_cast<unsigned>(entered.state),
        core.progress().estimated_cycles,
        io.idle_cycles(),
        clock.ticks()));
  }
}
}

int main() try {
  forecast_boundaries();
  constexpr Word reset_clock = 0x04601d;
  constexpr Word fast_clock = 0x012009;
  constexpr Word bypass_clock = 0x04401d;
  for (const auto control : {reset_clock, fast_clock, bypass_clock}) {
    for (const unsigned padding : {1U, 2U}) {
      fractional_port_boundary({.padding = padding, .control = control});
    }
  }
  quiescent_boundary(false);
  quiescent_boundary(true);
  waiting_boundary();
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
