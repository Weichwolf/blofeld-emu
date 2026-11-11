// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/execute.hpp"

#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "io/usb.hpp"
#include "mcu/devices.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/timer.hpp"
#include "preparation_clock.hpp"
#include "storage/flash_image.hpp"

#include <array>
#include <chrono>
#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t reset_vector = 0xfffe;
constexpr std::uint16_t xirq_vector = 0xfff4;
// An enabled but masked IRQ still needs native CCR/RTI effect boundaries.
// This unreachable horizon selects that Core path without an execution quantum.
constexpr auto no_deadline = std::numeric_limits<Timer::Cycle>::max() / 2;

struct PeerAdmission {
  PeerPort *peer;

  ~PeerAdmission() {
    if (peer != nullptr) { static_cast<void>(peer->mcu_interrupt_ready(false)); }
  }
};

Access operation(const cpu12::Fault fault) {
  switch (fault) {
    case cpu12::Fault::memory_read: return Access::read;
    case cpu12::Fault::memory_write: return Access::write;
    case cpu12::Fault::instruction_fetch: return Access::fetch;
    case cpu12::Fault::none:
    case cpu12::Fault::invalid_input:
    case cpu12::Fault::unsupported_instruction:
    case cpu12::Fault::unsupported_host:
    case cpu12::Fault::reentrant:
    case cpu12::Fault::unsupported_state:
    case cpu12::Fault::unprepared_execution:
    default: return Access::execute;
  }
}

bool interrupt(cpu12::Core &core, const Devices &devices, ExecutionReport &report) {
  if (devices.peer != nullptr &&
      !devices.peer->mcu_interrupt_ready(core.interrupt_ready(cpu12::InterruptKind::xirq))) {
    throw std::overflow_error(devices.peer->error());
  }
  const auto external = devices.peer != nullptr ? devices.peer->mcu_interrupt() : std::nullopt;
  const bool xirq = external && core.interrupt_ready(cpu12::InterruptKind::xirq);
  const auto vector = xirq ? std::optional<std::uint16_t>{xirq_vector} : devices.pending();
  const auto kind = xirq ? cpu12::InterruptKind::xirq : cpu12::InterruptKind::maskable;
  if (!vector || !core.interrupt_ready(kind)) { return false; }
  const auto entered = core.interrupt({.vector = *vector, .kind = kind});
  if (entered.fault != cpu12::Fault::none) {
    report.failure = Failure{.operation = operation(entered.fault),
                             .pc = entered.pc,
                             .address = entered.address,
                             .value = entered.value,
                             .code = static_cast<std::uint32_t>(entered.fault),
                             .reason = "MCU device interrupt entry failed"};
    return false;
  }
  if (!entered.accepted) { throw std::logic_error("Eligible MCU device interrupt was refused"); }
  if (xirq &&
      (!devices.peer->mcu_interrupt_ready(false) || !devices.peer->acknowledge_mcu(*external))) {
    throw std::logic_error("Accepted MCU peer interrupt has an invalid generation");
  }
  return true;
}

void advance(const cpu12::Core &core, Devices &devices) {
  if (!devices.advance(core) ||
      (devices.time != nullptr && !devices.time->publish_mcu(devices.clock.ticks()))) {
    if (devices.stop.stop_requested()) { return; }
    if (devices.error() != nullptr) { throw std::runtime_error(devices.error()); }
    throw std::overflow_error("MCU device time is not representable or monotonic");
  }
}

std::vector<cpu12::Ram> regions(Memory &memory, Usb &usb) {
  auto result = memory.regions();
  constexpr std::uint16_t register_mapping = 0x11;
  const auto base =
      static_cast<std::uint16_t>(unsigned{memory.read_mapping(register_mapping)} << 8U);
  for (const auto window : usb.regions(base)) {
    if (!window.bytes.empty()) { result.push_back(window); }
  }
  return result;
}

enum class Dispatch : std::uint8_t { run, again, done };

void prepare_code(cpu12::Core &core, ExecutionReport &report, PreparationClock *preparation) {
  const std::array entries{core.registers().pc};
  const PreparationClock::Scope phase(preparation);
  const auto begun = std::chrono::steady_clock::now();
  const auto prepared = core.prepare(entries);
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begun)
          .count();
  report.preparation_nanoseconds += static_cast<std::uint64_t>(elapsed);
  report.prepared_native_bytes += prepared.native_bytes;
  if (prepared.failure.fault != cpu12::Fault::none) {
    report.preparation_diagnostic = Failure{
        .operation = operation(prepared.failure.fault),
        .pc = prepared.failure.pc,
        .address = prepared.failure.address,
        .value = prepared.failure.value,
        .code = static_cast<std::uint32_t>(prepared.failure.fault),
        .reason = "MCU preparation incomplete; remaining entries use retained native execution"};
  }
}

void bind_memory(cpu12::Core &core,
                 Memory &memory,
                 Usb &usb,
                 std::vector<cpu12::Ram> &grants,
                 ExecutionReport &report,
                 PreparationClock *preparation) {
  const bool mapping_changed = memory.commit_mapping();
  const bool buffers_changed = usb.commit_mapping();
  const bool program_changed = memory.bind_program(core.registers().pc);
  if (!mapping_changed && !buffers_changed && !program_changed) { return; }
  auto next_grants = regions(memory, usb);
  core.set_memory(cpu12::Memory::mapped(next_grants));
  grants.swap(next_grants);
  prepare_code(core, report, preparation);
}

Dispatch prepare(cpu12::Core &core, Devices &devices, ExecutionReport &report) {
  advance(core, devices);
  if (devices.stop.stop_requested()) { return Dispatch::done; }
  const auto state = core.state().execution;
  if (state == cpu12::ExecutionState::stopped) { return Dispatch::done; }
  if (interrupt(core, devices, report)) {
    devices.activity(cpu12::ExecutionState::running);
    return Dispatch::again;
  }
  if (report.failure) { return Dispatch::done; }
  if (core.state().returned_frame) { core.close_return_window(); }
  if (state == cpu12::ExecutionState::waiting) {
    const auto next = devices.deadline();
    if (!next) { return Dispatch::done; }
    devices.idle = *next - core.progress().estimated_cycles;
    return Dispatch::again;
  }
  return Dispatch::run;
}

}

ExecutionReport execute(FlashImage &image,
                        Shi &shi,
                        DspReset &reset,
                        const std::stop_token &stop,
                        const Controls *controls,
                        DisplayState *display,
                        PeerPort *peer,
                        ExecutionTime *time,
                        MidiPort *midi,
                        PreparationClock *preparation) {
  const PeerAdmission admission{peer};
  if (peer != nullptr && !peer->mcu_interrupt_ready(false)) {
    throw std::overflow_error(peer->error());
  }
  Memory memory(image);
  Devices devices(stop);
  devices.peer = peer;
  devices.time = time;
  devices.midi = midi;
  auto grants = regions(memory, devices.usb);
  Io io(memory, image, devices, shi, reset, stop, controls, display, peer);
  auto callbacks = io.callbacks();
  cpu12::Core core(cpu12::Memory::mapped(grants), &callbacks);
  io.bind(core);
  io.capture_display();
  ExecutionReport report;
  const auto initial = core.reset(reset_vector);
  if (!initial.accepted) {
    report.failure = Failure{.operation = Access::reset,
                             .pc = initial.pc,
                             .address = initial.address,
                             .value = initial.value,
                             .code = static_cast<std::uint32_t>(initial.fault),
                             .reason = "MCU reset vector access failed"};
    return report;
  }
  prepare_code(core, report, preparation);
  while (!stop.stop_requested()) {
    bind_memory(core, memory, devices.usb, grants, report, preparation);
    const auto dispatch = prepare(core, devices, report);
    if (dispatch == Dispatch::done) { break; }
    if (dispatch == Dispatch::again) { continue; }
    const auto deadline = devices.deadline();
    const auto result = deadline || devices.interrupts_enabled()
                            ? core.run_until(deadline ? *deadline - devices.idle : no_deadline)
                            : core.run();
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (result.fault != cpu12::Fault::none) {
      report.failure =
          Failure{.operation = operation(result.fault),
                  .pc = result.pc,
                  .address = result.address,
                  .value = result.value,
                  .code = static_cast<std::uint32_t>(result.fault),
                  .reason = io.error() != nullptr ? io.error()
                                                  : "MCU access or instruction is not implemented"};
      break;
    }
    if (!io.complete_events()) { break; }
    advance(core, devices);
    devices.activity(result.state);
  }

  if (display != nullptr) { display->request(); }
  io.capture_display();
  report.stopped_pc = core.registers().pc;
  return report;
}
}
