// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/execute.hpp"

#if defined(BLOFELD_CENSUS)
#include "mcu/state_probe.hpp"
#endif

#include "display/state.hpp"
#include "execution.hpp"
#include "io/controls.hpp"
#include "io/coupling.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "io/usb.hpp"
#include "mcu/devices.hpp"
#include "mcu/io.hpp"
#include "mcu/memory.hpp"
#include "mcu/program.hpp"
#include "mcu/speculation.hpp"
#include "mcu/timer.hpp"
#include "storage/flash_image.hpp"

#include <cpu12/core.hpp>
#include <cpu12/memory.hpp>
#include <cstdint>
#include <format>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <stop_token>
#include <vector>

namespace blofeld::vi::mcu {
namespace {
constexpr std::uint16_t reset_vector = 0xfffe;
constexpr std::uint16_t xirq_vector = 0xfff4;
constexpr Timer::Cycle cancellation_cycles = 65536;

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

bool interrupt(cpu12::Core &core,
               const Devices &devices,
               ExecutionReport &report,
               const bool external_due = true) {
  const bool ready = core.interrupt_ready(cpu12::InterruptKind::xirq);
  if (devices.peer != nullptr && !devices.peer->mcu_interrupt_ready(ready)) {
    throw std::overflow_error(devices.peer->error());
  }
  const auto external =
      devices.peer != nullptr && external_due ? devices.peer->mcu_interrupt() : std::nullopt;
  const bool xirq = external && ready;
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
  if (!devices.advance(core)) {
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

void bind_memory(cpu12::Core &core,
                 Memory &memory,
                 Usb &usb,
                 std::vector<cpu12::Ram> &grants,
                 std::vector<std::uint64_t> &mounted,
                 const Program &program) {
  const bool mapping_changed = memory.commit_mapping();
  const bool buffers_changed = usb.commit_mapping();
  if (!mapping_changed && !buffers_changed && program.contains(mounted, core.registers().pc)) {
    return;
  }
  auto selected = program.admit(memory, core.registers().pc);
  auto next = regions(memory, usb);
  core.set_memory(cpu12::Memory::mapped(next));
  program.install(core, selected);
  grants.swap(next);
  mounted.swap(selected);
}

std::logic_error interval_failure(const cpu12::Core &core,
                                  const Devices &devices,
                                  DspReset &reset,
                                  const Io &io,
                                  const std::logic_error &error) {
  const auto registers = core.registers();
  const auto state = reset.state();
  return std::logic_error(std::format(
      "{}; CCR=${:02x} SP=${:04x} D=${:04x} X=${:04x} Y=${:04x} "
      "native={} idle={} clock={} ticks={} reset-held={} epoch={} quiescent={} peer-pending={}; {}",
      error.what(),
      registers.ccr,
      registers.sp,
      registers.d,
      registers.x,
      registers.y,
      core.progress().estimated_cycles,
      devices.idle,
      devices.clock.cycles(),
      devices.clock.ticks(),
      state.held,
      state.epoch,
      reset.quiescent(),
      devices.peer != nullptr && devices.peer->mcu_interrupt().has_value(),
      io.context()));
}

#if defined(BLOFELD_CENSUS)
void record_interval(const cpu12::Core &core,
                     const Memory &memory,
                     const Devices &devices,
                     const Io &io,
                     const ExecutionReport &report,
                     const cpu12::RunResult &result) {
  if (auto *probe = StateProbe::active()) {
    if (result.state == cpu12::ExecutionState::waiting && result.retired != 0) {
      probe->wait(StateProbe::capture(core, memory, devices, io, report.instructions), false);
    }
    probe->commit(report.instructions, core.progress(), devices.clock.ticks());
  }
}
#endif

bool complete_local(cpu12::Core &core,
                    Devices &devices,
                    ExecutionReport &report,
                    Io &io,
                    const cpu12::RunResult &result,
                    const std::stop_token &stop) {
  report.instructions += result.retired;
#if defined(BLOFELD_CENSUS)
  if (auto *probe = StateProbe::active()) { probe->executed(result.retired, false); }
#endif
  if (stop.stop_requested()) { return false; }
  if (result.fault != cpu12::Fault::none) {
    report.failure =
        Failure{.operation = operation(result.fault),
                .pc = result.pc,
                .address = result.address,
                .value = result.value,
                .code = static_cast<std::uint32_t>(result.fault),
                .reason = io.error() != nullptr ? io.error()
                                                : "MCU access or instruction is not implemented"};
    return false;
  }
  if (!io.complete_events()) { return false; }
  advance(core, devices);
  devices.activity(result.state);

  return true;
}

void execute_local(cpu12::Core &core,
                   Devices &devices,
                   Memory &memory,
                   ExecutionReport &report,
                   const Program &program,
                   Io &io,
                   std::vector<cpu12::Ram> &grants,
                   std::vector<std::uint64_t> &mounted,
                   const std::stop_token &stop) {
  while (!stop.stop_requested()) {
    bind_memory(core, memory, devices.usb, grants, mounted, program);
    advance(core, devices);
    if (stop.stop_requested() || core.state().execution == cpu12::ExecutionState::stopped) {
      break;
    }
    if (interrupt(core, devices, report)) {
      devices.activity(cpu12::ExecutionState::running);
      continue;
    }
    if (report.failure) { break; }
    if (core.state().returned_frame) { core.close_return_window(); }
    if (core.state().execution == cpu12::ExecutionState::waiting) {
      const auto next = devices.deadline();
      if (!next) { break; }
      devices.idle = *next - core.progress().estimated_cycles;
      continue;
    }
    const auto now = devices.clock.cycles();
    if (now > std::numeric_limits<Timer::Cycle>::max() - cancellation_cycles) {
      throw std::overflow_error("MCU cancellation boundary overflows");
    }
    auto until = now + cancellation_cycles;
    if (const auto next = devices.deadline(); next && *next < until) { until = *next; }
    const auto result = core.run_until(until - devices.idle);
    if (!complete_local(core, devices, report, io, result, stop)) { break; }
#if defined(BLOFELD_CENSUS)
    record_interval(core, memory, devices, io, report, result);
#endif
  }
}
}

ExecutionReport execute(FlashImage &image,
                        const Program &program,
                        Shi &shi,
                        DspReset &reset,
                        const std::stop_token &stop,
                        const Controls *controls,
                        DisplayState *display,
                        PeerPort *peer,
                        ExecutionTime *time,
                        MidiPort *midi,
                        Coupling *coupling) {
  const PeerAdmission admission{peer};
  if (peer != nullptr && !peer->mcu_interrupt_ready(false)) {
    throw std::overflow_error(peer->error());
  }
  Memory memory(image);
  Devices devices(stop);
  devices.peer = peer;
  devices.time = coupling == nullptr ? time : nullptr;
  devices.midi = midi;
  auto grants = regions(memory, devices.usb);
  Io io(memory,
        image,
        devices,
        shi,
        reset,
        stop,
        controls,
        coupling == nullptr ? display : nullptr,
        peer);
  auto callbacks = io.callbacks();
  cpu12::Core core(cpu12::Memory::mapped(grants), &callbacks);
  io.bind(core);
  if (display != nullptr) { io.capture_display(*display); }
  ExecutionReport report;
  report.prepared_native_bytes = program.report().native_bytes;
  std::vector<std::uint64_t> mounted;
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
  bind_memory(core, memory, devices.usb, grants, mounted, program);
  try {
    if (coupling != nullptr) {
      if (peer == nullptr || midi == nullptr) {
        throw std::logic_error("D2 requires board channels");
      }
      auto session = std::make_unique<Speculation>(*coupling,
                                                   memory,
                                                   devices,
                                                   io,
                                                   core,
                                                   program,
                                                   *peer,
                                                   *midi,
                                                   reset,
                                                   report,
                                                   display,
                                                   controls);
      session->run();
    } else {
      execute_local(core, devices, memory, report, program, io, grants, mounted, stop);
    }
  } catch (const std::logic_error &error) {
    throw interval_failure(core, devices, reset, io, error);
  }

  if (display != nullptr) { display->request(); }
  io.capture_display();
#if defined(BLOFELD_CENSUS)
  if (auto *probe = StateProbe::active()) {
    probe->finish(StateProbe::capture(core, memory, devices, io, report.instructions));
  }
#endif
  report.stopped_pc = core.registers().pc;
  return report;
}
}
