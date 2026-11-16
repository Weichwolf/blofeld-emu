// SPDX-License-Identifier: GPL-3.0-only
#include "mcu/execute.hpp"

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

#include <array>
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

bool interrupt(cpu12::Core &core,
               const Devices &devices,
               ExecutionReport &report,
               const bool external_due = true) {
  const bool ready = core.interrupt_ready(cpu12::InterruptKind::xirq);
  if (devices.peer != nullptr && ready && devices.time != nullptr &&
      !devices.time->settle_dsp(devices.clock.ticks(), devices.stop)) {
    return false;
  }
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

enum class Dispatch : std::uint8_t { run, transaction, again, done };

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
  constexpr std::uint16_t counter_offset = 0x44;
  const auto counter =
      static_cast<std::uint16_t>((unsigned{memory.read_mapping(0x11)} << 8U) + counter_offset);
  if (memory.locate(counter) == Location{.space = Space::registers, .offset = counter_offset} &&
      memory.locate(static_cast<std::uint16_t>(counter + 1)) ==
          Location{.space = Space::registers, .offset = counter_offset + 1}) {
    constexpr std::uint16_t flags_offset = 0x37;
    constexpr std::uint16_t scan_output = 0x270;
    constexpr std::uint16_t led_output = 0x240;
    const auto flags = static_cast<std::uint16_t>(counter - counter_offset + flags_offset);
    const std::array accesses{
        cpu12::LocalAccess{.first = flags, .size = 1, .role = cpu12::LocalAccess::Role::read_write},
        cpu12::LocalAccess{.first = counter, .size = 2},
        cpu12::LocalAccess{.first = static_cast<std::uint16_t>(counter + 2),
                           .size = 1,
                           .role = cpu12::LocalAccess::Role::read_write},
        cpu12::LocalAccess{.first =
                               static_cast<std::uint16_t>(counter - counter_offset + scan_output),
                           .size = 1,
                           .role = cpu12::LocalAccess::Role::write},
        cpu12::LocalAccess{.first =
                               static_cast<std::uint16_t>(counter - counter_offset + led_output),
                           .size = 1,
                           .role = cpu12::LocalAccess::Role::write}};
    core.isolate_accesses(accesses);
  }
  grants.swap(next);
  mounted.swap(selected);
}

std::optional<Timer::Cycle> idle_deadline(Devices &devices, const bool peer_wait) {
  auto next = devices.deadline();
  if (!peer_wait || (next && *next == devices.clock.cycles())) { return next; }
  auto until = std::numeric_limits<std::uint64_t>::max();
  if (next) {
    auto projected = devices.clock;
    if (!projected.advance(*next)) {
      throw std::overflow_error("MCU idle time is not representable");
    }
    until = projected.ticks();
  }
  const auto resumed = devices.time->idle_mcu(devices.clock.ticks(), until, devices.stop);
  if (!resumed) { return {}; }
  if (!next && *resumed == std::numeric_limits<std::uint64_t>::max()) { return {}; }
  next = devices.clock.at_ticks(*resumed);
  if (!next) { throw std::overflow_error("MCU wake time is not representable"); }
  return next;
}

std::uint64_t
running_prefix(cpu12::Core &core, const Devices &devices, const Io &io, const bool peer_event) {
  const auto local = devices.pending();
  const std::array incoming{
      cpu12::InterruptRequest{.vector = xirq_vector, .kind = cpu12::InterruptKind::xirq},
      cpu12::InterruptRequest{.vector = local.value_or(0)}};
  const auto first = peer_event ? 0U : 1U;
  const auto query = [&](const cpu12::ChannelAccess access) {
    return !peer_event && !local
               ? core.channel_prefix(cpu12::PrefixScope::foreground, access)
               : core.channel_prefix(std::span(incoming).subspan(
                                         first, unsigned{peer_event} + unsigned{local.has_value()}),
                                     access);
  };
  const auto prefix = query({});
  if (prefix != 0) { return prefix; }
  const auto reads = io.isolated_reads();
  const auto writes = io.isolated_writes();
  return query({.ranges = reads ? std::span(&*reads, 1) : std::span<const cpu12::LocalAccess>{},
                .writes = writes ? std::span(*writes) : std::span<const cpu12::ByteWrite>{}});
}

std::optional<Timer::Cycle> running_deadline(cpu12::Core &core,
                                             Devices &devices,
                                             const Io &io,
                                             std::optional<Timer::Cycle> deadline,
                                             const bool include_interrupts = false) {
  const auto prefix = running_prefix(core, devices, io, include_interrupts);
  const auto now = devices.clock.cycles();
  if (prefix == 0 || now != core.progress().estimated_cycles + devices.idle) {
    throw std::logic_error(
        std::format("Uncertified running MCU interval at ${:04x}", core.registers().pc));
  }
  constexpr auto infinite = std::numeric_limits<std::uint64_t>::max();
  if (prefix != infinite) {
    if (prefix > infinite - now) { throw std::overflow_error("MCU prefix time overflows"); }
    const auto boundary = now + prefix;
    if (!deadline || boundary < *deadline) { deadline = boundary; }
  }
  auto until = infinite;
  if (deadline) {
    auto projected = devices.clock;
    if (*deadline <= now || !projected.advance(*deadline)) {
      throw std::logic_error("MCU running interval has no future boundary");
    }
    until = projected.ticks();
  }
  const auto resumed = devices.time->idle_mcu(
      devices.clock.ticks(), until, devices.stop, ExecutionTime::McuWork::running);
  if (!resumed || *resumed == infinite) { return {}; }
  return devices.clock.at_ticks(*resumed);
}

Dispatch source_boundary(cpu12::Core &core,
                         Devices &devices,
                         const Io &io,
                         std::optional<Timer::Cycle> &admitted) {
  if (admitted && *admitted <= devices.clock.cycles()) { admitted.reset(); }
  if (admitted || devices.peer == nullptr || devices.time == nullptr ||
      !core.interrupt_ready(cpu12::InterruptKind::xirq)) {
    return Dispatch::run;
  }
  const bool poll = core.state().returned_frame ||
                    (devices.pending() && core.interrupt_ready(cpu12::InterruptKind::maskable));
  if (!poll && core.state().execution != cpu12::ExecutionState::running) { return Dispatch::run; }
  if (!devices.time->wait_for_dsp(devices.clock.ticks(), devices.stop)) { return Dispatch::done; }
  if (!devices.peer->mcu_interrupt_ready(true)) {
    throw std::overflow_error(devices.peer->error());
  }
  if (devices.peer->mcu_interrupt()) { return Dispatch::run; }
  if (running_prefix(core, devices, io, poll) != 0) {
    admitted = running_deadline(core, devices, io, devices.deadline(), poll);
    return admitted ? Dispatch::run : Dispatch::done;
  }
  if (core.state().execution != cpu12::ExecutionState::running) { return Dispatch::run; }
  return devices.time->settle_dsp(devices.clock.ticks(), devices.stop) ? Dispatch::transaction
                                                                       : Dispatch::done;
}

std::optional<cpu12::RunResult> run_interval(cpu12::Core &core,
                                             Devices &devices,
                                             const Io &io,
                                             std::optional<Timer::Cycle> &admitted) {
  auto deadline = devices.deadline();
  if (devices.peer != nullptr && devices.time != nullptr &&
      core.interrupt_ready(cpu12::InterruptKind::xirq)) {
    if (!admitted) { admitted = running_deadline(core, devices, io, deadline); }
    deadline = admitted;
    if (!deadline) { return {}; }
  }
  return deadline || devices.interrupts_enabled()
             ? core.run_until(deadline ? *deadline - devices.idle : no_deadline)
             : core.run();
}

Dispatch prepare(cpu12::Core &core,
                 Devices &devices,
                 ExecutionReport &report,
                 const Io &io,
                 std::optional<Timer::Cycle> &admitted) {
  advance(core, devices);
  if (devices.stop.stop_requested()) { return Dispatch::done; }
  const auto state = core.state().execution;
  if (state == cpu12::ExecutionState::stopped) { return Dispatch::done; }
  const bool peer_wait = state == cpu12::ExecutionState::waiting && devices.peer != nullptr &&
                         devices.time != nullptr &&
                         core.interrupt_ready(cpu12::InterruptKind::xirq);
  if (peer_wait && !devices.time->wait_for_dsp(devices.clock.ticks(), devices.stop)) {
    return Dispatch::done;
  }
  const auto boundary = source_boundary(core, devices, io, admitted);
  if (boundary == Dispatch::done) { return boundary; }
  const bool external_due = !admitted || *admitted <= devices.clock.cycles();
  if (interrupt(core, devices, report, external_due)) {
    devices.activity(cpu12::ExecutionState::running);
    return Dispatch::again;
  }
  if (report.failure) { return Dispatch::done; }
  if (core.state().returned_frame) { core.close_return_window(); }
  if (state == cpu12::ExecutionState::waiting) {
    const auto next = idle_deadline(devices, peer_wait);
    if (!next) { return Dispatch::done; }
    devices.idle = *next - core.progress().estimated_cycles;
    return Dispatch::again;
  }
  return boundary;
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

void execute_intervals(cpu12::Core &core,
                       Devices &devices,
                       Memory &memory,
                       ExecutionReport &report,
                       const Program &program,
                       Io &io,
                       std::vector<cpu12::Ram> &grants,
                       std::vector<std::uint64_t> &mounted,
                       std::optional<Timer::Cycle> &admitted,
                       const std::stop_token &stop) {
  while (!stop.stop_requested()) {
    bind_memory(core, memory, devices.usb, grants, mounted, program);
    const auto dispatch = prepare(core, devices, report, io, admitted);
    if (dispatch == Dispatch::done) { break; }
    if (dispatch == Dispatch::again) { continue; }
    const auto result = dispatch == Dispatch::transaction
                            ? std::optional{core.step()}
                            : run_interval(core, devices, io, admitted);
    if (!result) { break; }
    report.instructions += result->retired;
    if (stop.stop_requested()) { break; }
    if (result->fault != cpu12::Fault::none) {
      report.failure =
          Failure{.operation = operation(result->fault),
                  .pc = result->pc,
                  .address = result->address,
                  .value = result->value,
                  .code = static_cast<std::uint32_t>(result->fault),
                  .reason = io.error() != nullptr ? io.error()
                                                  : "MCU access or instruction is not implemented"};
      break;
    }
    if (!io.complete_events()) { break; }
    advance(core, devices);
    devices.activity(result->state);
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
  std::optional<Timer::Cycle> admitted;
  Io io(memory,
        image,
        devices,
        shi,
        reset,
        stop,
        controls,
        coupling == nullptr ? display : nullptr,
        peer);
  io.input_horizon(admitted);
  auto callbacks = io.callbacks();
  cpu12::Core core(cpu12::Memory::mapped(grants), &callbacks);
  io.bind(core);
  io.capture_display();
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
      execute_intervals(
          core, devices, memory, report, program, io, grants, mounted, admitted, stop);
    }
  } catch (const std::logic_error &error) {
    throw interval_failure(core, devices, reset, io, error);
  }

  if (core.statistics().compiled_entries != 0) {
    throw std::logic_error("Prepared MCU execution generated runtime code");
  }
  if (display != nullptr) { display->request(); }
  io.capture_display();
  report.stopped_pc = core.registers().pc;
  return report;
}
}
