// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"

#include "dsp/coupling.hpp"
#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif

#include "dsp/boot_rom.hpp"
#include "dsp/io.hpp"
#include "execution.hpp"
#include "io/coupling.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "preparation_clock.hpp"
#include "storage/media_image.hpp"

#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <optional>
#include <stdexcept>
#include <stop_token>

namespace blofeld::vi::dsp {
namespace {
constexpr dsp56300::Word boot_first = BootRom::main_first;
constexpr dsp56300::Word reset_status = 0xc00300;
constexpr dsp56300::Word reset_mode = 0x305;

struct OutputOwner {
  PcmOutput output;

  ~OutputOwner() {
    if (output.close != nullptr) { output.close(output.owner); }
  }
};

Access operation(const dsp56300::FaultKind fault) {
  switch (fault) {
    case dsp56300::FaultKind::io_read:
    case dsp56300::FaultKind::memory_read: return Access::read;
    case dsp56300::FaultKind::io_write:
    case dsp56300::FaultKind::memory_write: return Access::write;
    case dsp56300::FaultKind::io_bit: return Access::bit;
    case dsp56300::FaultKind::none:
    case dsp56300::FaultKind::unsupported_instruction:
    case dsp56300::FaultKind::unsupported_mode:
    case dsp56300::FaultKind::stack:
    case dsp56300::FaultKind::invalid_instruction:
    case dsp56300::FaultKind::host_event:
    case dsp56300::FaultKind::unprepared_execution:
    default: return Access::execute;
  }
}

}

void Io::prepare_code() {
  const auto registers = core_->registers();
  const bool transfer = installed_program_ && program_.transition({.variant = *installed_program_,
                                                                   .address = registers.pc});
  const bool mode_changed = registers.omr != prepared_mode_;
  if (!preparation_requested_ && !transfer && !mode_changed) { return; }
  memory_.configure(registers.omr);
  if (memory_.direct() &&
      Memory::mapping_index(registers.omr) != Memory::mapping_index(prepared_mode_)) {
    core_->set_memory(memory_.loan());
  }
  const PreparationClock::Scope phase(preparation_);
  installed_program_ = program_.install(*core_, *this);
  prepared_mode_ = registers.omr;
  preparation_requested_ = false;
}

namespace {
#if defined(BLOFELD_CENSUS)
void trace_session(Io &io,
                   const std::optional<Coupling::Stamp> begun,
                   const dsp56300::RunResult &result) {
  const auto ended = io.timestamp();
  if (!begun || !ended || ended->ticks < begun->ticks) {
    throw std::runtime_error("DSP census has no monotonic owner-clock interval");
  }
  auto endings = io.endings();
  const auto mark = [&](const Census::Event event) {
    endings |= std::uint64_t{1} << static_cast<unsigned>(event);
  };
  if (endings == 0) { mark(Census::Event::other_return); }
  if (result.yielded) { mark(Census::Event::yielded); }
  if (result.state == dsp56300::ExecutionState::waiting) { mark(Census::Event::waiting); }
  if (result.fault.kind != dsp56300::FaultKind::none) { mark(Census::Event::fault); }
  Census::native_session(
      {.begin = begun->ticks, .end = ended->ticks, .endings = endings, .retired = result.retired});
}

dsp56300::RunResult run_observed(Io &io) {
  const auto observed_session = Census::session_begin();
  const bool tracing = Census::tracing();
  const auto begun = tracing ? io.timestamp() : std::nullopt;
  const auto result = io.run();
  Census::unit_exit();
  Census::session_end(observed_session, result.retired);
  if (tracing) { trace_session(io, begun, result); }
  return result;
}
#endif

bool run_instance(Io &io,
                  DspReset &reset,
                  const std::uint64_t epoch,
                  const std::stop_token &stop,
                  ExecutionReport &report,
                  Coupling *coupling) {
  dsp56300::Core core(dsp56300::Memory::external(), &io);
  io.bind(core, epoch);
  const Coupled connected(coupling, io, core);
  core.reset();
  auto registers = core.registers();
  registers.pc = boot_first;
  registers.sr = reset_status;
  registers.omr = reset_mode;
  core.set_registers(registers);
#if defined(BLOFELD_CENSUS)
  io.trace_enabled(Census::tracing());

  struct Sample {
    dsp56300::Core &core;
    Io &io;
    std::uint64_t region_base, precise_base, batch_base, deferred_base;
  };

  const auto origin = io.retirement();
  const auto batches = io.batches();
  Sample sample{.core = core,
                .io = io,
                .region_base = origin[0],
                .precise_base = origin[1],
                .batch_base = batches[0],
                .deferred_base = batches[1]};
  const Census::GuestScope observed(&sample, [](void *context) noexcept {
    const auto &value = *static_cast<Sample *>(context);
    const auto progress = value.core.progress();
    const auto retired = value.io.retirement();
    const auto classified = value.io.batches();
    return Census::Guest{.instructions = progress.instructions,
                         .cycles = progress.estimated_cycles,
                         .idle = value.io.idle_cycles(),
                         .region_instructions = retired[0] - value.region_base,
                         .precise_instructions = retired[1] - value.precise_base,
                         .batched_instructions = classified[0] - value.batch_base,
                         .deferred_instructions = classified[1] - value.deferred_base,
                         .pc = value.core.registers().pc};
  });
#endif
  bool restart = false;
  while (!stop.stop_requested()) {
    if (reset.pending(epoch)) {
      restart = true;
      break;
    }
    io.prepare_code();
#if defined(BLOFELD_CENSUS)
    const auto result = run_observed(io);
#else
    const auto result = io.run();
#endif
    report.instructions += result.retired;
    if (stop.stop_requested()) { break; }
    if (io.reset_interrupted()) {
      restart = true;
      break;
    }
    if (result.fault.kind != dsp56300::FaultKind::none) {
      const auto &fault = result.fault;
      char space = 'P';
      if (fault.space == dsp56300::Space::x) { space = 'X'; }
      if (fault.space == dsp56300::Space::y) { space = 'Y'; }
      report.failure =
          Failure{.operation = operation(fault.kind),
                  .pc = fault.pc,
                  .address = fault.address,
                  .value = fault.value,
                  .code = static_cast<std::uint32_t>(fault.kind),
                  .space = space,
                  .reason = io.error() != nullptr ? io.error()
                                                  : "DSP access or instruction is not implemented"};
      break;
    }
    if (reset.pending(epoch)) {
      restart = true;
      break;
    }
    if (!io.complete_events(report)) {
      restart = io.reset_interrupted();
      break;
    }
    if (result.state != dsp56300::ExecutionState::running && !io.waiting_event()) { break; }
  }
  io.finish(report);
  return restart;
}

bool retire_reset(const DspReset::State state,
                  std::uint64_t &applied,
                  DspReset &reset,
                  Shi &shi,
                  PeerPort *peer,
                  Coupling *coupling) {
  if (state.epoch != applied) {
    // run_instance has destroyed the previous Core before peripherals change.
    // Reset retains SRAM; its known bytes remain owned by this execution Io.
    shi.reset();
    if (peer != nullptr) { peer->retire_mcu_data(); }
    applied = state.epoch;
    if (!reset.acknowledge(applied)) { throw std::logic_error("Invalid DSP reset epoch"); }
    if (coupling != nullptr && !coupling->outputs(coupling->known())) { return false; }
  }
  return true;
}
}

ExecutionReport execute(const BootRom boot,
                        Shi &shi,
                        DspReset &reset,
                        const std::stop_token &stop,
                        MediaImage *image,
                        const PcmOutput output,
                        PeerPort *peer,
                        ExecutionTime *time,
                        MidiPort *midi,
                        PreparationClock *preparation,
                        const Program *program,
                        Coupling *coupling,
                        const Memory::Storage memory) {
  if (program == nullptr) {
    throw std::invalid_argument("DSP execution requires a complete prepared program");
  }
  if (!boot.valid()) {
    throw std::invalid_argument("DSP boot ROM has invalid or overlapping bounds");
  }
  const OutputOwner owner{output};
  Io io(boot,
        shi,
        reset,
        stop,
        image,
        output,
        peer,
        time,
        midi,
        preparation,
        *program,
        coupling,
        memory);
  ExecutionReport report;
  report.prepared_native_bytes = program->report().native_bytes;
  std::uint64_t applied = 0;
  while (!stop.stop_requested()) {
    const auto state = reset.state();
    if (!retire_reset(state, applied, reset, shi, peer, coupling)) { break; }
    if (state.held) {
      if (coupling != nullptr) {
        if (!coupling->held()) { break; }
      } else if (!reset.wait_release(stop)) {
        break;
      }
      continue;
    }
    if (!run_instance(io, reset, applied, stop, report, coupling)) { break; }
  }
  return report;
}
}
