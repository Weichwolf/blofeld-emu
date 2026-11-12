// SPDX-License-Identifier: GPL-3.0-only
#include "dsp/execute.hpp"
#if defined(BLOFELD_CENSUS)
#include "census.hpp"
#endif

#include "dsp/boot_rom.hpp"
#include "dsp/io.hpp"
#include "execution.hpp"
#include "io/dsp_reset.hpp"
#include "io/execution_time.hpp"
#include "io/midi_port.hpp"
#include "io/pcm_output.hpp"
#include "io/peer_port.hpp"
#include "io/shi.hpp"
#include "preparation_clock.hpp"
#include "storage/media_image.hpp"

#include <chrono>
#include <cstdint>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
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

void Io::prepare_code(ExecutionReport &report) {
  if (program_ == nullptr && !preparation_requested_) { return; }
  const auto registers = core_->registers();
  const bool transfer =
      program_ != nullptr && installed_program_ &&
      program_->transition({.variant = *installed_program_, .address = registers.pc});
  if (!preparation_requested_ && !transfer) { return; }
  if (program_ != nullptr) {
    memory_.configure(registers.omr);
    const PreparationClock::Scope phase(preparation_);
    installed_program_ = program_->install(*core_, *this);
    prepared_mode_ = registers.omr;
    prepared_vba_ = registers.vba & vector_base_mask;
    preparation_requested_ = false;
    return;
  }
  // Reset, reached code and configured vectors establish entries, not a ROM scan.
  auto entries = interrupts_.entries(registers.vba);
  entries.insert(entries.end(), {0, boot_first, registers.pc});
  const PreparationClock::Scope phase(preparation_);
  const auto begun = std::chrono::steady_clock::now();
  const auto prepared = core_->prepare(entries);
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - begun)
          .count();
  report.preparation_nanoseconds += static_cast<std::uint64_t>(elapsed);
  report.prepared_native_bytes += prepared.native_bytes;
  prepared_mode_ = registers.omr;
  prepared_vba_ = registers.vba & vector_base_mask;
  preparation_requested_ = false;
  if (prepared.failure.kind != dsp56300::FaultKind::none) {
    const auto &fault = prepared.failure;
    char space = 'P';
    if (fault.space == dsp56300::Space::x) { space = 'X'; }
    if (fault.space == dsp56300::Space::y) { space = 'Y'; }
    report.preparation_diagnostic = Failure{
        .operation = operation(fault.kind),
        .pc = fault.pc,
        .address = fault.address,
        .value = fault.value,
        .code = static_cast<std::uint32_t>(fault.kind),
        .space = space,
        .reason = "DSP preparation incomplete; remaining entries use retained native execution"};
  }
}

namespace {
bool run_instance(Io &io,
                  DspReset &reset,
                  const std::uint64_t epoch,
                  const std::stop_token &stop,
                  ExecutionReport &report) {
  dsp56300::Core core(dsp56300::Memory::external(), &io);
  io.bind(core, epoch);
  core.reset();
  auto registers = core.registers();
  registers.pc = boot_first;
  registers.sr = reset_status;
  registers.omr = reset_mode;
  core.set_registers(registers);
#if defined(BLOFELD_CENSUS)
  struct Sample {
    dsp56300::Core &core;
    Io &io;
  };

  Sample sample{.core = core, .io = io};
  const Census::GuestScope observed(&sample, [](void *context) noexcept {
    const auto &value = *static_cast<Sample *>(context);
    const auto progress = value.core.progress();
    return Census::Guest{.instructions = progress.instructions,
                         .cycles = progress.estimated_cycles,
                         .idle = value.io.idle_cycles(),
                         .pc = value.core.registers().pc};
  });
#endif
  bool restart = false;
  while (!stop.stop_requested()) {
    if (reset.pending(epoch)) {
      restart = true;
      break;
    }
    io.prepare_code(report);
#if defined(BLOFELD_CENSUS)
    const auto observed_session = Census::session_begin();
#endif
    const auto result = io.run();
#if defined(BLOFELD_CENSUS)
    Census::session_end(observed_session, result.retired);
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
                        const Program *program) {
  if (!boot.valid()) {
    throw std::invalid_argument("DSP boot ROM has invalid or overlapping bounds");
  }
  const OutputOwner owner{output};
  Io io(boot, shi, reset, stop, image, output, peer, time, midi, preparation, program);
  ExecutionReport report;
  if (program != nullptr) { report.prepared_native_bytes = program->report().native_bytes; }
  std::uint64_t applied = 0;
  while (!stop.stop_requested()) {
    const auto state = reset.state();
    if (state.epoch != applied) {
      // run_instance has destroyed the previous Core before peripherals change.
      // Reset retains SRAM; its known bytes remain owned by this execution Io.
      shi.reset();
      if (peer != nullptr) { peer->retire_mcu_data(); }
      applied = state.epoch;
      if (!reset.acknowledge(applied)) { throw std::logic_error("Invalid DSP reset epoch"); }
    }
    if (state.held) {
      if (!reset.wait_release(stop)) { break; }
      continue;
    }
    if (!run_instance(io, reset, applied, stop, report)) { break; }
  }
  return report;
}
}
