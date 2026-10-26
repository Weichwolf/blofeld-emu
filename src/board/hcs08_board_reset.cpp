// SPDX-License-Identifier: GPL-3.0-only
#include "board/hcs08_board.hpp"
#include "board/hcs08_board_memory.hpp"
#include "common/bus.hpp"
#include "keyboard/hcs08.hpp"
#include "keyboard/hcs08_bdc.hpp"
#include "keyboard/hcs08_flash.hpp"
#include "keyboard/hcs08_icg.hpp"
#include "keyboard/hcs08_run_clock.hpp"

#include <filesystem>
#include <optional>
#include <utility>

namespace blofeld {
Hcs08 &Hcs08Board::reset_checkpoint_cpu(Hcs08 &cpu,
                                        Hcs08BoardMemory &memory,
                                        const std::optional<Hcs08Checkpoint> &before,
                                        const Hcs08BoardResetRequest &request,
                                        const Hcs08BoardClockObservation observation) {
  if (before) { cpu.restore_checkpoint(*before); }
  if (observation != Hcs08BoardClockObservation::unobserved) {
    memory.observe_clock_interval(observation);
    memory.observed_cpu_period_ = true;
  }
  cpu.reset(request.entry, request.cause);
  memory.observed_cpu_period_ = false;
  return cpu;
}

Hcs08BoardResetCheckpoint Hcs08Board::power_on_checkpoint(const Hcs08FlashImage &image,
                                                          const Hcs08BoardContract contract,
                                                          const Hcs08ResetEntry entry,
                                                          const Hcs08RtiIrqContract rti) {
  if (contract != Hcs08BoardContract::candidate_gt8a_reference_retirement ||
      (entry != Hcs08ResetEntry::normal && entry != Hcs08ResetEntry::active_background)) {
    throw Fault("HCS08 board power-on address=0xfffe pc=unknown agent=CPU: explicit board/reset "
                "entry contract required");
  }
  Hcs08BoardMemory memory(image, Hcs08BoardMemoryContract::candidate_gt8a_reference_bus);
  constexpr unsigned power_on_status = 0x80;
  return {std::move(memory),
          std::nullopt,
          {.cause = Hcs08ResetCause::power_on, .entry = entry, .reset_sources = power_on_status},
          rti};
}

Hcs08BoardResetCheckpoint::Hcs08BoardResetCheckpoint(Hcs08BoardMemory memory,
                                                     std::optional<Hcs08Checkpoint> cpu,
                                                     const Hcs08BoardResetRequest request,
                                                     const Hcs08RtiIrqContract rti)
    : memory_(std::move(memory)), cpu_(std::move(cpu)), request_(request), rti_(rti) {}

Hcs08BoardResetCheckpoint
Hcs08Board::reset_checkpoint(const Hcs08BoardResetRequest &request) const {
  if (request.cause != Hcs08ResetCause::power_on && request.cause != Hcs08ResetCause::warm) {
    fail("reset checkpoint", "explicit architectural reset cause required");
  }
  if (request.entry != Hcs08ResetEntry::normal &&
      request.entry != Hcs08ResetEntry::active_background) {
    fail("reset checkpoint", "explicit sampled reset entry required");
  }
  if (request.retention != Hcs08FlashAbort::unqualified_retention &&
      request.retention != Hcs08FlashAbort::reference_command_range) {
    fail("reset checkpoint", "explicit NVM retention contract required");
  }
  // Warm reset cannot invent A/X after a partially failed native CPU transaction.
  // A pending reservation is not a committed instruction;native reset discards it.
  const auto before =
      request.cause == Hcs08ResetCause::warm ? std::optional(cpu_.checkpoint()) : std::nullopt;
  auto memory = memory_;
  memory.reset_peripherals(request.cause == Hcs08ResetCause::power_on ? Hcs08IcgReset::power_on
                                                                      : Hcs08IcgReset::warm,
                           request.reset_sources,
                           request.retention);
  return {std::move(memory), before, request, rti_};
}

Hcs08Board::Hcs08Board(const Hcs08BoardResetCheckpoint &checkpoint,
                       const std::filesystem::path &specification,
                       const Hcs08BoardInputs &inputs)
    : memory_(checkpoint.memory_),
      cpu_(memory_, specification, checkpoint.rti_),
      bdc_(reset_checkpoint_cpu(
               cpu_, memory_, checkpoint.cpu_, checkpoint.request_, inputs.reset_observation),
           memory_,
           Hcs08FlashContract::candidate_gt8a,
           checkpoint.request_.entry == Hcs08ResetEntry::active_background
               ? Hcs08BdcReset::bkgd_held_low
               : Hcs08BdcReset::normal),
      clock_(memory_, bdc_, Hcs08RunContract::reference_instruction_retirement),
      rti_(checkpoint.rti_) {
  if (inputs.adc_reference) { memory_.observe_adc_reference(*inputs.adc_reference); }
  for (unsigned channel = 0; channel < inputs.analog.size(); ++channel) {
    if (inputs.analog[channel]) { memory_.observe_adc_input(channel, inputs.analog[channel]); }
  }
  if (inputs.timer_phase) { memory_.observe_timer_phase(*inputs.timer_phase); }
}
} // namespace blofeld
