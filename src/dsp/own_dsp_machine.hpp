// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_run_clock.hpp"
#include "dsp/own_dsp_interrupts.hpp"
#include "dsp/own_dsp_io.hpp"
#include "dsp56300/core.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace blofeld {
class OwnDspFetch;

struct OwnDspResetInputs {
  std::uint8_t mode_known = 0;
  std::uint8_t mode_levels = 0;
  std::optional<bool> pinit;
  std::uint32_t recovery_cycles = 0;
};

// Immutable trusted in-process chip state, excluding external board devices.
class OwnDspCheckpoint {
public:
  OwnDspCheckpoint() = default;
  OwnDspCheckpoint(const OwnDspCheckpoint &) = default;
  OwnDspCheckpoint &operator=(const OwnDspCheckpoint &) = default;
  OwnDspCheckpoint(OwnDspCheckpoint &&) noexcept = default;
  OwnDspCheckpoint &operator=(OwnDspCheckpoint &&) noexcept = default;
  ~OwnDspCheckpoint();

private:
  friend class OwnDspMachine;
  struct Storage;
  explicit OwnDspCheckpoint(std::shared_ptr<const Storage> storage);
  std::shared_ptr<const Storage> storage_;
};

enum class OwnDspNativeCycleModel : std::uint8_t { unqualified, isolated_effect };

struct OwnDspDmaCycleResult {
  DspDmaBusResult dma;
  std::optional<dsp56300::InstructionProgress> native;
};

// Coordinated own-core owner. Only reset time is modeled here; staged execution
// remains untimed until the board scheduler supplies pipeline and bus cycles.
class OwnDspMachine {
public:
  explicit OwnDspMachine(DspRunClockInputs clock);
  OwnDspMachine(const OwnDspMachine &) = delete;
  OwnDspMachine &operator=(const OwnDspMachine &) = delete;

  [[nodiscard]] DspBus &bus() noexcept { return bus_; }

  [[nodiscard]] const dsp56300::Core &cpu() const noexcept { return cpu_; }

  [[nodiscard]] const DspRunClock &clock() const noexcept { return clock_; }

  [[nodiscard]] const OwnDspIo &io() const noexcept { return io_; }

  // Assertion discards staged work at the current published effect boundary.
  // Previously accepted bus writes remain; subcycle assertion timing is unqualified.
  void begin_hardware_reset(OwnDspResetInputs inputs);
  std::uint64_t advance_hardware_reset_ticks(std::uint64_t maximum_ticks);
  void release_hardware_reset();
  [[nodiscard]] dsp56300::InstructionProgress begin_fetch(
      dsp56300::InstructionPublication publication = dsp56300::InstructionPublication::pcu_updates);
  [[nodiscard]] dsp56300::InstructionProgress advance(const dsp56300::EffectTicket &ticket);
  // One externally scheduled reference period; does not advance an instruction.
  // DMA arbitration and subcycle effects remain the scheduler's responsibility.
  std::uint64_t advance_reference_cycle();
  // Explicit DMA-first cycle with the native CPU and captured fetch held.
  // The caller supplies actual request levels and consecutive cycle numbers.
  [[nodiscard]] DspDmaBusResult advance_dma_reference_cycle(DspDmaInputs inputs);
  // DMA first, then the actual pending native effect if its bus is not stalled.
  // The caller must qualify the whole instruction as safe to split at this effect;
  // staged ordinary memory effects alone do not identify atomic instructions.
  [[nodiscard]] OwnDspDmaCycleResult advance_dma_native_cycle(DspDmaInputs inputs,
                                                              OwnDspNativeCycleModel model);
  [[nodiscard]] bool admit_interrupt(dsp56300::InstructionWords foreground);
  [[nodiscard]] OwnDspCheckpoint checkpoint() const;
  // An untimed speculative continuation, not a published instruction or
  // complete board checkpoint. Peripheral effects already admitted are copied.
  [[nodiscard]] OwnDspCheckpoint candidate_checkpoint(const dsp56300::EffectTicket &ticket) const;
  // Strong exception guarantee. Requires identical clock inputs. External pad
  // readers are detached, retaining the obligation to reconnect before sampling.
  // Previously issued effect tickets become invalid, including on the same owner.
  void restore(const OwnDspCheckpoint &checkpoint);

private:
  friend class OwnDspFetch;
  friend class OwnDspTimeline;
  void require_running() const;
  [[nodiscard]] static std::optional<DspMemoryAccess>
  dma_native_footprint(const dsp56300::PendingInstruction &pending);
  [[nodiscard]] OwnDspDmaCycleResult dma_cycle(DspDmaInputs inputs, bool admit_native);
  DspBus bus_;
  OwnDspIo io_;
  dsp56300::Core cpu_;
  OwnDspInterrupts interrupts_;
  DspRunClock clock_;
  const DspRunClockInputs clock_inputs_;
  std::weak_ptr<const bool> captured_fetch_owner_;
  bool advancing_effect_ = false;
};
}
