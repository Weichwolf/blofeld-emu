// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/dsp_interlocks.hpp"
#include "dsp56300/core.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace blofeld {
// Qualified uses measured resource descriptions only. Conservative serializes
// valid Core instructions when that description is absent; bus effects remain real.
enum class DspDependencyPolicy : std::uint8_t { qualified, conservative };

struct DspTimedInstruction {
  dsp56300::InstructionWords words;
  unsigned base_units;
  unsigned dependency_units;
};

// Single-owner timing around real bounded CPU execution. The supplied Io must
// be that Core's stable code/memory owner. No external mutation may occur during
// run(); native Core guards still own cache, peripheral and RAM admission.
class DspNativeTimedExecution {
public:
  explicit DspNativeTimedExecution(dsp56300::Progress initial = {},
                                   DspDependencyPolicy policy = DspDependencyPolicy::qualified);
  DspNativeTimedExecution(const DspNativeTimedExecution &other);
  DspNativeTimedExecution &operator=(const DspNativeTimedExecution &other);
  DspNativeTimedExecution(DspNativeTimedExecution &&) noexcept = default;
  DspNativeTimedExecution &operator=(DspNativeTimedExecution &&) noexcept = default;
  ~DspNativeTimedExecution() = default;

  [[nodiscard]] dsp56300::BoundedRunResult
  run(dsp56300::Core &cpu, const dsp56300::Io &code, dsp56300::RunBudget budget);

  [[nodiscard]] DspTimedInstruction prepare_owner(const dsp56300::Core &cpu,
                                                  dsp56300::InstructionWords words,
                                                  unsigned base_units) const;
  // The external clock owner executes the captured instruction through genuine
  // staged Core effects, then supplies its actually accounted dependency clocks.
  void complete_owner(const dsp56300::Core &cpu, DspTimedInstruction accounted);
  void idle(const dsp56300::Core &cpu, unsigned units);
  void reset(dsp56300::Progress actual);

  [[nodiscard]] DspDependencyPolicy policy() const noexcept { return policy_; }

private:
  static constexpr unsigned maximum_prefix = 64;
  static constexpr unsigned cost_slots = 256;

  struct Cost {
    dsp56300::Word opcode = dsp56300::word_mask + 1;
    unsigned units = 0;
  };

  struct Offered {
    dsp56300::InstructionWords words;
    unsigned base_units;
  };

  struct Entry {
    dsp56300::InstructionWords words;
    DspInterlockInstruction dependencies;
    bool sequential;
  };

  struct Cursor {
    const dsp56300::Io &code;
    dsp56300::InstructionWords initial;
    std::optional<dsp56300::Word> repeated;
    dsp56300::Word address;
    bool first;
  };

  [[nodiscard]] static unsigned slot(dsp56300::Word opcode);
  void check_progress(const dsp56300::Core &cpu) const;
  void check_offer(dsp56300::InstructionWords words, unsigned units) const;
  void learn(const dsp56300::BoundedRunResult &preview);
  [[nodiscard]] std::optional<unsigned> cost(dsp56300::Word opcode) const;
  [[nodiscard]] static DspInterlockInstruction required(dsp56300::InstructionWords words,
                                                        unsigned units);
  [[nodiscard]] static std::optional<DspInterlockInstruction> describe(dsp56300::Word opcode,
                                                                       unsigned units);
  [[nodiscard]] static std::optional<DspInterlockInstruction> control(dsp56300::Word opcode,
                                                                      unsigned units);
  [[nodiscard]] static bool conditional(dsp56300::Word opcode);
  [[nodiscard]] std::optional<Entry>
  inspect(const Cursor &cursor, const DspInterlocks &history, std::uint64_t available) const;
  [[nodiscard]] unsigned plan(const dsp56300::Core &cpu,
                              const dsp56300::Io &code,
                              const dsp56300::BoundedRunResult &preview,
                              dsp56300::RunBudget budget);
  void accept(const dsp56300::Core &cpu, const dsp56300::BoundedRunResult &actual);
  dsp56300::Progress expected_;
  DspDependencyPolicy policy_ = DspDependencyPolicy::qualified;
  unsigned control_delay_ = 0;
  DspInterlocks history_;
  std::optional<Offered> offered_;
  std::array<Cost, cost_slots> costs_{};
  std::vector<Entry> prefix_;
};
}
