// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "dsp/detail/dsp_program_fetch.hpp"
#include "dsp/dsp_branch_form.hpp"
#include "dsp/dsp_bus.hpp"
#include "dsp/dsp_native_access.hpp"
#include "dsp/dsp_peripheral_read_form.hpp"
#include "dsp/dsp_peripheral_read_stage.hpp"
#include "dsp/dsp_program_fetch.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace blofeld {
// Internal once-only bus response. DspCore must own generation/ticket admission;
// this value object is not a caller-supplied or serialized execution grant.
class DspPeripheralRead {
public:
  [[nodiscard]] static DspPeripheralRead capture(DspBus &bus,
                                                 const DspProgramFetch &fetch,
                                                 DspPeripheralReadPoint point,
                                                 std::uint64_t occurrence = 0);

  [[nodiscard]] const DspFetchedInstruction &fetched() const { return fetched_; }

  void follow_context(const DspFetchContext &transition);

  [[nodiscard]] std::uint32_t sample_omr() const { return sample_omr_; }

  [[nodiscard]] bool
  forward(DspProgramWord producer, const DspNativeAccess &completed_store, std::uint64_t epoch);
  [[nodiscard]] std::uint32_t consume(const DspNativeAccess &opcode,
                                      const std::optional<DspNativeAccess> &extension,
                                      const DspNativeAccess &pending,
                                      std::uint64_t epoch);
  [[nodiscard]] DspBranchDecision preview_branch(std::uint64_t epoch) const;

private:
  DspPeripheralRead() = default;
  void check_word(const DspNativeAccess &access,
                  DspProgramWord word,
                  DspNativeAccessKind kind,
                  std::uint64_t ordinal) const;
  [[nodiscard]] std::string context() const;
  DspProgramWord word_;
  DspFetchedInstruction fetched_{};
  std::optional<DspProgramWord> extension_;
  DspPeripheralReadForm form_;
  DspNativeAccessKind kind_ = DspNativeAccessKind::read;
  std::uint32_t omr_ = 0;
  std::uint32_t sample_omr_ = 0;
  std::uint32_t value_ = 0;
  std::uint64_t epoch_ = 0;
  std::optional<std::uint64_t> forward_epoch_;
  bool active_ = true;
};
}
