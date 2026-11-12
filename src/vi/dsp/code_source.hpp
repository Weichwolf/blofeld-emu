// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "storage/dsp_records.hpp"

#include <cstddef>
#include <dsp56300/core.hpp>
#include <dsp56300/memory.hpp>
#include <span>
#include <vector>

namespace blofeld::vi::dsp {
struct CodeRange {
  dsp56300::Word address = 0;
  std::size_t words = 0;
};

// Cold logical P-space source view after explicitly ordered stored writes.
// Ranges declare instruction contexts, not all readable words or physical aliases.
// This neither loads the device nor treats a record as an atomic runtime write.
class CodeSource {
public:
  CodeSource(std::span<const DspRecord> records, std::span<const CodeRange> ranges);
  CodeSource(std::span<const dsp56300::CodeSpan> writes, std::span<const CodeRange> ranges);
  CodeSource(const CodeSource &) = delete;
  CodeSource &operator=(const CodeSource &) = delete;
  CodeSource(CodeSource &&) noexcept = default;
  CodeSource &operator=(CodeSource &&) noexcept = default;

  [[nodiscard]] std::span<const dsp56300::CodeSpan> code() const noexcept { return code_; }

private:
  void index(std::span<const CodeRange> ranges);
  std::vector<std::vector<dsp56300::Word>> words_;
  std::vector<dsp56300::CodeSpan> code_;
};
}
