// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "aot/code.hpp"

#include <mutex>
#include <vector>

namespace llvm::orc {
class LLJIT;
}

namespace blofeld::host::aot::detail {
struct SourceMap {
  void attach(llvm::orc::LLJIT &jit);
  [[nodiscard]] Code::FaultMap map() const noexcept;
  std::vector<Code::TextRange> text;
  std::vector<Code::Site> sites;
  std::mutex mutation;
};
}
