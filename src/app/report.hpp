// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "execution.hpp"

#include <span>

namespace blofeld::app {
[[nodiscard]] bool report_execution(std::span<const vi::ExecutionReport> reports);
}
