// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <functional>

namespace blofeld::detail {
// Linux exposes the current grant, not a maximum. On an unused client, probe
// bounded preferences and read each actual grant; handle ignored or clamped
// requests. Two event cells are the minimum for a SysEx record plus payload.
int negotiate_sequence_pool(int preferred, int current, const std::function<int(int)> &request);
}
