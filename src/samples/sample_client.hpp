// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <iosfwd>
#include <span>

namespace blofeld::samples {
int run_client(std::span<char *> arguments, std::ostream &output);
}
