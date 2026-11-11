// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace blofeld::test::aot {
struct Calls {
  unsigned machines = 0, engines = 0, objects = 0, helpers = 0, modules = 0;
  bool operator==(const Calls &) const = default;
};

[[nodiscard]] Calls calls() noexcept;
}
