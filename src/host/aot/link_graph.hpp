// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace llvm::orc {
class LLJIT;
}

namespace blofeld::host::aot::detail {
void named_dependencies(llvm::orc::LLJIT &jit);
}
