// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace llvm::orc {
class LLJITBuilder;
}

namespace blofeld::host::aot::detail {
void use_jit_memory(llvm::orc::LLJITBuilder &builder);
}
