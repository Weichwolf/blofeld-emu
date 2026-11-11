// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "llvm/IR/Attributes.h"

#include <array>

namespace blofeld::host::aot::detail {
inline constexpr std::array abi_attributes{llvm::Attribute::SExt,
                                           llvm::Attribute::ZExt,
                                           llvm::Attribute::InReg,
                                           llvm::Attribute::StructRet,
                                           llvm::Attribute::ByVal,
                                           llvm::Attribute::ByRef,
                                           llvm::Attribute::InAlloca,
                                           llvm::Attribute::Preallocated,
                                           llvm::Attribute::Nest,
                                           llvm::Attribute::SwiftSelf,
                                           llvm::Attribute::SwiftError,
                                           llvm::Attribute::SwiftAsync,
                                           llvm::Attribute::Returned};
}
