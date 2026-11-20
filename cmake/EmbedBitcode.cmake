# SPDX-License-Identifier: GPL-3.0-only
if(NOT DEFINED INPUT OR NOT DEFINED OUTPUT OR NOT DEFINED ASSEMBLY OR
   NOT SYMBOL MATCHES "^[a-z][a-z0-9_]*$")
  message(FATAL_ERROR "Specify INPUT, OUTPUT, ASSEMBLY and a valid SYMBOL for own binding IR")
endif()
file(SIZE "${INPUT}" size)
if(size EQUAL 0)
  message(FATAL_ERROR "Cannot embed empty binding bitcode")
endif()
string(REPLACE "\\" "\\\\" payload "${INPUT}")
string(REPLACE "\"" "\\\"" payload "${payload}")
set(label "blofeld_${SYMBOL}_data")

file(CONFIGURE OUTPUT "${OUTPUT}" CONTENT [=[
#pragma once
#include <cstddef>
#include <span>

namespace blofeld::app::aot::detail {
extern "C" const std::byte @label@[@size@];
inline constexpr std::span<const std::byte> @SYMBOL@{@label@};
}
]=] @ONLY)

file(CONFIGURE OUTPUT "${ASSEMBLY}" CONTENT [=[
#if defined(__APPLE__)
.section __TEXT,__const
.globl _@label@
.private_extern _@label@
_@label@:
#else
.section .rodata
.globl @label@
.hidden @label@
.type @label@,@object
@label@:
#endif
.incbin "@payload@"
#if !defined(__APPLE__)
.size @label@,.-@label@
.section .note.GNU-stack,"",@progbits
#endif
]=] @ONLY)
