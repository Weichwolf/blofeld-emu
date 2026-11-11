# SPDX-License-Identifier: GPL-3.0-only
if(NOT DEFINED INPUT OR NOT DEFINED OUTPUT OR NOT SYMBOL MATCHES "^[a-z][a-z0-9_]*$")
  message(FATAL_ERROR "Specify INPUT, OUTPUT and a valid SYMBOL for own binding IR")
endif()
file(READ "${INPUT}" payload HEX)
string(LENGTH "${payload}" digits)
if(digits EQUAL 0)
  message(FATAL_ERROR "Cannot embed empty binding bitcode")
endif()
math(EXPR size "${digits} / 2")
string(REGEX REPLACE "(..)" "std::byte{0x\\1}," values "${payload}")
file(WRITE "${OUTPUT}"
  "// Generated from our own binding IR; do not edit.\n#pragma once\n"
  "#include <array>\n#include <cstddef>\n"
  "namespace blofeld::app::aot::detail {\n"
  "inline constexpr std::array<std::byte, ${size}> ${SYMBOL}{${values}};\n}\n")
