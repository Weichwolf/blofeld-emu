# SPDX-License-Identifier: GPL-3.0-only
find_program(LLVM19_C_COMPILER NAMES clang-19 clang
  HINTS /opt/homebrew/opt/llvm@19/bin /usr/local/opt/llvm@19/bin REQUIRED)
find_program(LLVM19_CXX_COMPILER NAMES clang++-19 clang++
  HINTS /opt/homebrew/opt/llvm@19/bin /usr/local/opt/llvm@19/bin REQUIRED)
set(CMAKE_C_COMPILER "${LLVM19_C_COMPILER}" CACHE FILEPATH "LLVM 19 C compiler")
set(CMAKE_CXX_COMPILER "${LLVM19_CXX_COMPILER}" CACHE FILEPATH "LLVM 19 C++ compiler")
