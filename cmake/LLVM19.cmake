# SPDX-License-Identifier: GPL-3.0-only
find_program(LLVM19_C_COMPILER NAMES clang-19 clang
  HINTS /opt/homebrew/opt/llvm@19/bin /usr/local/opt/llvm@19/bin REQUIRED)
find_program(LLVM19_CXX_COMPILER NAMES clang++-19 clang++
  HINTS /opt/homebrew/opt/llvm@19/bin /usr/local/opt/llvm@19/bin REQUIRED)
set(CMAKE_C_COMPILER "${LLVM19_C_COMPILER}" CACHE FILEPATH "LLVM 19 C compiler")
set(CMAKE_CXX_COMPILER "${LLVM19_CXX_COMPILER}" CACHE FILEPATH "LLVM 19 C++ compiler")

if(CMAKE_HOST_APPLE AND NOT CMAKE_OSX_DEPLOYMENT_TARGET AND
   NOT DEFINED ENV{MACOSX_DEPLOYMENT_TARGET})
  execute_process(COMMAND /usr/bin/sw_vers -productVersion
    OUTPUT_VARIABLE llvm19_host_macos OUTPUT_STRIP_TRAILING_WHITESPACE
    COMMAND_ERROR_IS_FATAL ANY)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "${llvm19_host_macos}" CACHE STRING
    "Minimum macOS version for the native packaged LLVM build" FORCE)
endif()
