# SPDX-License-Identifier: GPL-3.0-only
# Strict project checks; third-party source keeps its own warning policy.
set(BLOFELD_SANITIZER "address-undefined" CACHE STRING "none, address-undefined, or thread")
set_property(CACHE BLOFELD_SANITIZER PROPERTY STRINGS none address-undefined thread)
option(BLOFELD_FUZZING "Build offline libFuzzer targets" OFF)

include("${CMAKE_CURRENT_LIST_DIR}/Hardening.cmake")
add_library(blofeld_quality INTERFACE)
target_link_libraries(blofeld_quality INTERFACE blofeld_hardening)
target_compile_options(blofeld_quality INTERFACE
  -Wall -Wextra -Wpedantic -Werror -pedantic-errors
  -Wconversion -Wsign-conversion -Wshadow -Wundef
  -Wcast-align -Wcast-qual -Wdouble-promotion -Wnull-dereference
  -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast
  -Woverloaded-virtual -Wextra-semi -Wvla -Wswitch-enum -Wswitch-default
  -ffp-contract=off -fno-omit-frame-pointer)
# SLEIGH's error interface requires exceptions. They must not cross a future audio callback.
target_link_options(blofeld_quality INTERFACE
  -Wl,--fatal-warnings -Wl,-z,defs)

if(BLOFELD_SANITIZER STREQUAL "address-undefined")
  set(blofeld_sanitizers "address,undefined,float-divide-by-zero,unsigned-integer-overflow,implicit-conversion")
  set(blofeld_dependency_sanitizers "address,undefined,float-divide-by-zero")
elseif(BLOFELD_SANITIZER STREQUAL "thread")
  set(blofeld_sanitizers "thread")
  set(blofeld_dependency_sanitizers "thread")
elseif(NOT BLOFELD_SANITIZER STREQUAL "none")
  message(FATAL_ERROR "Unsupported BLOFELD_SANITIZER=${BLOFELD_SANITIZER}")
endif()
if(DEFINED blofeld_sanitizers)
  target_compile_options(blofeld_quality INTERFACE
    "-fsanitize=${blofeld_sanitizers}" -fno-sanitize-recover=all -g)
  target_link_options(blofeld_quality INTERFACE "-fsanitize=${blofeld_sanitizers}")
endif()
if(BLOFELD_FUZZING)
  if(NOT BLOFELD_SANITIZER STREQUAL "address-undefined")
    message(FATAL_ERROR "Fuzzing requires address-undefined sanitizers")
  endif()
  target_compile_options(blofeld_quality INTERFACE -fsanitize=fuzzer-no-link)
endif()
