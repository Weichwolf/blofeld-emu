# SPDX-License-Identifier: GPL-3.0-only
# Strict project checks; third-party source keeps its own warning policy.
set(BLOFELD_SANITIZER "address-undefined" CACHE STRING "none, address-undefined, or thread")
set_property(CACHE BLOFELD_SANITIZER PROPERTY STRINGS none address-undefined thread)

include("${CMAKE_CURRENT_LIST_DIR}/Hardening.cmake")
add_library(blofeld_diagnostics INTERFACE)
target_link_libraries(blofeld_diagnostics INTERFACE blofeld_hardening)
target_compile_options(blofeld_diagnostics INTERFACE
  -Wall -Wextra -Wpedantic -Werror -pedantic-errors
  -Wconversion -Wsign-conversion -Wshadow -Wundef
  -Wcast-align -Wcast-qual -Wdouble-promotion -Wnull-dereference
  -Wimplicit-fallthrough -Wnon-virtual-dtor -Wold-style-cast
  -Woverloaded-virtual -Wextra-semi -Wvla -Wswitch-enum -Wswitch-default
  -ffp-contract=off -fno-omit-frame-pointer)
add_library(blofeld_quality INTERFACE)
target_link_libraries(blofeld_quality INTERFACE blofeld_diagnostics)
# Contextual control-path exceptions must never cross the audio callback.
if(APPLE)
  target_link_options(blofeld_quality INTERFACE -Wl,-fatal_warnings)
  get_filename_component(BLOFELD_LLVM_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)
  find_library(BLOFELD_EXPERIMENTAL_LIBRARY NAMES c++experimental
    HINTS "${BLOFELD_LLVM_BIN}/../lib/c++" REQUIRED)
  target_compile_options(blofeld_quality INTERFACE -fexperimental-library)
  target_link_libraries(blofeld_quality INTERFACE "${BLOFELD_EXPERIMENTAL_LIBRARY}")
else()
  target_link_options(blofeld_quality INTERFACE -Wl,--fatal-warnings -Wl,-z,defs)
endif()

if(BLOFELD_SANITIZER STREQUAL "address-undefined")
  set(blofeld_sanitizers "address,undefined,float-divide-by-zero,unsigned-integer-overflow,implicit-conversion")
elseif(BLOFELD_SANITIZER STREQUAL "thread")
  set(blofeld_sanitizers "thread")
elseif(NOT BLOFELD_SANITIZER STREQUAL "none")
  message(FATAL_ERROR "Unsupported BLOFELD_SANITIZER=${BLOFELD_SANITIZER}")
endif()
if(DEFINED blofeld_sanitizers)
  target_compile_options(blofeld_quality INTERFACE
    "-fsanitize=${blofeld_sanitizers}" -fno-sanitize-recover=all -g)
  target_link_options(blofeld_quality INTERFACE "-fsanitize=${blofeld_sanitizers}")
endif()
