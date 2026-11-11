# SPDX-License-Identifier: GPL-3.0-only
include_guard(GLOBAL)
foreach(tool llvm-link opt)
  string(REPLACE "-" "_" key "${tool}")
  find_program(BLOFELD_${key} NAMES "${tool}-19" "${tool}"
    HINTS "${BLOFELD_LLVM_BIN}" REQUIRED)
  execute_process(COMMAND "${BLOFELD_${key}}" --version OUTPUT_VARIABLE tool_version
    COMMAND_ERROR_IS_FATAL ANY)
  if(NOT tool_version MATCHES "version 19\\.")
    message(FATAL_ERROR "${tool} must be LLVM 19")
  endif()
endforeach()
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
  set(blofeld_binding_native -mcpu=native)
else()
  set(blofeld_binding_native -march=native)
endif()
set(blofeld_binding_directory "${CMAKE_CURRENT_BINARY_DIR}/generated/aot")
file(MAKE_DIRECTORY "${blofeld_binding_directory}")

function(blofeld_device_binding core exports)
  set(target "blofeld_vi_${core}_ir")
  add_library(${target} OBJECT ${ARGN})
  target_link_libraries(${target} PRIVATE blofeld_vi_execution)
  target_compile_options(${target} PRIVATE -O3 -emit-llvm ${blofeld_binding_native})
  set_target_properties(${target} PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION OFF INTERPROCEDURAL_OPTIMIZATION_RELEASE OFF)
  set(binding "${blofeld_binding_directory}/${core}_bindings.bc")
  set(header "${blofeld_binding_directory}/${core}_bindings.hpp")
  add_custom_command(OUTPUT "${header}" "${binding}"
    COMMAND "${BLOFELD_llvm_link}" $<TARGET_OBJECTS:${target}>
      -o "${blofeld_binding_directory}/${core}_linked.bc"
    COMMAND "${BLOFELD_opt}" -passes=internalize,globaldce
      "--internalize-public-api-list=${exports}"
      "${blofeld_binding_directory}/${core}_linked.bc" -o "${binding}"
    COMMAND "${BLOFELD_opt}" -passes=verify -disable-output "${binding}"
    COMMAND "${CMAKE_COMMAND}" "-DINPUT=${binding}" "-DOUTPUT=${header}"
      "-DSYMBOL=${core}_bindings" -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedBitcode.cmake"
    DEPENDS ${target} $<TARGET_OBJECTS:${target}> cmake/EmbedBitcode.cmake
    COMMAND_EXPAND_LISTS VERBATIM)
endfunction()
