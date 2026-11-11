# SPDX-License-Identifier: GPL-3.0-only
# Compile the actual device adapter, without native channel/owner implementations.
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
add_library(blofeld_vi_dsp_ir OBJECT
  src/vi/dsp/bindings.cpp src/vi/dsp/audio.cpp src/vi/dsp/clock.cpp
  src/vi/dsp/dax.cpp src/vi/dsp/dma.cpp src/vi/dsp/esai.cpp
  src/vi/dsp/interrupts.cpp src/vi/dsp/memory.cpp src/vi/dsp/midi.cpp
  src/vi/dsp/ports.cpp src/vi/dsp/timers.cpp src/vi/io/dsp_reset.cpp
  src/vi/io/memory_controller.cpp src/vi/storage/media_image.cpp)
target_link_libraries(blofeld_vi_dsp_ir PRIVATE blofeld_vi_execution)
target_compile_options(blofeld_vi_dsp_ir PRIVATE -O3 -emit-llvm ${blofeld_binding_native})
set_target_properties(blofeld_vi_dsp_ir PROPERTIES
  INTERPROCEDURAL_OPTIMIZATION OFF INTERPROCEDURAL_OPTIMIZATION_RELEASE OFF)
set(blofeld_binding_directory "${CMAKE_CURRENT_BINARY_DIR}/generated/aot")
file(MAKE_DIRECTORY "${blofeld_binding_directory}")
set(blofeld_dsp_binding "${blofeld_binding_directory}/dsp_bindings.bc")
set(blofeld_dsp_header "${blofeld_binding_directory}/dsp_bindings.hpp")
set(blofeld_dsp_exports "blofeld_dsp_read,blofeld_dsp_write,blofeld_dsp_bit,blofeld_dsp_reset,blofeld_dsp_pending,blofeld_dsp_revision")
add_custom_command(OUTPUT "${blofeld_dsp_header}" "${blofeld_dsp_binding}"
  COMMAND "${BLOFELD_llvm_link}" $<TARGET_OBJECTS:blofeld_vi_dsp_ir>
    -o "${blofeld_binding_directory}/dsp_linked.bc"
  COMMAND "${BLOFELD_opt}" -passes=internalize,globaldce
    "--internalize-public-api-list=${blofeld_dsp_exports}"
    "${blofeld_binding_directory}/dsp_linked.bc" -o "${blofeld_dsp_binding}"
  COMMAND "${BLOFELD_opt}" -passes=verify -disable-output "${blofeld_dsp_binding}"
  COMMAND "${CMAKE_COMMAND}" "-DINPUT=${blofeld_dsp_binding}" "-DOUTPUT=${blofeld_dsp_header}"
    -DSYMBOL=dsp_bindings -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/EmbedBitcode.cmake"
  DEPENDS blofeld_vi_dsp_ir $<TARGET_OBJECTS:blofeld_vi_dsp_ir> cmake/EmbedBitcode.cmake
  COMMAND_EXPAND_LISTS VERBATIM)
add_library(blofeld_app_aot src/app/aot/dsp.cpp "${blofeld_dsp_header}")
target_include_directories(blofeld_app_aot PUBLIC src/app PRIVATE "${blofeld_binding_directory}")
target_include_directories(blofeld_app_aot SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
target_link_libraries(blofeld_app_aot PUBLIC dsp56300::core
  PRIVATE blofeld_vi_execution blofeld_host_aot LLVM)
