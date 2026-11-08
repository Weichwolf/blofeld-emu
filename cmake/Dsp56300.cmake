# SPDX-License-Identifier: GPL-3.0-only
# Pinned independent core used by the instrument and headless runtime.
set(BLOFELD_DSP56300_ROOT "$ENV{BLOFELD_DSP56300_ROOT}" CACHE PATH "Independent DSP core source checkout")
if(NOT BLOFELD_DSP56300_ROOT)
  set(BLOFELD_DSP56300_ROOT "$ENV{HOME}/Git/dsp56300")
endif()
file(READ "${CMAKE_CURRENT_LIST_DIR}/dependencies.json" blofeld_dependencies)
string(JSON BLOFELD_DSP56300_PIN GET "${blofeld_dependencies}" independent_dsp56300 revision)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/dependencies.json")
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BLOFELD_DSP56300_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE BLOFELD_DSP56300_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT BLOFELD_DSP56300_REVISION STREQUAL BLOFELD_DSP56300_PIN)
  message(FATAL_ERROR "BLOFELD_DSP56300_ROOT must contain the revision pinned in cmake/dependencies.json")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BLOFELD_DSP56300_ROOT}" diff --exit-code HEAD -- src include cmake CMakeLists.txt
  RESULT_VARIABLE BLOFELD_DSP56300_DIRTY OUTPUT_QUIET ERROR_QUIET)
if(NOT BLOFELD_DSP56300_DIRTY EQUAL 0)
  message(FATAL_ERROR "The pinned independent DSP source tree has tracked modifications")
endif()
if(DEFINED DSP56300_SANITIZER AND NOT "${DSP56300_SANITIZER}" STREQUAL "${BLOFELD_SANITIZER}")
  message(FATAL_ERROR "DSP56300_SANITIZER must match BLOFELD_SANITIZER")
endif()
set(DSP56300_SANITIZER "${BLOFELD_SANITIZER}" CACHE STRING "Independent DSP sanitizer")
add_subdirectory("${BLOFELD_DSP56300_ROOT}" "independent-dsp" EXCLUDE_FROM_ALL)
