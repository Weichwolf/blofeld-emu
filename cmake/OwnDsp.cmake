# SPDX-License-Identifier: GPL-3.0-only
# Pinned independent core used by the regular Board and headless runtime.
# Transitional integration names are removed with the remaining legacy harness.
set(BLOFELD_OWN_DSP_ROOT "$ENV{BLOFELD_OWN_DSP_ROOT}" CACHE PATH "Independent DSP core source checkout")
if(NOT BLOFELD_OWN_DSP_ROOT)
  set(BLOFELD_OWN_DSP_ROOT "$ENV{HOME}/Git/dsp56300")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BLOFELD_OWN_DSP_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE BLOFELD_OWN_DSP_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT BLOFELD_OWN_DSP_REVISION STREQUAL "7d87f922af058da50b7652bf0c6808e40adcb22f")
  message(FATAL_ERROR "BLOFELD_OWN_DSP_ROOT must contain the revision pinned in doc/dependencies.json")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BLOFELD_OWN_DSP_ROOT}" diff --exit-code HEAD -- src include cmake CMakeLists.txt
  RESULT_VARIABLE BLOFELD_OWN_DSP_DIRTY OUTPUT_QUIET ERROR_QUIET)
if(NOT BLOFELD_OWN_DSP_DIRTY EQUAL 0)
  message(FATAL_ERROR "The pinned independent DSP source tree has tracked modifications")
endif()
if(DEFINED DSP56300_SANITIZER AND NOT "${DSP56300_SANITIZER}" STREQUAL "${BLOFELD_SANITIZER}")
  message(FATAL_ERROR "DSP56300_SANITIZER must match BLOFELD_SANITIZER")
endif()
set(DSP56300_SANITIZER "${BLOFELD_SANITIZER}" CACHE STRING "Independent DSP sanitizer")
add_subdirectory("${BLOFELD_OWN_DSP_ROOT}" "independent-dsp" EXCLUDE_FROM_ALL)
