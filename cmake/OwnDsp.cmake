# SPDX-License-Identifier: GPL-3.0-only
# Transitional integration of the independent DSP core. The existing runtime
# still uses the legacy core until its scheduler and pipeline are migrated.
set(BLOFELD_OWN_DSP_ROOT "$ENV{BLOFELD_OWN_DSP_ROOT}" CACHE PATH "Independent DSP core source checkout")
if(NOT BLOFELD_OWN_DSP_ROOT)
  set(BLOFELD_OWN_DSP_ROOT "$ENV{HOME}/Git/dsp56300")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BLOFELD_OWN_DSP_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE BLOFELD_OWN_DSP_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT BLOFELD_OWN_DSP_REVISION STREQUAL "393e317f761db19a2e59890e7f395c0d2ec75f8c")
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
