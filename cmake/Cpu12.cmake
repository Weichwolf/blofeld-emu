# SPDX-License-Identifier: GPL-3.0-only
# The caller supplies an immutable local checkout of the qualified CPU library.
set(BLOFELD_CPU12_ROOT "$ENV{BLOFELD_CPU12_ROOT}" CACHE PATH "Independent CPU12 source checkout")
if(NOT BLOFELD_CPU12_ROOT)
  set(BLOFELD_CPU12_ROOT "$ENV{HOME}/Git/cpu12")
endif()
file(READ "${CMAKE_CURRENT_LIST_DIR}/dependencies.json" blofeld_dependencies)
string(JSON BLOFELD_CPU12_PIN GET "${blofeld_dependencies}" independent_cpu12 revision)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/dependencies.json")
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BLOFELD_CPU12_ROOT}" rev-parse HEAD
  OUTPUT_VARIABLE BLOFELD_CPU12_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT BLOFELD_CPU12_REVISION STREQUAL BLOFELD_CPU12_PIN)
  message(FATAL_ERROR "BLOFELD_CPU12_ROOT must contain the revision pinned in cmake/dependencies.json")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${BLOFELD_CPU12_ROOT}"
  diff --exit-code HEAD -- src include cmake CMakeLists.txt
  RESULT_VARIABLE BLOFELD_CPU12_DIRTY OUTPUT_QUIET ERROR_QUIET)
if(NOT BLOFELD_CPU12_DIRTY EQUAL 0)
  message(FATAL_ERROR "The pinned independent CPU12 source tree has tracked modifications")
endif()
if(DEFINED CPU12_SANITIZER AND NOT "${CPU12_SANITIZER}" STREQUAL "${BLOFELD_SANITIZER}")
  message(FATAL_ERROR "CPU12_SANITIZER must match BLOFELD_SANITIZER")
endif()
set(CPU12_SANITIZER "${BLOFELD_SANITIZER}" CACHE STRING "Independent CPU12 sanitizer")
add_subdirectory("${BLOFELD_CPU12_ROOT}" "independent-cpu12" EXCLUDE_FROM_ALL)
