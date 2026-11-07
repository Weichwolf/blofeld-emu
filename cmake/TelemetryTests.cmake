# SPDX-License-Identifier: GPL-3.0-only
if(NOT BUILD_TESTING)
  return()
endif()
add_executable(blofeld-telemetry-test test/telemetry_test.cpp)
target_include_directories(blofeld-telemetry-test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(blofeld-telemetry-test PRIVATE blofeld_quality)
add_test(NAME telemetry_lazy_output COMMAND blofeld-telemetry-test)

if(TARGET quality)
add_dependencies(quality blofeld-telemetry-test)
endif()
