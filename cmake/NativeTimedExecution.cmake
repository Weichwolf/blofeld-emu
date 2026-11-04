# SPDX-License-Identifier: GPL-3.0-only
# Generic dependency timing around real native and owned Core execution.
add_library(blofeld_dsp_timed_execution STATIC EXCLUDE_FROM_ALL
  src/dsp/dsp_native_timed_execution.cpp src/dsp/dsp_native_timed_prefix.cpp
  src/dsp/dsp_native_timed_control.cpp)
target_include_directories(blofeld_dsp_timed_execution PUBLIC src)
target_link_libraries(blofeld_dsp_timed_execution PUBLIC blofeld_dsp_dependencies
  PRIVATE blofeld_firmware)
target_link_libraries(blofeld_own_dsp_fetch PUBLIC blofeld_dsp_timed_execution)
if(BUILD_TESTING)
  add_executable(blofeld-native-dependency-timeline-test test/native_dependency_timeline_test.cpp)
  target_link_libraries(blofeld-native-dependency-timeline-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME native_dependency_timeline COMMAND blofeld-native-dependency-timeline-test)
  add_executable(blofeld-dsp-native-timed-test test/dsp_native_timed_test.cpp)
  target_link_libraries(blofeld-dsp-native-timed-test PRIVATE blofeld_dsp_timed_execution)
  add_test(NAME dsp_native_timed COMMAND blofeld-dsp-native-timed-test)
endif()
