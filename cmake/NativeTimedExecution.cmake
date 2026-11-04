# SPDX-License-Identifier: GPL-3.0-only
# Generic dependency timing around real native and owned Core execution.
add_library(blofeld_dsp_timed_execution STATIC EXCLUDE_FROM_ALL
  src/dsp/dsp_native_timed_execution.cpp src/dsp/dsp_native_timed_prefix.cpp)
target_include_directories(blofeld_dsp_timed_execution PUBLIC src)
target_link_libraries(blofeld_dsp_timed_execution PUBLIC blofeld_dsp_dependencies
  PRIVATE blofeld_firmware)
if(BUILD_TESTING)
  add_executable(blofeld-dsp-native-timed-test test/dsp_native_timed_test.cpp)
  target_link_libraries(blofeld-dsp-native-timed-test PRIVATE blofeld_dsp_timed_execution)
  add_test(NAME dsp_native_timed COMMAND blofeld-dsp-native-timed-test)
endif()
