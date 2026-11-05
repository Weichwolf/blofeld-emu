# SPDX-License-Identifier: GPL-3.0-only
add_executable(blofeld-original-master-history-test test/original_master_history_test.cpp)
target_link_libraries(blofeld-original-master-history-test PRIVATE
  blofeld_firmware dsp56300::core)
if(TARGET quality)
  add_dependencies(quality blofeld-original-master-history-test)
endif()
add_test(NAME original_master_history COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/original_master_history.py"
  $<TARGET_FILE:blofeld-original-master-history-test>
  "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
  "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-master-history")
set_tests_properties(original_master_history PROPERTIES
  LABELS original_dsp SKIP_RETURN_CODE 77 TIMEOUT 150)
