# SPDX-License-Identifier: GPL-3.0-only
add_executable(blofeld-original-nand-status-test test/original_nand_status_test.cpp)
target_link_libraries(blofeld-original-nand-status-test PRIVATE
  blofeld_nand_native_fixture blofeld_firmware)
add_dependencies(quality blofeld-original-nand-status-test)
add_test(NAME original_nand_status COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/original_nand_status.py"
  $<TARGET_FILE:blofeld-original-nand-status-test>
  "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
  "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-status.trace")
set_tests_properties(original_nand_status PROPERTIES
  LABELS nand_causal_medium SKIP_RETURN_CODE 77 TIMEOUT 120)
