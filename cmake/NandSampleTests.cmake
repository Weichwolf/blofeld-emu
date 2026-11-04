# SPDX-License-Identifier: GPL-3.0-only
add_executable(blofeld-original-sample-nand-test test/original_sample_nand_test.cpp)
target_link_libraries(blofeld-original-sample-nand-test PRIVATE
  blofeld_nand_native_fixture blofeld_firmware)
add_dependencies(quality blofeld-original-sample-nand-test)
add_test(NAME original_sample_nand COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/test/original_sample_nand.py"
  $<TARGET_FILE:blofeld-original-sample-nand-test>
  "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
  "${CMAKE_CURRENT_SOURCE_DIR}/doc/library/Standard_Factory_Sample_Content_V1.0_for_License_SL_as_MID_File.zip"
  "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-sample-nand")
set_tests_properties(original_sample_nand PROPERTIES
  LABELS nand_causal_medium SKIP_RETURN_CODE 77 TIMEOUT 300)
