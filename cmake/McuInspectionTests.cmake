# SPDX-License-Identifier: GPL-3.0-only
if(NOT BUILD_TESTING)
  return()
endif()
add_executable(blofeld-mcu-memory-inspection-test test/mcu_memory_inspection_test.cpp)
target_link_libraries(blofeld-mcu-memory-inspection-test PRIVATE blofeld_mcu blofeld_quality)
add_test(NAME mcu_memory_inspection COMMAND blofeld-mcu-memory-inspection-test)
add_executable(blofeld-mcu-pim-input-test test/mcu_pim_input_test.cpp)
target_link_libraries(blofeld-mcu-pim-input-test PRIVATE blofeld_mcu blofeld_quality)
add_test(NAME mcu_pim_input COMMAND blofeld-mcu-pim-input-test)
add_executable(blofeld-reference-memory-inspection-test test/reference_memory_inspection_test.cpp)
target_link_libraries(blofeld-reference-memory-inspection-test PRIVATE blofeld_reference_board blofeld_quality)
add_test(NAME reference_memory_inspection COMMAND blofeld-reference-memory-inspection-test)
set_tests_properties(mcu_memory_inspection mcu_pim_input reference_memory_inspection
  PROPERTIES TIMEOUT 60)
if(TARGET quality)
  add_dependencies(quality blofeld-mcu-memory-inspection-test blofeld-mcu-pim-input-test
    blofeld-reference-memory-inspection-test)
endif()
