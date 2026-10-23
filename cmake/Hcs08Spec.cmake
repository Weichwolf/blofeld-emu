# SPDX-License-Identifier: GPL-3.0-only
set(BLOFELD_HCS08_LANGUAGES "${GHIDRA_ROOT}/Ghidra/Processors/HCS08/data/languages")
foreach(hcs08_language IN ITEMS HCS08.slaspec HCS_HC.sinc)
  file(SHA256 "${BLOFELD_HCS08_LANGUAGES}/${hcs08_language}" BLOFELD_HCS08_LANGUAGE_SHA)
  if(hcs08_language STREQUAL "HCS08.slaspec")
    set(BLOFELD_HCS08_EXPECTED_SHA "4f6baf078d8d6ca7ed38f1321c7d6394af7757d42a84b636f16b5d0b5708bb10")
  else()
    set(BLOFELD_HCS08_EXPECTED_SHA "00f64687772e608c2e74f003227aa3af6a163e3e8e8c3fe98380131f7155b477")
  endif()
  if(NOT BLOFELD_HCS08_LANGUAGE_SHA STREQUAL BLOFELD_HCS08_EXPECTED_SHA)
    message(FATAL_ERROR "HCS08 execution requires the pinned Ghidra 12.1.2 ${hcs08_language}")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${BLOFELD_HCS08_LANGUAGES}/${hcs08_language}")
  configure_file("${BLOFELD_HCS08_LANGUAGES}/${hcs08_language}" "generated/hcs08/${hcs08_language}" COPYONLY)
endforeach()
set(BLOFELD_HCS08_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/patches/ghidra-hcs08-arithmetic.patch")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${BLOFELD_HCS08_PATCH}")
execute_process(
  COMMAND "${BLOFELD_PATCH}" --batch --silent --fuzz=0
    --input "${BLOFELD_HCS08_PATCH}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/generated/hcs08/HCS_HC.sinc"
    "${BLOFELD_HCS08_LANGUAGES}/HCS_HC.sinc"
  COMMAND_ERROR_IS_FATAL ANY)
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
  COMMAND "${BLOFELD_SLEIGH_COMPILER}"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/hcs08/HCS08.slaspec"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/hcs08/HCS08.slaspec"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/hcs08/HCS_HC.sinc"
    "${BLOFELD_SLEIGH_COMPILER}"
  VERBATIM)
add_custom_target(blofeld_hcs08_spec DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
