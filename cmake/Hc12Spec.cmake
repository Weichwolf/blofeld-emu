# SPDX-License-Identifier: GPL-3.0-only
# Compile a private, corrected HC12 language description without changing Ghidra.
set(BLOFELD_HC12_LANGUAGES "${GHIDRA_ROOT}/Ghidra/Processors/HCS12/data/languages")
set(BLOFELD_SLEIGH_COMPILER
  "${GHIDRA_ROOT}/Ghidra/Features/Decompiler/os/linux_x86_64/sleigh"
  CACHE FILEPATH "Native SLEIGH compiler from the Ghidra distribution")
if(NOT EXISTS "${BLOFELD_SLEIGH_COMPILER}")
  message(FATAL_ERROR "Set BLOFELD_SLEIGH_COMPILER to the native Ghidra SLEIGH compiler")
endif()
file(SHA256 "${BLOFELD_HC12_LANGUAGES}/HCS_HC12.sinc" BLOFELD_HC12_SHA256)
if(NOT BLOFELD_HC12_SHA256 STREQUAL "7904b9b892232890287e0052198d6fab8167409654207484e407cef49919d276")
  message(FATAL_ERROR "HC12 corrections require the verified Ghidra 12.1.2 specification")
endif()
set(BLOFELD_HC12_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/patches/ghidra-hc12-branches.patch")
set(BLOFELD_HC12_INDEXED_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/patches/ghidra-hc12-indexed-word.patch")
set(BLOFELD_HC12_EXCEPTIONS_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/patches/ghidra-hc12-exceptions.patch")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${BLOFELD_HC12_PATCH}" "${BLOFELD_HC12_INDEXED_PATCH}" "${BLOFELD_HC12_EXCEPTIONS_PATCH}"
  "${BLOFELD_HC12_LANGUAGES}/HCS_HC12.sinc")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12" "${CMAKE_CURRENT_BINARY_DIR}/share")
configure_file("${BLOFELD_HC12_LANGUAGES}/HC12.slaspec" "generated/hc12/HC12.slaspec" COPYONLY)
execute_process(
  COMMAND "${BLOFELD_PATCH}" --batch --silent
    --input "${BLOFELD_HC12_PATCH}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HCS_HC12.branches.sinc"
    "${BLOFELD_HC12_LANGUAGES}/HCS_HC12.sinc"
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(
  COMMAND "${BLOFELD_PATCH}" --batch --silent
    --input "${BLOFELD_HC12_INDEXED_PATCH}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HCS_HC12.indexed.sinc"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HCS_HC12.branches.sinc"
  COMMAND_ERROR_IS_FATAL ANY)
execute_process(
  COMMAND "${BLOFELD_PATCH}" --batch --silent
    --input "${BLOFELD_HC12_EXCEPTIONS_PATCH}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HCS_HC12.sinc"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HCS_HC12.indexed.sinc"
  COMMAND_ERROR_IS_FATAL ANY)
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
  COMMAND "${BLOFELD_SLEIGH_COMPILER}"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HC12.slaspec"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HC12.slaspec"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/hc12/HCS_HC12.sinc"
    "${BLOFELD_SLEIGH_COMPILER}"
  VERBATIM)
add_custom_target(blofeld_hc12_spec DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
