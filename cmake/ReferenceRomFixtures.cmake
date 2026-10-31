# SPDX-License-Identifier: GPL-3.0-only
# ROM fixtures need the firmware parser, not a DSP executor or assembler.
if(BUILD_TESTING)
  add_executable(blofeld-dsp-rom-program-test test/dsp_rom_program_test.cpp)
  target_link_libraries(blofeld-dsp-rom-program-test PRIVATE blofeld_firmware)
  add_test(NAME dsp_rom_program COMMAND blofeld-dsp-rom-program-test)
  add_executable(blofeld-reference-original-midi-rom test/reference_original_midi_rom.cpp)
  target_link_libraries(blofeld-reference-original-midi-rom PRIVATE blofeld_firmware)
  add_test(NAME reference_midi_rom_fixture COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_midi_rom_fixture.py"
    $<TARGET_FILE:blofeld-reference-original-midi-rom>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-midi-rom.bin")
  set_tests_properties(reference_midi_rom_fixture PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 40)
endif()
