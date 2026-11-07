# SPDX-License-Identifier: GPL-3.0-only
# ROM fixtures need the firmware parser, not a DSP executor or assembler.
if(BUILD_TESTING)
  add_executable(blofeld-dsp-rom-program-test test/dsp_rom_program_test.cpp)
  target_link_libraries(blofeld-dsp-rom-program-test PRIVATE blofeld_firmware)
  add_test(NAME dsp_rom_program COMMAND blofeld-dsp-rom-program-test)

endif()
