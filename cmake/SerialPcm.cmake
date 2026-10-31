# SPDX-License-Identifier: GPL-3.0-only
# Board-independent serial receiver and native DSP/ESAI test fixture.
add_library(blofeld_serial_pcm src/audio/serial_pcm.cpp src/audio/pcm_sink.cpp)
target_include_directories(blofeld_serial_pcm PUBLIC src)
target_link_libraries(blofeld_serial_pcm PUBLIC blofeld_firmware)
if(BUILD_TESTING)
  add_library(blofeld_dsp_pcm_fixture STATIC test/dsp_pcm_native.cpp)
  target_link_libraries(blofeld_dsp_pcm_fixture PUBLIC blofeld_serial_pcm blofeld_own_dsp_fetch)
  add_executable(blofeld-serial-pcm-test test/serial_pcm_test.cpp test/serial_pcm_native.cpp)
  target_link_libraries(blofeld-serial-pcm-test PRIVATE blofeld_dsp_pcm_fixture)
  add_test(NAME serial_pcm COMMAND blofeld-serial-pcm-test)
endif()
