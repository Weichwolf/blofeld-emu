# SPDX-License-Identifier: GPL-3.0-only
# Direct word output; bit decoding belongs to independent test support.
add_library(blofeld_serial_pcm src/audio/serial_pcm.cpp src/audio/pcm_sink.cpp src/audio/pcm_words.cpp)
target_include_directories(blofeld_serial_pcm PUBLIC src)
target_link_libraries(blofeld_serial_pcm PUBLIC blofeld_firmware)

if(BUILD_TESTING)
  add_library(blofeld_pcm_reference STATIC test/support/serial_pcm.cpp)
  target_include_directories(blofeld_pcm_reference PUBLIC test)
  target_link_libraries(blofeld_pcm_reference PUBLIC blofeld_serial_pcm)
  add_executable(blofeld-pcm-words-test test/audio/pcm_words_test.cpp)
  target_link_libraries(blofeld-pcm-words-test PRIVATE blofeld_pcm_reference blofeld_dsp_peripherals)
  add_test(NAME pcm_words COMMAND blofeld-pcm-words-test)
  add_executable(blofeld-esai-span-test test/audio/esai_span_test.cpp)
  target_link_libraries(blofeld-esai-span-test PRIVATE blofeld_pcm_reference blofeld_dsp_peripherals)
  add_test(NAME dsp_esai_spans COMMAND blofeld-esai-span-test)
endif()
