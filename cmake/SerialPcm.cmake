# SPDX-License-Identifier: GPL-3.0-only
# Board-independent serial receiver and native DSP/ESAI test fixture.
add_library(blofeld_serial_pcm src/audio/serial_pcm.cpp src/audio/pcm_sink.cpp)
target_include_directories(blofeld_serial_pcm PUBLIC src)
target_link_libraries(blofeld_serial_pcm PUBLIC blofeld_firmware)
