# SPDX-License-Identifier: GPL-3.0-only
# Shared by the regular build and the independent-DSP integration build.
add_library(blofeld_firmware src/firmware/firmware.cpp src/firmware/archive.cpp src/firmware/midi_file.cpp src/common/diagnostic.cpp src/common/bus.cpp)
target_include_directories(blofeld_firmware PUBLIC src)
target_link_libraries(blofeld_firmware PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_gpio_pads src/board/gpio_pads.cpp)
target_include_directories(blofeld_gpio_pads PUBLIC src)
target_link_libraries(blofeld_gpio_pads PUBLIC blofeld_firmware)
add_library(blofeld_dsp_peripherals src/dsp/dsp_shi.cpp src/dsp/dsp_shi_spi.cpp src/dsp/dsp_bus.cpp src/dsp/dsp_pll.cpp
  src/dsp/dsp_esai.cpp src/dsp/dsp_esai_transmit.cpp src/dsp/dsp_esai_clock.cpp src/dsp/dsp_bus_esai.cpp
  src/dsp/dsp_dma.cpp src/dsp/dsp_dma_address.cpp src/dsp/dsp_dma_cycle.cpp src/dsp/dsp_bus_dma.cpp
  src/dsp/dsp_dax.cpp src/dsp/dsp_dax_clock.cpp src/dsp/dsp_bus_dax.cpp
  src/dsp/dsp_gpio.cpp src/dsp/dsp_timers.cpp src/dsp/dsp_timer_clock.cpp src/dsp/dsp_timer_capture.cpp src/dsp/dsp_interrupts.cpp
  src/dsp/dsp_run_clock.cpp src/dsp/dsp_run_reset.cpp)
target_include_directories(blofeld_dsp_peripherals PUBLIC src)
target_link_libraries(blofeld_dsp_peripherals PUBLIC blofeld_firmware blofeld_gpio_pads)
add_library(blofeld_own_dsp_io STATIC EXCLUDE_FROM_ALL src/dsp/own_dsp_io.cpp
  src/dsp/own_dsp_interrupts.cpp src/dsp/own_dsp_machine.cpp)
target_include_directories(blofeld_own_dsp_io PUBLIC src)
target_link_libraries(blofeld_own_dsp_io PUBLIC blofeld_dsp_peripherals dsp56300::core)
add_library(blofeld_dsp_branch_decode src/dsp/dsp_branch_form.cpp)
target_include_directories(blofeld_dsp_branch_decode PUBLIC src)
target_link_libraries(blofeld_dsp_branch_decode PUBLIC blofeld_firmware dsp56300::core)
add_library(blofeld_dsp_address_decode src/dsp/dsp_address_only.cpp)
target_include_directories(blofeld_dsp_address_decode PUBLIC src)
target_link_libraries(blofeld_dsp_address_decode PUBLIC blofeld_firmware dsp56300::core)
add_library(blofeld_dsp_peripheral_decode src/dsp/dsp_peripheral_read_form.cpp)
target_include_directories(blofeld_dsp_peripheral_decode PUBLIC src)
target_link_libraries(blofeld_dsp_peripheral_decode PUBLIC blofeld_dsp_peripherals dsp56300::core)
add_library(blofeld_dsp_control_decode src/dsp/dsp_control_move.cpp src/dsp/dsp_omr_move.cpp
  src/dsp/dsp_sp_move.cpp src/dsp/dsp_sr_move.cpp)
target_include_directories(blofeld_dsp_control_decode PUBLIC src)
target_link_libraries(blofeld_dsp_control_decode PUBLIC blofeld_firmware dsp56300::core)
