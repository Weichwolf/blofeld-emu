# SPDX-License-Identifier: GPL-3.0-only
include("${CMAKE_CURRENT_LIST_DIR}/DeviceBindings.cmake")
set(blofeld_mcu_exports "blofeld_mcu_read,blofeld_mcu_write,blofeld_mcu_read_word,blofeld_mcu_write_word,blofeld_mcu_read_masked")
blofeld_device_binding(mcu "${blofeld_mcu_exports}"
  src/vi/mcu/bindings.cpp src/vi/mcu/memory.cpp src/vi/mcu/boot_port.cpp
  src/vi/mcu/ports.cpp src/vi/mcu/timer.cpp src/vi/mcu/clock.cpp
  src/vi/io/sci.cpp src/vi/io/usb.cpp src/vi/io/mcu_flash.cpp
  src/vi/display/port.cpp src/vi/display/st7565.cpp src/vi/storage/flash_image.cpp)
target_sources(blofeld_app_aot PRIVATE src/app/aot/mcu.cpp
  "${blofeld_binding_directory}/mcu_bindings.hpp")
