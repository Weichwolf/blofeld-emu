# SPDX-License-Identifier: GPL-3.0-only
include("${CMAKE_CURRENT_LIST_DIR}/DeviceBindings.cmake")
set(blofeld_dsp_exports "blofeld_dsp_read,blofeld_dsp_write,blofeld_dsp_bit,blofeld_dsp_reset,blofeld_dsp_pending,blofeld_dsp_revision")
blofeld_device_binding(dsp "${blofeld_dsp_exports}"
  src/vi/dsp/bindings.cpp src/vi/dsp/audio.cpp src/vi/dsp/clock.cpp
  src/vi/dsp/dax.cpp src/vi/dsp/dma.cpp src/vi/dsp/esai.cpp
  src/vi/dsp/interrupts.cpp src/vi/dsp/memory.cpp src/vi/dsp/midi.cpp
  src/vi/dsp/ports.cpp src/vi/dsp/timers.cpp src/vi/io/dsp_reset.cpp
  src/vi/io/memory_controller.cpp src/vi/storage/media_image.cpp)
add_library(blofeld_app_aot src/app/aot/dsp.cpp "${blofeld_binding_directory}/dsp_bindings.hpp")
target_include_directories(blofeld_app_aot PUBLIC src/app PRIVATE "${blofeld_binding_directory}")
target_include_directories(blofeld_app_aot SYSTEM PRIVATE ${LLVM_INCLUDE_DIRS})
target_link_libraries(blofeld_app_aot PUBLIC dsp56300::core blofeld_host_aot
  PRIVATE blofeld_vi_execution LLVM)
