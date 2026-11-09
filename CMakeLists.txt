# SPDX-License-Identifier: GPL-3.0-only
cmake_minimum_required(VERSION 3.25)
project(blofeld_emu VERSION 0.1.0 LANGUAGES C CXX)

include(cmake/CompilerCache.cmake)
include(CTest)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
foreach(language C CXX)
  if(NOT CMAKE_${language}_COMPILER_ID STREQUAL "Clang" OR
     NOT CMAKE_${language}_COMPILER_VERSION VERSION_GREATER_EQUAL 19 OR
     NOT CMAKE_${language}_COMPILER_VERSION VERSION_LESS 20)
    message(FATAL_ERROR "Use LLVM 19 for C and C++: cmake --preset strict")
  endif()
endforeach()

include(cmake/Quality.cmake)
get_filename_component(BLOFELD_LLVM_BIN "${CMAKE_CXX_COMPILER}" DIRECTORY)
find_program(BLOFELD_CLANG_TIDY NAMES clang-tidy-19 clang-tidy HINTS "${BLOFELD_LLVM_BIN}" REQUIRED)
find_program(BLOFELD_CLANG_FORMAT NAMES clang-format-19 clang-format HINTS "${BLOFELD_LLVM_BIN}" REQUIRED)
foreach(tool BLOFELD_CLANG_TIDY BLOFELD_CLANG_FORMAT)
  execute_process(COMMAND "${${tool}}" --version OUTPUT_VARIABLE tool_version
    COMMAND_ERROR_IS_FATAL ANY)
  if(NOT tool_version MATCHES "version 19\\.")
    message(FATAL_ERROR "${tool} must be LLVM 19")
  endif()
endforeach()
find_package(Python3 3.11 REQUIRED COMPONENTS Interpreter)
find_package(Git REQUIRED)
find_package(Threads REQUIRED)

# Core correctness suites run in their own projects, independently of the app.
set(CPU12_BUILD_TESTING OFF)
set(DSP56300_BUILD_TESTING OFF)
include(cmake/Cpu12.cmake)
include(cmake/Dsp56300.cmake)
include(cmake/Zlib.cmake)
find_program(BLOFELD_PATCH NAMES patch REQUIRED)
if(APPLE)
  find_library(BLOFELD_COREAUDIO CoreAudio REQUIRED)
  find_library(BLOFELD_AUDIOTOOLBOX AudioToolbox REQUIRED)
  find_library(BLOFELD_COREFOUNDATION CoreFoundation REQUIRED)
  set(blofeld_playback src/host/audio/playback_coreaudio.cpp src/host/audio/device_buffer.cpp
    src/host/audio/producer_coreaudio.cpp)
  set(blofeld_signals src/host/process/signals_darwin.cpp)
else()
  include(cmake/PulseAudio.cmake)
  set(blofeld_playback src/host/audio/playback.cpp src/host/audio/producer.cpp)
  set(blofeld_signals src/host/process/signals.cpp)
endif()
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

add_library(blofeld_vi_display src/vi/display/st7565.cpp src/vi/display/lcd_image.cpp src/vi/display/state.cpp)
target_include_directories(blofeld_vi_display PUBLIC src/vi)
target_link_libraries(blofeld_vi_display PUBLIC blofeld_quality Threads::Threads)
add_library(blofeld_host_files src/host/files/directory.cpp src/host/files/read.cpp)
target_include_directories(blofeld_host_files PUBLIC src/host)
target_link_libraries(blofeld_host_files PUBLIC blofeld_quality)
add_library(blofeld_app_display src/app/display/publication.cpp)
target_include_directories(blofeld_app_display PUBLIC src/app)
target_link_libraries(blofeld_app_display PUBLIC blofeld_vi_display blofeld_host_files)

add_library(blofeld_host_archive src/host/archive/zip.cpp)
target_include_directories(blofeld_host_archive PUBLIC src/host)
target_link_libraries(blofeld_host_archive PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_vi_firmware src/vi/storage/firmware.cpp src/vi/storage/update_midi.cpp)
target_include_directories(blofeld_vi_firmware PUBLIC src/vi)
target_link_libraries(blofeld_vi_firmware PUBLIC blofeld_quality)
add_library(blofeld_app_firmware src/app/firmware.cpp)
target_include_directories(blofeld_app_firmware PUBLIC src/app)
target_link_libraries(blofeld_app_firmware PUBLIC
  blofeld_vi_firmware blofeld_host_files blofeld_host_archive)
add_library(blofeld_vi_storage src/vi/storage/flash_image.cpp src/vi/storage/media_image.cpp)
target_include_directories(blofeld_vi_storage PUBLIC src/vi)
target_link_libraries(blofeld_vi_storage PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_vi_mcu_memory src/vi/mcu/memory.cpp)
target_include_directories(blofeld_vi_mcu_memory PUBLIC src/vi)
target_link_libraries(blofeld_vi_mcu_memory PUBLIC blofeld_vi_storage cpu12::core)
add_library(blofeld_vi_peer src/vi/io/peer_port.cpp src/vi/io/execution_time.cpp)
target_include_directories(blofeld_vi_peer PUBLIC src/vi)
target_link_libraries(blofeld_vi_peer PUBLIC blofeld_quality Threads::Threads)
add_library(blofeld_vi_midi src/vi/io/midi_port.cpp)
target_include_directories(blofeld_vi_midi PUBLIC src/vi)
target_link_libraries(blofeld_vi_midi PUBLIC blofeld_quality)
add_library(blofeld_vi_execution
  src/vi/display/port.cpp
  src/vi/io/memory_controller.cpp src/vi/io/dsp_reset.cpp src/vi/io/shi.cpp src/vi/io/sci.cpp src/vi/io/usb.cpp src/vi/io/mcu_flash.cpp
  src/vi/mcu/boot_port.cpp src/vi/mcu/ports.cpp src/vi/mcu/timer.cpp src/vi/mcu/clock.cpp
  src/vi/mcu/execute.cpp src/vi/dsp/audio.cpp src/vi/dsp/clock.cpp src/vi/dsp/dax.cpp src/vi/dsp/dma.cpp src/vi/dsp/esai.cpp src/vi/dsp/interrupts.cpp src/vi/dsp/memory.cpp src/vi/dsp/ports.cpp src/vi/dsp/timers.cpp src/vi/dsp/execute.cpp src/vi/vm_context.cpp)
target_include_directories(blofeld_vi_execution PUBLIC src/vi)
target_link_libraries(blofeld_vi_execution PUBLIC
  blofeld_vi_mcu_memory blofeld_vi_display blofeld_vi_peer blofeld_vi_midi dsp56300::core Threads::Threads)
add_library(blofeld_app_profile src/app/profile.cpp)
add_library(blofeld_app_boot src/app/boot_rom.cpp)
target_include_directories(blofeld_app_boot PUBLIC src/app)
target_link_libraries(blofeld_app_boot PUBLIC blofeld_host_files)
add_library(blofeld_host_process src/host/process/configuration.cpp ${blofeld_signals})
target_include_directories(blofeld_host_process PUBLIC src/host)
target_link_libraries(blofeld_host_process PUBLIC blofeld_quality Threads::Threads)
target_include_directories(blofeld_app_profile PUBLIC src/app)
target_link_libraries(blofeld_app_profile PUBLIC blofeld_host_files blofeld_host_process)
add_library(blofeld_host_audio src/host/audio/buffer.cpp src/host/audio/render_buffer.cpp ${blofeld_playback})
target_include_directories(blofeld_host_audio PUBLIC src/host)
target_link_libraries(blofeld_host_audio PUBLIC blofeld_quality Threads::Threads)
if(APPLE)
  target_link_libraries(blofeld_host_audio PRIVATE
    "${BLOFELD_COREAUDIO}" "${BLOFELD_AUDIOTOOLBOX}" "${BLOFELD_COREFOUNDATION}")
else()
  target_link_libraries(blofeld_host_audio PRIVATE blofeld_pulse)
endif()
add_library(blofeld_host_wav src/host/audio/wav.cpp)
target_include_directories(blofeld_host_wav PUBLIC src/host)
target_link_libraries(blofeld_host_wav PUBLIC blofeld_quality)
add_library(blofeld_app_audio src/app/audio/consumer.cpp)
target_include_directories(blofeld_app_audio PUBLIC src/app)
target_link_libraries(blofeld_app_audio PUBLIC blofeld_host_audio blofeld_vi_execution)
add_library(blofeld_app_wav src/app/audio/wav_consumer.cpp)
target_include_directories(blofeld_app_wav PUBLIC src/app)
target_link_libraries(blofeld_app_wav PUBLIC blofeld_host_wav blofeld_vi_execution)
add_executable(blofeld-headless src/app/main.cpp)
target_include_directories(blofeld-headless PRIVATE src/app)
target_link_libraries(blofeld-headless PRIVATE blofeld_app_boot blofeld_app_profile blofeld_app_audio blofeld_app_display)

file(GLOB_RECURSE BLOFELD_FORMAT_SOURCES CONFIGURE_DEPENDS
  "src/*.hpp" "src/*.cpp" "test/*.hpp" "test/*.cpp")
add_custom_target(format-check
  COMMAND "${BLOFELD_CLANG_FORMAT}" --dry-run --Werror ${BLOFELD_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
  function(blofeld_unit name source library)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ${library})
    add_test(NAME ${name} COMMAND ${name} "${CMAKE_CURRENT_BINARY_DIR}/Testing/${name}" ${ARGN})
    set_tests_properties(${name} PROPERTIES TIMEOUT 10)
    set_property(GLOBAL APPEND PROPERTY BLOFELD_UNIT_TARGETS ${name})
  endfunction()
  blofeld_unit(host-audio-buffer-test test/host/audio/buffer_test.cpp blofeld_host_audio)
  blofeld_unit(app-boot-test test/app/boot_rom_test.cpp blofeld_app_boot)
  blofeld_unit(host-audio-render-test test/host/audio/render_buffer_test.cpp blofeld_host_audio)
  blofeld_unit(host-audio-wav-test test/host/audio/wav_test.cpp "blofeld_host_wav;blofeld_host_files")
  blofeld_unit(app-audio-wav-test test/app/audio/wav_consumer_test.cpp "blofeld_app_wav;blofeld_host_files")
  blofeld_unit(host-audio-playback-test test/host/audio/playback_test.cpp blofeld_host_audio)
  if(APPLE)
    add_library(blofeld_coreaudio_absent SHARED test/host/audio/coreaudio_absent.cpp)
    target_link_libraries(blofeld_coreaudio_absent PRIVATE blofeld_quality "${BLOFELD_COREAUDIO}")
    add_dependencies(host-audio-playback-test blofeld_coreaudio_absent)
    set_tests_properties(host-audio-playback-test PROPERTIES
      ENVIRONMENT "DYLD_INSERT_LIBRARIES=$<TARGET_FILE:blofeld_coreaudio_absent>")
  else()
    set_tests_properties(host-audio-playback-test PROPERTIES
      ENVIRONMENT "PULSE_SERVER=unix:${CMAKE_CURRENT_BINARY_DIR}/Testing/absent-audio-socket")
  endif()
  blofeld_unit(host-signals-test test/host/process/signals_test.cpp blofeld_host_process)
  blofeld_unit(vi-display-test test/vi/display/st7565_test.cpp blofeld_vi_display)
  blofeld_unit(vi-display-port-test test/vi/display/port_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-peer-port-test test/vi/io/peer_port_test.cpp blofeld_vi_peer)
  blofeld_unit(vi-midi-port-test test/vi/io/midi_port_test.cpp "blofeld_vi_midi;Threads::Threads")
  blofeld_unit(vi-mcu-midi-test test/vi/mcu/midi_test.cpp blofeld_vi_execution)
  target_include_directories(vi-mcu-midi-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  blofeld_unit(vi-execution-time-test test/vi/io/execution_time_test.cpp blofeld_vi_peer)
  blofeld_unit(vi-display-state-test test/vi/display/state_test.cpp blofeld_vi_display)
  blofeld_unit(vi-lcd-image-test test/vi/display/lcd_image_test.cpp blofeld_vi_display)
  blofeld_unit(host-directory-test test/host/files/directory_test.cpp blofeld_host_files)
  blofeld_unit(app-display-test test/app/display/publication_test.cpp blofeld_app_display)
  blofeld_unit(host-read-test test/host/files/read_test.cpp blofeld_host_files)
  set(blofeld_unit_inputs "${CMAKE_CURRENT_BINARY_DIR}/generated/unit-inputs")
  add_custom_command(OUTPUT "${blofeld_unit_inputs}/zip.done" "${blofeld_unit_inputs}/firmware.done"
    BYPRODUCTS "${blofeld_unit_inputs}/media-writable.bfnand"
      "${blofeld_unit_inputs}/media-programmable.bfnand"
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test/host/archive/fixtures.py"
      "${blofeld_unit_inputs}"
    COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test/vi/storage/fixtures.py"
      "${blofeld_unit_inputs}"
    DEPENDS test/host/archive/fixtures.py test/vi/storage/fixtures.py VERBATIM)
  add_custom_target(unit-inputs
    DEPENDS "${blofeld_unit_inputs}/zip.done" "${blofeld_unit_inputs}/firmware.done")
  blofeld_unit(host-zip-test test/host/archive/zip_test.cpp
    "blofeld_host_archive;blofeld_host_files" "${blofeld_unit_inputs}")
  blofeld_unit(vi-firmware-test test/vi/storage/firmware_test.cpp
    blofeld_vi_firmware "${blofeld_unit_inputs}")
  blofeld_unit(app-firmware-test test/app/firmware_test.cpp
    blofeld_app_firmware "${blofeld_unit_inputs}")
  add_dependencies(host-zip-test unit-inputs)
  add_dependencies(vi-firmware-test unit-inputs)
  add_dependencies(app-firmware-test unit-inputs)
  blofeld_unit(vi-flash-image-test test/vi/storage/flash_image_test.cpp
    blofeld_vi_storage "${blofeld_unit_inputs}")
  add_dependencies(vi-flash-image-test unit-inputs)
  blofeld_unit(vi-media-image-test test/vi/storage/media_image_test.cpp
    "blofeld_vi_storage;blofeld_host_files" "${blofeld_unit_inputs}")
  blofeld_unit(vi-memory-controller-test test/vi/io/memory_controller_test.cpp
    "blofeld_vi_execution;blofeld_host_files" "${blofeld_unit_inputs}")
  add_dependencies(vi-media-image-test unit-inputs)
  add_dependencies(vi-memory-controller-test unit-inputs)
  blofeld_unit(vi-dsp-ports-test test/vi/dsp/ports_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-mcu-memory-test test/vi/mcu/memory_test.cpp blofeld_vi_mcu_memory)
  target_include_directories(vi-mcu-memory-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  blofeld_unit(vi-vm-context-test test/vi/vm_context_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-usb-test test/vi/io/usb_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-sci-test test/vi/io/sci_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-mcu-execute-test test/vi/mcu/execute_test.cpp blofeld_vi_execution)
  target_include_directories(vi-mcu-execute-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  blofeld_unit(vi-dsp-memory-test test/vi/dsp/memory_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-clock-test test/vi/dsp/clock_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-dma-test test/vi/dsp/dma_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-esai-test test/vi/dsp/esai_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-audio-test test/vi/dsp/audio_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-dax-test test/vi/dsp/dax_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-timers-test test/vi/dsp/timers_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-interrupts-test test/vi/dsp/interrupts_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-dsp-execute-test test/vi/dsp/execute_test.cpp
    "blofeld_vi_execution;blofeld_host_files" "${blofeld_unit_inputs}")
  add_dependencies(vi-dsp-execute-test unit-inputs)
  target_include_directories(vi-dsp-execute-test PRIVATE "${BLOFELD_DSP56300_ROOT}/test")
  blofeld_unit(vi-dsp-reset-test test/vi/io/dsp_reset_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-shi-test test/vi/io/shi_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-mcu-flash-test test/vi/io/mcu_flash_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-boot-port-test test/vi/mcu/boot_port_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-mcu-ports-test test/vi/mcu/ports_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-mcu-timer-test test/vi/mcu/timer_test.cpp blofeld_vi_execution)
  blofeld_unit(vi-mcu-clock-test test/vi/mcu/clock_test.cpp blofeld_vi_execution)
  target_include_directories(vi-vm-context-test PRIVATE
    "${BLOFELD_CPU12_ROOT}/test/support" "${BLOFELD_DSP56300_ROOT}/test")
  blofeld_unit(app-profile-test test/app/profile_test.cpp blofeld_app_profile)
  add_test(NAME app-ui-frame-test COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/test/app/ui/frame_test.py" "${CMAKE_CURRENT_BINARY_DIR}/Testing")
  set_tests_properties(app-ui-frame-test PROPERTIES TIMEOUT 10)
  add_test(NAME app-ui-display-test COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/test/app/ui/display_test.py" "${CMAKE_CURRENT_BINARY_DIR}/Testing")
  set_tests_properties(app-ui-display-test PROPERTIES TIMEOUT 10)
  get_property(blofeld_unit_targets GLOBAL PROPERTY BLOFELD_UNIT_TARGETS)
  add_custom_target(quality
    COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
    DEPENDS format-check blofeld-headless ${blofeld_unit_targets}
    USES_TERMINAL)
endif()

install(TARGETS blofeld-headless RUNTIME DESTINATION bin)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS tools/blofeld-display)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/tools/blofeld-display" blofeld_viewer)
string(REGEX REPLACE "^#![^\n]*" "#!${Python3_EXECUTABLE}" blofeld_viewer "${blofeld_viewer}")
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/blofeld-display"
  CONTENT "${blofeld_viewer}" @ONLY)
install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/generated/blofeld-display"
  DESTINATION bin COMPONENT Display)
install(FILES src/app/ui/__init__.py src/app/ui/frame.py src/app/ui/watch.py src/app/ui/display.py
  DESTINATION share/blofeld-emu/ui COMPONENT Display)
install(FILES packaging/org.blofeld_emu.Display.desktop
  DESTINATION share/applications COMPONENT Display)
install(FILES LICENSE THIRD_PARTY.txt DESTINATION share/doc/blofeld-emu)
install(DIRECTORY LICENSES/ DESTINATION share/doc/blofeld-emu/licenses)
