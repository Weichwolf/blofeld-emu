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
find_program(BLOFELD_CLANG_TIDY NAMES clang-tidy-19 REQUIRED)
find_program(BLOFELD_CLANG_FORMAT NAMES clang-format-19 REQUIRED)
include(cmake/Zlib.cmake)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_package(Git REQUIRED)
find_package(Threads REQUIRED)
include(cmake/Dsp56300.cmake)
include(cmake/PulseAudio.cmake)
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

include(cmake/DspComponents.cmake)
include(cmake/ReferenceRomFixtures.cmake)
include(cmake/SerialPcm.cmake)

add_library(blofeld_private_files src/storage/private_file.cpp)
target_include_directories(blofeld_private_files PUBLIC src)
target_link_libraries(blofeld_private_files PUBLIC blofeld_firmware)
add_library(blofeld_sample_library src/samples/sample_library.cpp src/samples/sample_xml_reader.cpp
  src/samples/sample_xml_values.cpp src/samples/sample_xml_codec.cpp src/samples/sample_xml_file.cpp src/samples/sample_client.cpp
  src/samples/sample_wave.cpp src/samples/sample_wave_format.cpp src/samples/sample_wave_metadata.cpp src/samples/sample_pitch.cpp
  src/samples/sample_wave_client.cpp)
target_include_directories(blofeld_sample_library PUBLIC src)
target_link_libraries(blofeld_sample_library PUBLIC blofeld_private_files)
add_library(blofeld_storage src/storage/flash_image.cpp src/storage/flash_file.cpp)
target_include_directories(blofeld_storage PUBLIC src)
target_link_libraries(blofeld_storage PUBLIC blofeld_firmware PRIVATE blofeld_private_files ZLIB::ZLIB)
add_library(blofeld_device_identity src/storage/device_identity.cpp src/storage/virtual_device_state.cpp)
target_include_directories(blofeld_device_identity PUBLIC src)
target_link_libraries(blofeld_device_identity PUBLIC blofeld_storage)
add_library(blofeld_nand src/storage/nand.cpp src/storage/nand_write.cpp src/storage/nand_array.cpp src/storage/nand_state.cpp src/storage/nand_sequential_read.cpp
  src/storage/nand_image.cpp src/storage/nand_image_codec.cpp src/storage/nand_identification.cpp)
target_include_directories(blofeld_nand PUBLIC src)
target_link_libraries(blofeld_nand PUBLIC blofeld_firmware PRIVATE ZLIB::ZLIB)
add_library(blofeld_firmware_nand src/storage/firmware_nand_image.cpp)
target_include_directories(blofeld_firmware_nand PUBLIC src)
target_link_libraries(blofeld_firmware_nand PUBLIC blofeld_firmware blofeld_nand)
add_library(blofeld_nand_operation_clock src/storage/nand_operation_clock.cpp)
target_include_directories(blofeld_nand_operation_clock PUBLIC src)
target_link_libraries(blofeld_nand_operation_clock PUBLIC blofeld_nand)
add_library(blofeld_nand_write_medium src/storage/nand_cell_model.cpp src/storage/nand_write_medium.cpp)
target_include_directories(blofeld_nand_write_medium PUBLIC src)
target_link_libraries(blofeld_nand_write_medium PUBLIC blofeld_nand_operation_clock)

add_library(blofeld_nand_store src/storage/nand_file.cpp src/storage/nand_store.cpp)
target_include_directories(blofeld_nand_store PUBLIC src)
target_link_libraries(blofeld_nand_store PUBLIC blofeld_nand PRIVATE blofeld_private_files)
add_library(blofeld_nand_write_store src/storage/nand_write_store.cpp)
target_include_directories(blofeld_nand_write_store PUBLIC src)
target_link_libraries(blofeld_nand_write_store PUBLIC blofeld_nand_write_medium blofeld_nand_store
  PRIVATE blofeld_private_files)
add_library(blofeld_nand_peer src/runtime/nand_peer.cpp src/runtime/nand_peer_medium.cpp
  src/runtime/nand_bus_decode.cpp)
target_include_directories(blofeld_nand_peer PUBLIC src)
target_link_libraries(blofeld_nand_peer PUBLIC blofeld_dsp_machine blofeld_nand_write_medium blofeld_nand_write_store blofeld_dsp_peripheral_calendar)

add_library(blofeld_mcu src/mcu/mcu_bus.cpp src/mcu/mcu_bus_clone.cpp src/mcu/mcu_bus_usb.cpp src/mcu/mcu_code_cache.cpp src/mcu/mcu_memory.cpp src/mcu/mcu_gpio.cpp
  src/mcu/mcu_flash_config.cpp src/mcu/mcu_flash.cpp src/mcu/mcu_flash_clock.cpp
  src/mcu/mcu_flash_timing.cpp src/mcu/mcu_flash_persistence.cpp
  src/mcu/mcu_flash_backdoor.cpp src/mcu/mcu_flash_memory.cpp src/mcu/mcu_bus_flash.cpp
  src/mcu/mcu_clock.cpp src/mcu/mcu_cop.cpp src/mcu/mcu_reset.cpp src/mcu/mcu_run_clock.cpp src/mcu/mcu_sci.cpp src/mcu/mcu_timer.cpp src/mcu/mcu_interrupts.cpp
  src/mcu/mcu_usb.cpp src/mcu/mcu_usb_io.cpp src/mcu/mcu_usb_status.cpp src/mcu/mcu_usb_packets.cpp
  src/mcu/mcu_usb_control.cpp src/mcu/mcu_usb_standard_status.cpp
  src/mcu/mcu_usb_ique_control.cpp src/mcu/mcu_usb_reset.cpp src/mcu/mcu_usb_host.cpp)
target_include_directories(blofeld_mcu PUBLIC src)
target_link_libraries(blofeld_mcu PUBLIC blofeld_firmware blofeld_storage blofeld_gpio_pads
  PRIVATE blofeld_device_identity)
add_library(blofeld_usb_host src/usb/usb_transfer.cpp src/usb/usb_host.cpp src/usb/usb_host_packets.cpp
  src/usb/usb_device_probe.cpp src/usb/usb_descriptors.cpp src/usb/usb_host_link.cpp src/usb/usb_midi_host.cpp)
target_include_directories(blofeld_usb_host PUBLIC src)
target_link_libraries(blofeld_usb_host PUBLIC blofeld_mcu)
add_library(blofeld_usbip src/usb/usbip_decode.cpp src/usb/usbip_encode.cpp
  src/usb/usbip_management.cpp src/usb/usbip_export.cpp src/usb/usbip_session.cpp src/usb/usbip_session_output.cpp)
target_include_directories(blofeld_usbip PUBLIC src)
target_link_libraries(blofeld_usbip PUBLIC blofeld_usb_host)
add_library(blofeld_usbip_tcp src/usb/usbip_tcp.cpp src/usb/usbip_tcp_io.cpp src/usb/usbip_runtime.cpp)
target_include_directories(blofeld_usbip_tcp PUBLIC src)
target_link_libraries(blofeld_usbip_tcp PUBLIC blofeld_usbip)
add_library(blofeld_display src/display/st7565.cpp src/display/st7565_parallel.cpp)
target_include_directories(blofeld_display PUBLIC src)
target_link_libraries(blofeld_display PUBLIC blofeld_firmware)
add_library(blofeld_lcd_export src/display/lcd_image.cpp src/display/lcd_png.cpp src/display/lcd_export.cpp)
target_include_directories(blofeld_lcd_export PUBLIC src)
target_link_libraries(blofeld_lcd_export PUBLIC blofeld_display blofeld_private_files PRIVATE ZLIB::ZLIB)
include(cmake/Cpu12.cmake)
add_library(blofeld_core src/mcu/hc12_native.cpp src/mcu/mcu_machine.cpp)

add_library(blofeld_pcm_transport src/audio/pcm_recording.cpp src/audio/pcm_wave.cpp src/audio/pcm_wave_file.cpp)
target_include_directories(blofeld_pcm_transport PUBLIC src)
target_link_libraries(blofeld_pcm_transport PUBLIC blofeld_serial_pcm PRIVATE blofeld_private_files)
add_library(blofeld_host_audio src/audio/host_audio.cpp src/audio/host_audio_pcm.cpp
  src/audio/host_audio_stream.cpp)
target_include_directories(blofeld_host_audio PUBLIC src)
target_link_libraries(blofeld_host_audio PUBLIC blofeld_serial_pcm PRIVATE blofeld_pulse)
add_library(blofeld_pcm_queue src/audio/pcm_queue.cpp)
target_include_directories(blofeld_pcm_queue PUBLIC src)
target_link_libraries(blofeld_pcm_queue PUBLIC blofeld_serial_pcm)
add_library(blofeld_audio_runtime src/audio/audio_runtime.cpp src/audio/audio_worker.cpp
  src/audio/audio_wake.cpp)
target_include_directories(blofeld_audio_runtime PUBLIC src)
target_link_libraries(blofeld_audio_runtime PUBLIC blofeld_host_audio blofeld_pcm_queue PRIVATE Threads::Threads)

target_include_directories(blofeld_core PUBLIC src)
target_link_libraries(blofeld_core PUBLIC blofeld_mcu)
set(blofeld_independent_native_args --native dsp56300_core=$<TARGET_FILE:dsp56300_core>)
target_link_libraries(blofeld_core PUBLIC cpu12::core)
list(APPEND blofeld_independent_native_args --native cpu12_core=$<TARGET_FILE:cpu12_core>)
target_compile_definitions(blofeld_core PUBLIC BLOFELD_DEFAULT_SLA="")

add_library(blofeld_vi
  src/runtime/mcu_dsp_gpio_peer.cpp
  src/vi/vi.cpp src/vi/vi_runtime.cpp src/vi/vi_execution.cpp
  src/vi/vi_io.cpp src/vi/vi_display.cpp src/vi/vi_profile.cpp
  src/vi/vi_time.cpp src/vi/vi_clock.cpp src/vi/vi_audio.cpp)
target_include_directories(blofeld_vi PUBLIC src)
target_link_libraries(blofeld_vi PUBLIC blofeld_mcu_dsp_runtime
  blofeld_private_files blofeld_pcm_transport blofeld_usbip_tcp blofeld_nand_peer
  blofeld_nand_store blofeld_lcd_export)
add_executable(blofeld-headless src/app/main.cpp src/vi/host/vi_display.cpp)
target_link_libraries(blofeld-headless PRIVATE blofeld_vi
  blofeld_audio_runtime blofeld_device_identity)

if(BUILD_TESTING)
  add_test(NAME instrument_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/instrument/cli_test.py" $<TARGET_FILE:blofeld-headless>)
  add_executable(blofeld-firmware-parser-test test/firmware/inspect.cpp)
  target_link_libraries(blofeld-firmware-parser-test PRIVATE blofeld_firmware)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-nand-io-test test/runtime/nand_io_test.cpp)
  target_link_libraries(blofeld-nand-io-test PRIVATE blofeld_nand_peer)
  add_test(NAME nand_io COMMAND blofeld-nand-io-test)
endif()

if(BUILD_TESTING)

  add_executable(blofeld-dsp-adapter-io-test test/dsp_adapter_io_test.cpp)
  target_link_libraries(blofeld-dsp-adapter-io-test PRIVATE blofeld_dsp_machine)
  add_test(NAME dsp_adapter_io COMMAND blofeld-dsp-adapter-io-test)
  add_executable(blofeld-dsp-adapter-interrupts-test test/dsp_adapter_interrupts_test.cpp)
  target_link_libraries(blofeld-dsp-adapter-interrupts-test PRIVATE blofeld_dsp_machine)
  target_include_directories(blofeld-dsp-adapter-interrupts-test PRIVATE "${BLOFELD_DSP56300_ROOT}/test/support")
  add_test(NAME dsp_interrupt_admission COMMAND blofeld-dsp-adapter-interrupts-test)
  add_executable(blofeld-dsp-adapter-reset-test test/dsp_adapter_reset_test.cpp)
  target_link_libraries(blofeld-dsp-adapter-reset-test PRIVATE blofeld_dsp_machine)
  add_test(NAME dsp_adapter_reset COMMAND blofeld-dsp-adapter-reset-test)

  add_executable(blofeld-original-device-identity-test test/original_device_identity_test.cpp)
  target_link_libraries(blofeld-original-device-identity-test PRIVATE blofeld_device_identity blofeld_core)

  add_test(NAME original_device_identity COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_device_identity.py"
    $<TARGET_FILE:blofeld-original-device-identity-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
  set_tests_properties(original_device_identity PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-original-startup-identity-test test/original_startup_identity_test.cpp)
  target_link_libraries(blofeld-original-startup-identity-test PRIVATE
    blofeld_device_identity blofeld_parallel_fixture blofeld_lcd_fixture blofeld_lcd_export)
  add_test(NAME original_startup_identity COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_startup_identity.py"
    $<TARGET_FILE:blofeld-original-startup-identity-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-startup-identity.trace")
  set_tests_properties(original_startup_identity PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-virtual-device-state-test test/virtual_device_state_test.cpp)
  target_link_libraries(blofeld-virtual-device-state-test PRIVATE blofeld_device_identity)
  add_test(NAME virtual_device_state COMMAND blofeld-virtual-device-state-test)
  add_executable(blofeld-device-identity-test test/device_identity_test.cpp)
  target_link_libraries(blofeld-device-identity-test PRIVATE blofeld_device_identity)
  add_test(NAME device_identity COMMAND blofeld-device-identity-test)

  set(blofeld_audio_test_timeout 120)
  if(BLOFELD_SANITIZER STREQUAL "thread")
    set(blofeld_audio_test_timeout 300)
  endif()

  add_executable(blofeld-pcm-queue-test test/pcm_queue_test.cpp)
  target_link_libraries(blofeld-pcm-queue-test PRIVATE blofeld_pcm_queue Threads::Threads blofeld_pcm_reference)
  add_test(NAME pcm_queue COMMAND blofeld-pcm-queue-test)
  set_tests_properties(pcm_queue PROPERTIES TIMEOUT 40)

  add_executable(blofeld-hc12-ccr-test test/hc12_ccr_test.cpp)
  target_link_libraries(blofeld-hc12-ccr-test PRIVATE blofeld_core)
  add_test(NAME hc12_ccr COMMAND blofeld-hc12-ccr-test)
  add_executable(blofeld-original-usb-midi-test test/original_usb_midi_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-usb-midi-test PRIVATE blofeld_core)
  add_test(NAME original_usb_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_usb_midi.py"
    $<TARGET_FILE:blofeld-original-usb-midi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-usb-midi.log")
  set_tests_properties(original_usb_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
  add_executable(blofeld-original-usb-queue-test test/original_usb_queue_test.cpp
    test/original_usb_queue_fixture.cpp test/original_usb_queue_interrupts.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-usb-queue-test PRIVATE blofeld_core)
  add_test(NAME original_usb_queue COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_usb_queue.py"
    $<TARGET_FILE:blofeld-original-usb-queue-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-usb-queue.log")
  set_tests_properties(original_usb_queue PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-original-midi-forward-test test/original_midi_forward_test.cpp
    test/original_midi_forward_fixture.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-midi-forward-test PRIVATE blofeld_core)
  add_test(NAME original_midi_forward COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_midi_forward.py"
    $<TARGET_FILE:blofeld-original-midi-forward-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-midi-forward.log")
  set_tests_properties(original_midi_forward PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-original-midi-dispatch-test test/original_midi_dispatch_test.cpp
    test/original_midi_dispatch_fixture.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-midi-dispatch-test PRIVATE blofeld_core)
  add_test(NAME original_midi_dispatch COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_midi_dispatch.py"
    $<TARGET_FILE:blofeld-original-midi-dispatch-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-midi-dispatch.log")
  set_tests_properties(original_midi_dispatch PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-original-usb-peer-test test/original_usb_peer_test.cpp
    test/original_usb_peer_fixture.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-usb-peer-test PRIVATE blofeld_core)
  add_test(NAME original_usb_peer COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_usb_peer.py"
    $<TARGET_FILE:blofeld-original-usb-peer-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-usb-peer.log")
  set_tests_properties(original_usb_peer PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-usbip-output-timeout-test test/usbip_output_timeout_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_include_directories(blofeld-usbip-output-timeout-test PRIVATE src)
  target_link_libraries(blofeld-usbip-output-timeout-test PRIVATE blofeld_usbip_tcp blofeld_core)
  add_test(NAME usbip_output_timeout COMMAND blofeld-usbip-output-timeout-test)
  set_tests_properties(usbip_output_timeout PROPERTIES TIMEOUT 30)
  add_executable(blofeld-usbip-frame-timeout-test test/usbip_frame_timeout_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_include_directories(blofeld-usbip-frame-timeout-test PRIVATE src)
  target_link_libraries(blofeld-usbip-frame-timeout-test PRIVATE blofeld_usbip_tcp blofeld_core)
  add_test(NAME usbip_frame_timeout COMMAND blofeld-usbip-frame-timeout-test)
  add_executable(blofeld-usbip-suspend-test test/usbip_suspend_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_include_directories(blofeld-usbip-suspend-test PRIVATE src)
  target_link_libraries(blofeld-usbip-suspend-test PRIVATE blofeld_usbip_tcp blofeld_core)
  add_test(NAME usbip_suspend COMMAND blofeld-usbip-suspend-test)
  add_executable(blofeld-usbip-runtime-test test/usbip_runtime_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_include_directories(blofeld-usbip-runtime-test PRIVATE src)
  target_link_libraries(blofeld-usbip-runtime-test PRIVATE blofeld_usbip_tcp blofeld_core)
  add_test(NAME usbip_runtime COMMAND blofeld-usbip-runtime-test)

  add_executable(blofeld-usbip-session-boundaries-test test/usbip_session_boundaries_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usbip-session-boundaries-test PRIVATE blofeld_usbip)
  add_test(NAME usbip_session_boundaries COMMAND blofeld-usbip-session-boundaries-test)
  add_executable(blofeld-usbip-tcp-native-test test/usbip_tcp_native_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_include_directories(blofeld-usbip-tcp-native-test PRIVATE src)
  target_link_libraries(blofeld-usbip-tcp-native-test PRIVATE blofeld_usbip_tcp blofeld_core)
  add_test(NAME usbip_tcp_native COMMAND blofeld-usbip-tcp-native-test)
  add_executable(blofeld-usbip-session-test test/usbip_session_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usbip-session-test PRIVATE blofeld_usbip)
  add_test(NAME usbip_session COMMAND blofeld-usbip-session-test)
  add_executable(blofeld-usbip-tcp-test test/usbip_tcp_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_include_directories(blofeld-usbip-tcp-test PRIVATE src)
  target_link_libraries(blofeld-usbip-tcp-test PRIVATE blofeld_usbip_tcp)
  add_test(NAME usbip_tcp COMMAND blofeld-usbip-tcp-test)
  add_executable(blofeld-usb-device-native-test test/usb_device_native_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usb-device-native-test PRIVATE blofeld_usbip blofeld_core)
  add_test(NAME usb_device_native COMMAND blofeld-usb-device-native-test)
  add_executable(blofeld-usb-host-link-test test/usb_host_link_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usb-host-link-test PRIVATE blofeld_usb_host)
  add_test(NAME usb_host_link COMMAND blofeld-usb-host-link-test)
  add_executable(blofeld-usb-midi-host-test test/usb_midi_host_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_include_directories(blofeld-usb-midi-host-test PRIVATE "${BLOFELD_CPU12_ROOT}/test")
  target_link_libraries(blofeld-usb-midi-host-test PRIVATE blofeld_usb_host blofeld_core)
  add_test(NAME usb_midi_host COMMAND blofeld-usb-midi-host-test)
  add_executable(blofeld-usb-device-test test/usb_device_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usb-device-test PRIVATE blofeld_usb_host)
  add_test(NAME usb_device_probe COMMAND blofeld-usb-device-test)
  add_executable(blofeld-usbip-management-test test/usbip_management_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usbip-management-test PRIVATE blofeld_usbip)
  add_test(NAME usbip_management COMMAND blofeld-usbip-management-test)
  add_executable(blofeld-usbip-test test/usbip_test.cpp)
  target_link_libraries(blofeld-usbip-test PRIVATE blofeld_usbip)
  add_test(NAME usbip_protocol COMMAND blofeld-usbip-test)
  add_executable(blofeld-usb-host-test test/usb_host_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usb-host-test PRIVATE blofeld_usb_host)
  add_test(NAME usb_host_transfers COMMAND blofeld-usb-host-test)
  add_executable(blofeld-usb-host-native-test test/usb_host_native_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usb-host-native-test PRIVATE blofeld_usbip blofeld_core)
  add_test(NAME usb_host_native COMMAND blofeld-usb-host-native-test)
  add_executable(blofeld-sample-client-test test/samples_client.cpp)
  target_link_libraries(blofeld-sample-client-test PRIVATE blofeld_sample_library)
  add_executable(blofeld-sample-wave-test test/sample_wave_test.cpp)
  target_link_libraries(blofeld-sample-wave-test PRIVATE blofeld_sample_library)
  add_test(NAME sample_wave COMMAND blofeld-sample-wave-test)
  add_test(NAME sample_wave_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_wave_cli.py" $<TARGET_FILE:blofeld-sample-client-test>)
  add_executable(blofeld-sample-library-test test/sample_library_test.cpp)
  target_link_libraries(blofeld-sample-library-test PRIVATE blofeld_sample_library)
  add_test(NAME sample_library COMMAND blofeld-sample-library-test)
  add_test(NAME sample_library_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_library_cli.py" $<TARGET_FILE:blofeld-sample-client-test>)

  add_executable(blofeld-pcm-transport-test test/pcm_transport_test.cpp test/pcm_wave_test.cpp)
  target_link_libraries(blofeld-pcm-transport-test PRIVATE blofeld_pcm_transport blofeld_private_files blofeld_pcm_reference)
  add_test(NAME pcm_transport COMMAND blofeld-pcm-transport-test
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/pcm-transport")
  add_executable(blofeld-esai-initialization-test test/dsp_esai_initialization_test.cpp)
  target_link_libraries(blofeld-esai-initialization-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_esai_initialization COMMAND blofeld-esai-initialization-test)
  add_executable(blofeld-esai-cold-clock-test test/dsp_esai_cold_clock_test.cpp)
  target_link_libraries(blofeld-esai-cold-clock-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_esai_cold_clock COMMAND blofeld-esai-cold-clock-test)
  add_executable(blofeld-dsp-io-map-test test/dsp_io_map_test.cpp)
  target_link_libraries(blofeld-dsp-io-map-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_io_map COMMAND blofeld-dsp-io-map-test)

  add_executable(blofeld-startup-probe EXCLUDE_FROM_ALL test/instrument/startup_probe.cpp)
  target_link_libraries(blofeld-startup-probe PRIVATE blofeld_vi blofeld_device_identity)

  add_executable(blofeld-mcu-dsp-execution-test test/runtime/mcu_dsp_execution_test.cpp)
  target_include_directories(blofeld-mcu-dsp-execution-test PRIVATE test
    "${BLOFELD_CPU12_ROOT}/test/support" "${BLOFELD_DSP56300_ROOT}/test")
  target_link_libraries(blofeld-mcu-dsp-execution-test PRIVATE blofeld_mcu_dsp_runtime)
  add_test(NAME mcu_dsp_execution COMMAND blofeld-mcu-dsp-execution-test)

  add_executable(blofeld-mcu-native-execution-test test/mcu_native_execution_test.cpp)
  target_include_directories(blofeld-mcu-native-execution-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  target_link_libraries(blofeld-mcu-native-execution-test PRIVATE blofeld_core)
  add_test(NAME mcu_native_execution COMMAND blofeld-mcu-native-execution-test)

  add_executable(blofeld-mcu-ram-program-test test/mcu_ram_program_test.cpp)
  target_include_directories(blofeld-mcu-ram-program-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  target_link_libraries(blofeld-mcu-ram-program-test PRIVATE blofeld_core)
  add_test(NAME mcu_ram_program COMMAND blofeld-mcu-ram-program-test)
  set_tests_properties(mcu_ram_program PROPERTIES TIMEOUT 10)

  add_executable(blofeld-mcu-io-continuation-test test/mcu_io_continuation_test.cpp)
  target_include_directories(blofeld-mcu-io-continuation-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  target_link_libraries(blofeld-mcu-io-continuation-test PRIVATE blofeld_core)
  add_test(NAME mcu_io_continuation COMMAND blofeld-mcu-io-continuation-test)
  set_tests_properties(mcu_io_continuation PROPERTIES TIMEOUT 10)

  add_executable(blofeld-mcu-power-on-test test/mcu_power_on_test.cpp)
  target_link_libraries(blofeld-mcu-power-on-test PRIVATE blofeld_core)
  add_test(NAME mcu_power_on COMMAND blofeld-mcu-power-on-test)

  add_executable(blofeld-mcu-wait-clock-test test/mcu_wait_clock_test.cpp)
  target_include_directories(blofeld-mcu-wait-clock-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  target_link_libraries(blofeld-mcu-wait-clock-test PRIVATE blofeld_core)
  add_test(NAME mcu_wait_clock COMMAND blofeld-mcu-wait-clock-test)

  set(BLOFELD_CLOCKED_STARTUP_REFERENCE
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    CACHE FILEPATH "Pinned original ZIP or identical complete source MID for clocked startup")

  find_program(BLOFELD_LLVM_NM NAMES llvm-nm-19 REQUIRED)
  find_program(BLOFELD_LLVM_READELF NAMES llvm-readelf-19 REQUIRED)
  find_program(BLOFELD_LLVM_DIS NAMES llvm-dis-19 REQUIRED)

  add_executable(blofeld-firmware-nand-test test/firmware_nand_image_test.cpp)
  target_link_libraries(blofeld-firmware-nand-test PRIVATE blofeld_firmware_nand blofeld_nand_store)
  add_test(NAME firmware_nand_image COMMAND blofeld-firmware-nand-test)
  set_tests_properties(firmware_nand_image PROPERTIES TIMEOUT 120)
  add_test(NAME firmware_nand_reference COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/firmware_nand_reference.py"
    $<TARGET_FILE:blofeld-firmware-nand-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/firmware-nand-reference.bfnand")
  set_tests_properties(firmware_nand_reference PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)

  add_executable(blofeld-nand-read-test test/nand_read_test.cpp)
  target_link_libraries(blofeld-nand-read-test PRIVATE blofeld_nand)
  add_test(NAME nand_read COMMAND blofeld-nand-read-test)
  add_executable(blofeld-nand-identification-test test/nand_identification_test.cpp)
  target_link_libraries(blofeld-nand-identification-test PRIVATE blofeld_nand_store blofeld_nand_operation_clock)
  add_test(NAME nand_identification COMMAND blofeld-nand-identification-test)
  add_executable(blofeld-original-nand-identification-test test/original_nand_identification_test.cpp
    test/dsp_core_fixture.cpp)
  target_link_libraries(blofeld-original-nand-identification-test PRIVATE blofeld_dma_fixture blofeld_nand)
  add_test(NAME original_nand_identification COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_nand_identification.py"
    $<TARGET_FILE:blofeld-original-nand-identification-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-identification.trace")
  set_tests_properties(original_nand_identification PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-nand-operation-clock-test test/nand_operation_clock_test.cpp)
  target_link_libraries(blofeld-nand-operation-clock-test PRIVATE blofeld_nand_operation_clock)
  add_test(NAME nand_operation_clock COMMAND blofeld-nand-operation-clock-test)

  add_executable(blofeld-nand-image-test test/nand_image_test.cpp)
  target_link_libraries(blofeld-nand-image-test PRIVATE blofeld_nand)
  add_test(NAME nand_image COMMAND blofeld-nand-image-test)
  add_executable(blofeld-nand-page-cache-test test/nand_page_cache_test.cpp)
  target_link_libraries(blofeld-nand-page-cache-test PRIVATE blofeld_nand)
  add_test(NAME nand_page_cache COMMAND blofeld-nand-page-cache-test)
  add_executable(blofeld-nand-store-test test/nand_store_test.cpp)
  target_link_libraries(blofeld-nand-store-test PRIVATE blofeld_nand_store)
  add_test(NAME nand_store COMMAND blofeld-nand-store-test)
  add_executable(blofeld-nand-write-test test/nand_write_test.cpp)
  target_link_libraries(blofeld-nand-write-test PRIVATE blofeld_nand)
  add_test(NAME nand_write COMMAND blofeld-nand-write-test)
  add_executable(blofeld-nand-power-test test/nand_power_test.cpp)
  target_link_libraries(blofeld-nand-power-test PRIVATE blofeld_nand)
  add_test(NAME nand_power COMMAND blofeld-nand-power-test)
  add_executable(blofeld-nand-gpio-test test/nand_gpio_test.cpp test/nand_gpio_fixture.cpp
    test/dsp_core_fixture.cpp)
  target_link_libraries(blofeld-nand-gpio-test PRIVATE blofeld_nand_store blofeld_dma_fixture)
  add_test(NAME nand_gpio COMMAND blofeld-nand-gpio-test)
  add_executable(blofeld-nand-gpio-write-test test/nand_gpio_write_test.cpp test/nand_gpio_fixture.cpp
    test/dsp_core_fixture.cpp)
  target_link_libraries(blofeld-nand-gpio-write-test PRIVATE blofeld_nand_store blofeld_dma_fixture)
  add_test(NAME nand_gpio_write COMMAND blofeld-nand-gpio-write-test
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/nand-gpio-write.trace")
  add_test(NAME original_nand_address COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-nand-gpio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-address.trace")
  set_tests_properties(original_nand_address PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-nand-write-test test/original_nand_write_test.cpp test/nand_gpio_fixture.cpp
    test/dsp_core_fixture.cpp)
  target_link_libraries(blofeld-original-nand-write-test PRIVATE blofeld_nand_store blofeld_dma_fixture)
  add_test(NAME original_nand_write COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-nand-write-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-write.trace")
  set_tests_properties(original_nand_write PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-update-validation-test test/original_update_validation_test.cpp
    test/dsp_core_fixture.cpp)
  target_link_libraries(blofeld-original-update-validation-test PRIVATE blofeld_dma_fixture)
  add_test(NAME original_update_validation COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_update_validation.py"
    $<TARGET_FILE:blofeld-original-update-validation-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-update-validation.trace")
  set_tests_properties(original_update_validation PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)

  add_test(NAME original_secondary_update_validation COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_secondary_update_validation.py"
    $<TARGET_FILE:blofeld-original-update-validation-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-secondary-update-validation.trace")
  set_tests_properties(original_secondary_update_validation PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_test(NAME nand_store_processes COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/nand_store_processes.py"
    $<TARGET_FILE:blofeld-original-nand-write-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/nand-store-processes.json")
  set_tests_properties(nand_store_processes PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-flash-image-test test/flash_image_test.cpp)
  target_link_libraries(blofeld-flash-image-test PRIVATE blofeld_core)
  add_test(NAME flash_image COMMAND blofeld-flash-image-test)
  add_executable(blofeld-flash-file-worker test/flash_file_worker.cpp)
  target_link_libraries(blofeld-flash-file-worker PRIVATE blofeld_storage)
  add_test(NAME flash_file_processes COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/flash_file_processes.py"
    $<TARGET_FILE:blofeld-flash-file-worker>)

  add_executable(blofeld-flash-backdoor-test test/flash_backdoor_test.cpp test/flash_backdoor_native.cpp)
  target_link_libraries(blofeld-flash-backdoor-test PRIVATE blofeld_core)
  add_test(NAME flash_backdoor COMMAND blofeld-flash-backdoor-test)
  add_executable(blofeld-flash-timing-test test/mcu_flash_timing_test.cpp)
  target_link_libraries(blofeld-flash-timing-test PRIVATE blofeld_core)
  add_test(NAME flash_timing COMMAND blofeld-flash-timing-test)
  add_executable(blofeld-flash-clock-test test/mcu_flash_clock_test.cpp)
  target_link_libraries(blofeld-flash-clock-test PRIVATE blofeld_core)
  add_test(NAME flash_native_clock COMMAND blofeld-flash-clock-test)
  add_executable(blofeld-flash-calendar-test test/mcu_flash_calendar_test.cpp)
  target_link_libraries(blofeld-flash-calendar-test PRIVATE blofeld_core)
  add_test(NAME flash_calendar COMMAND blofeld-flash-calendar-test)
  add_executable(blofeld-flash-live-test test/mcu_flash_live_test.cpp)
  target_link_libraries(blofeld-flash-live-test PRIVATE blofeld_core)
  add_executable(blofeld-flash-live-worker test/mcu_flash_live_worker.cpp)
  target_link_libraries(blofeld-flash-live-worker PRIVATE blofeld_core)
  add_executable(blofeld-flash-durable-queue-test test/mcu_flash_durable_queue_test.cpp)
  target_link_libraries(blofeld-flash-durable-queue-test PRIVATE blofeld_core)
  add_test(NAME flash_native_persistence COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/mcu_flash_live_processes.py"
    $<TARGET_FILE:blofeld-flash-live-test> $<TARGET_FILE:blofeld-flash-durable-queue-test>
    $<TARGET_FILE:blofeld-flash-live-worker>)
  set_tests_properties(flash_native_persistence PROPERTIES TIMEOUT 120)

  add_executable(blofeld-flash-test test/mcu_flash_test.cpp)
  target_link_libraries(blofeld-flash-test PRIVATE blofeld_core)
  add_test(NAME flash_commands COMMAND blofeld-flash-test)
  add_executable(blofeld-flash-stop-test test/mcu_flash_stop_test.cpp)
  target_link_libraries(blofeld-flash-stop-test PRIVATE blofeld_core)
  add_test(NAME flash_stop COMMAND blofeld-flash-stop-test)
  add_executable(blofeld-original-flash-stop-test test/original_flash_stop_test.cpp)
  target_link_libraries(blofeld-original-flash-stop-test PRIVATE blofeld_core)
  add_test(NAME original_flash_stop COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-flash-stop-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-flash-stop.trace")
  set_tests_properties(original_flash_stop PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-flash-test test/original_flash_commands_test.cpp)
  target_link_libraries(blofeld-original-flash-test PRIVATE blofeld_core)
  add_test(NAME original_flash_commands COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-flash-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-flash-commands.trace")
  set_tests_properties(original_flash_commands PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-flash-restart-test test/original_flash_restart_test.cpp)
  target_link_libraries(blofeld-original-flash-restart-test PRIVATE blofeld_core)
  add_test(NAME original_flash_restarts COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_flash_restarts.py"
    $<TARGET_FILE:blofeld-original-flash-test>
    $<TARGET_FILE:blofeld-original-flash-restart-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing")
  set_tests_properties(original_flash_restarts PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME original_recovery_wire COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_flash_restarts.py"
    $<TARGET_FILE:blofeld-original-flash-test>
    $<TARGET_FILE:blofeld-original-flash-restart-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing" --wire-only)
  set_tests_properties(original_recovery_wire PROPERTIES SKIP_RETURN_CODE 77)
  add_library(blofeld_parallel_fixture STATIC test/parallel_update_fixture.cpp)
  target_link_libraries(blofeld_parallel_fixture PUBLIC blofeld_core)
  add_executable(blofeld-original-parallel-update-test test/original_parallel_update_test.cpp)
  target_link_libraries(blofeld-original-parallel-update-test PRIVATE blofeld_parallel_fixture)
  add_test(NAME original_parallel_update COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-parallel-update-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-parallel-update.trace")
  set_tests_properties(original_parallel_update PROPERTIES SKIP_RETURN_CODE 77)
  add_library(blofeld_lcd_fixture STATIC test/lcd_fixture.cpp)
  target_link_libraries(blofeld_lcd_fixture PUBLIC blofeld_core blofeld_display)

  add_executable(blofeld-original-lcd-helper-test test/original_lcd_helper_test.cpp)
  target_link_libraries(blofeld-original-lcd-helper-test PRIVATE blofeld_parallel_fixture blofeld_lcd_fixture)
  add_test(NAME original_lcd_helper COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-lcd-helper-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-lcd-helper.trace")
  set_tests_properties(original_lcd_helper PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-lcd-image-test test/original_lcd_image_test.cpp)
  target_link_libraries(blofeld-original-lcd-image-test PRIVATE blofeld_parallel_fixture
    blofeld_lcd_fixture blofeld_lcd_export)
  add_test(NAME original_lcd_image COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-lcd-image-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-lcd-image.trace")
  set_tests_properties(original_lcd_image PROPERTIES SKIP_RETURN_CODE 77)

  add_executable(blofeld-usb-ique-control-test test/usb_ique_control_test.cpp)
  target_link_libraries(blofeld-usb-ique-control-test PRIVATE blofeld_mcu)
  add_test(NAME usb_ique_control COMMAND blofeld-usb-ique-control-test)
  add_executable(blofeld-usb-endpoint-reset-test test/usb_endpoint_reset_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usb-endpoint-reset-test PRIVATE blofeld_core)
  add_test(NAME usb_endpoint_reset COMMAND blofeld-usb-endpoint-reset-test)

  add_executable(blofeld-usb-bulk-fields-test test/usb_bulk_fields_test.cpp)
  target_link_libraries(blofeld-usb-bulk-fields-test PRIVATE blofeld_mcu)
  add_test(NAME usb_bulk_fields COMMAND blofeld-usb-bulk-fields-test)

  foreach(identity_mode baseline factory)

  endforeach()
  add_executable(blofeld-flash-config-test test/mcu_flash_config_test.cpp)
  target_link_libraries(blofeld-flash-config-test PRIVATE blofeld_mcu)
  add_test(NAME flash_config COMMAND blofeld-flash-config-test)
  add_executable(blofeld-original-flash-config-test test/original_flash_config_test.cpp)
  target_link_libraries(blofeld-original-flash-config-test PRIVATE blofeld_core)
  add_test(NAME original_flash_config COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-flash-config-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-flash-config.trace")
  set_tests_properties(original_flash_config PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-dsp-timers-test test/dsp_timers_test.cpp)
  target_link_libraries(blofeld-dsp-timers-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_timers COMMAND blofeld-dsp-timers-test)
  add_executable(blofeld-dsp-timer-updates-test test/dsp_timer_updates_test.cpp)
  target_link_libraries(blofeld-dsp-timer-updates-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_timer_updates COMMAND blofeld-dsp-timer-updates-test)

  add_executable(blofeld-dsp-gpio-test test/dsp_gpio_test.cpp)
  target_link_libraries(blofeld-dsp-gpio-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_gpio COMMAND blofeld-dsp-gpio-test)
  add_executable(blofeld-dsp-pll-test test/dsp_pll_test.cpp)
  target_link_libraries(blofeld-dsp-pll-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_pll COMMAND blofeld-dsp-pll-test)

  add_executable(blofeld-original-mcu-request-release-test test/original_mcu_request_release.cpp)
  target_link_libraries(blofeld-original-mcu-request-release-test PRIVATE blofeld_core blofeld_firmware)
  add_test(NAME original_mcu_request_release COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_mcu_request_release.py"
    $<TARGET_FILE:blofeld-original-mcu-request-release-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-mcu-request-release.trace")
  set_tests_properties(original_mcu_request_release PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)

  add_test(NAME sample_midi_pages COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_midi_pages_test.py")
  add_test(NAME sample_midi_reference COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_midi_pages_test.py" --reference
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/sample-midi-reference.json")
  set_tests_properties(sample_midi_reference PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)

  add_test(NAME wpc_codec COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/wpc_reference.py" --synthetic)
  add_test(NAME wpc_reference COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/wpc_reference.py"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/wpc-reference.json")
  set_tests_properties(wpc_reference PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_test(NAME wpc_catalogue COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/wpc_catalogue_test.py" --synthetic)
  add_test(NAME wpc_catalogue_reference COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/wpc_catalogue_test.py"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/wpc-catalogue.json")
  set_tests_properties(wpc_catalogue_reference PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)

  add_executable(blofeld-shi-test test/dsp_shi_test.cpp)
  target_link_libraries(blofeld-shi-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_shi COMMAND blofeld-shi-test)
  add_executable(blofeld-mcu-shi-test test/mcu_shi_test.cpp)
  target_link_libraries(blofeld-mcu-shi-test PRIVATE blofeld_core blofeld_dsp_peripherals)
  add_test(NAME mcu_shi COMMAND blofeld-mcu-shi-test)
  add_executable(blofeld-core-test test/core_test.cpp)
  target_link_libraries(blofeld-core-test PRIVATE blofeld_core)
  add_test(NAME native_core COMMAND blofeld-core-test)
  add_executable(blofeld-code-cache-test test/code_cache_test.cpp)
  target_link_libraries(blofeld-code-cache-test PRIVATE blofeld_core)
  add_test(NAME code_cache COMMAND blofeld-code-cache-test)
  add_executable(blofeld-mcu-test test/mcu_model_test.cpp)
  target_link_libraries(blofeld-mcu-test PRIVATE blofeld_core)
  add_test(NAME mcu_model COMMAND blofeld-mcu-test)
  add_executable(blofeld-gpio-output-test test/mcu_gpio_output_test.cpp)
  target_link_libraries(blofeld-gpio-output-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_gpio_outputs COMMAND blofeld-gpio-output-test)
  add_executable(blofeld-gpio-unknown-driver-test test/gpio_unknown_driver_test.cpp)
  target_link_libraries(blofeld-gpio-unknown-driver-test PRIVATE blofeld_mcu blofeld_dsp_peripherals)
  add_test(NAME gpio_unknown_drivers COMMAND blofeld-gpio-unknown-driver-test)
  add_executable(blofeld-gpio-pad-source-test test/gpio_pad_source_test.cpp)
  target_link_libraries(blofeld-gpio-pad-source-test PRIVATE blofeld_mcu blofeld_dsp_machine)
  add_test(NAME gpio_pad_sources COMMAND blofeld-gpio-pad-source-test)

  add_executable(blofeld-mcu-cop-test test/mcu_cop_test.cpp)
  target_link_libraries(blofeld-mcu-cop-test PRIVATE blofeld_core)
  add_test(NAME mcu_cop COMMAND blofeld-mcu-cop-test)
  add_executable(blofeld-clock-test test/mcu_clock_test.cpp)
  target_link_libraries(blofeld-clock-test PRIVATE blofeld_core)
  add_test(NAME mcu_clock COMMAND blofeld-clock-test
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip")
  add_executable(blofeld-sci-test test/mcu_sci_test.cpp)
  target_link_libraries(blofeld-sci-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_sci COMMAND blofeld-sci-test)
  add_executable(blofeld-sci-reference-test test/mcu_sci_reference_test.cpp
    test/mcu_sci_reference_owner.cpp)
  target_link_libraries(blofeld-sci-reference-test PRIVATE blofeld_core)
  add_test(NAME mcu_sci_reference COMMAND blofeld-sci-reference-test)

  add_executable(blofeld-timer-test test/mcu_timer_test.cpp)
  target_link_libraries(blofeld-timer-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_timer COMMAND blofeld-timer-test)
  add_executable(blofeld-mcu-timer-span-test test/mcu_timer_span_test.cpp)
  target_link_libraries(blofeld-mcu-timer-span-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_timer_span COMMAND blofeld-mcu-timer-span-test)
  add_executable(blofeld-pacnt-storage-test test/mcu_pacnt_storage_test.cpp)
  target_link_libraries(blofeld-pacnt-storage-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_pacnt_storage COMMAND blofeld-pacnt-storage-test)

  add_executable(blofeld-pin-routing-test test/mcu_pin_routing_test.cpp)
  target_link_libraries(blofeld-pin-routing-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_pin_routing COMMAND blofeld-pin-routing-test)
  add_executable(blofeld-interrupt-test test/interrupt_test.cpp)
  target_link_libraries(blofeld-interrupt-test PRIVATE blofeld_core)
  add_test(NAME mcu_interrupt COMMAND blofeld-interrupt-test)
  add_executable(blofeld-wai-test test/wai_test.cpp)
  target_link_libraries(blofeld-wai-test PRIVATE blofeld_core)
  add_test(NAME mcu_wai COMMAND blofeld-wai-test)
  add_executable(blofeld-st7565-test test/st7565_test.cpp)
  target_link_libraries(blofeld-st7565-test PRIVATE blofeld_display)
  add_test(NAME st7565 COMMAND blofeld-st7565-test)
  add_executable(blofeld-st7565-parallel-test test/st7565_parallel_test.cpp)
  target_link_libraries(blofeld-st7565-parallel-test PRIVATE blofeld_display)
  add_test(NAME st7565_parallel COMMAND blofeld-st7565-parallel-test)
  add_executable(blofeld-lcd-image-test test/lcd_image_test.cpp)
  target_link_libraries(blofeld-lcd-image-test PRIVATE blofeld_lcd_export)
  add_test(NAME lcd_image COMMAND blofeld-lcd-image-test)
  add_executable(blofeld-lcd-export-test test/lcd_export_test.cpp)
  target_link_libraries(blofeld-lcd-export-test PRIVATE blofeld_lcd_export)
  add_test(NAME lcd_export COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/lcd_export.py" $<TARGET_FILE:blofeld-lcd-export-test>)
  add_test(NAME display_gui COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/display_gui.py")
  set_tests_properties(display_gui PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-rti-test test/mcu_rti_test.cpp)
  target_link_libraries(blofeld-rti-test PRIVATE blofeld_core)
  add_test(NAME mcu_rti COMMAND blofeld-rti-test)
  add_executable(blofeld-memory-usb-test test/memory_usb_test.cpp)
  target_link_libraries(blofeld-memory-usb-test PRIVATE blofeld_core)
  add_test(NAME memory_usb COMMAND blofeld-memory-usb-test)
  add_executable(blofeld-usb-event-test test/usb_event_test.cpp)
  target_link_libraries(blofeld-usb-event-test PRIVATE blofeld_core)
  add_test(NAME usb_events COMMAND blofeld-usb-event-test)
  add_executable(blofeld-usb-configuration-status-test test/usb_configuration_status_test.cpp)
  target_link_libraries(blofeld-usb-configuration-status-test PRIVATE blofeld_core)
  add_test(NAME usb_configuration_status COMMAND blofeld-usb-configuration-status-test)
  add_executable(blofeld-usb-standard-read-test test/usb_standard_read_test.cpp)
  target_link_libraries(blofeld-usb-standard-read-test PRIVATE blofeld_mcu)
  add_test(NAME usb_standard_read COMMAND blofeld-usb-standard-read-test)
  add_executable(blofeld-usb-device-status-test test/usb_device_status_test.cpp)
  target_link_libraries(blofeld-usb-device-status-test PRIVATE blofeld_mcu)
  add_test(NAME usb_device_status COMMAND blofeld-usb-device-status-test)
  add_executable(blofeld-usb-isr-test test/usb_isr_test.cpp)
  target_link_libraries(blofeld-usb-isr-test PRIVATE blofeld_core)
  add_test(NAME usb_cpu_isr COMMAND blofeld-usb-isr-test)
  add_executable(blofeld-boot-test test/original_boot_test.cpp)
  target_link_libraries(blofeld-boot-test PRIVATE blofeld_core)
  add_test(NAME original_mcu_boot COMMAND blofeld-boot-test
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-boot.trace")
  set_tests_properties(original_mcu_boot PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-mcu-sender-test test/original_mcu_sender_test.cpp)
  target_link_libraries(blofeld-mcu-sender-test PRIVATE blofeld_core)
  add_test(NAME original_mcu_sender COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-mcu-sender-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-mcu-sender.trace")
  set_tests_properties(original_mcu_sender PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME firmware_python
    COMMAND ${Python3_EXECUTABLE} -m unittest discover -s test -v)
  add_test(NAME source_guard
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/test/source_guard_test.py")
  set_tests_properties(source_guard PROPERTIES TIMEOUT 30)
  set_tests_properties(firmware_python PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    ENVIRONMENT "BLOFELD_FIRMWARE_TEST_BINARY=$<TARGET_FILE:blofeld-firmware-parser-test>")

endif()

install(TARGETS blofeld-headless RUNTIME DESTINATION bin)
install(PROGRAMS tools/blofeld-display DESTINATION bin COMPONENT Display)
install(FILES src/gui/__init__.py src/gui/frame.py src/gui/watch.py src/gui/display.py
  DESTINATION share/blofeld-emu/gui COMPONENT Display)
install(FILES packaging/org.blofeld_emu.Display.desktop
  DESTINATION share/applications COMPONENT Display)
install(FILES LICENSE DESTINATION share/doc/blofeld-emu COMPONENT Display)
install(FILES LICENSE THIRD_PARTY.txt DESTINATION share/doc/blofeld-emu)
install(DIRECTORY LICENSES/ DESTINATION share/doc/blofeld-emu/licenses)

file(GLOB_RECURSE BLOFELD_FORMAT_SOURCES CONFIGURE_DEPENDS
  "src/*.hpp" "src/*.cpp" "test/*.cpp" "test/*.hpp")
add_custom_target(format-check
  COMMAND "${BLOFELD_CLANG_FORMAT}" --dry-run --Werror ${BLOFELD_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
add_custom_target(quality
  COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
  DEPENDS format-check blofeld-startup-probe blofeld-mcu-ram-program-test blofeld-mcu-io-continuation-test
  USES_TERMINAL)

endif()

if(BUILD_TESTING)
  add_executable(blofeld-sci-fast-test test/mcu_sci_fast_test.cpp)
  target_link_libraries(blofeld-sci-fast-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_sci_fast COMMAND blofeld-sci-fast-test)
  add_executable(blofeld-sci-overlap-test test/mcu_sci_overlap_test.cpp)
  target_link_libraries(blofeld-sci-overlap-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_sci_overlap COMMAND blofeld-sci-overlap-test)
  add_dependencies(quality blofeld-sci-fast-test blofeld-sci-overlap-test)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-usb-functional-halt-test test/usb_functional_halt_test.cpp)
  target_link_libraries(blofeld-usb-functional-halt-test PRIVATE blofeld_mcu)
  add_test(NAME usb_functional_halt COMMAND blofeld-usb-functional-halt-test)
  add_dependencies(quality blofeld-usb-functional-halt-test)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-usb-endpoint-status-test test/usb_endpoint_status_test.cpp)
  target_link_libraries(blofeld-usb-endpoint-status-test PRIVATE blofeld_mcu)
  add_test(NAME usb_endpoint_status COMMAND blofeld-usb-endpoint-status-test)
  add_executable(blofeld-usb-endpoint-status-native-test test/usb_endpoint_status_native_test.cpp)
  target_link_libraries(blofeld-usb-endpoint-status-native-test PRIVATE blofeld_usb_host blofeld_core)
  add_test(NAME usb_endpoint_status_native COMMAND blofeld-usb-endpoint-status-native-test)
  add_dependencies(quality blofeld-usb-endpoint-status-test blofeld-usb-endpoint-status-native-test)
endif()

if(BUILD_TESTING)
  add_test(NAME hardware_usb_capture
    COMMAND ${Python3_EXECUTABLE} -B "${CMAKE_CURRENT_SOURCE_DIR}/test/hardware_usb_capture_test.py" -v)
endif()

if(BUILD_TESTING)
  add_dependencies(quality blofeld-modifier-agu-test)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-dsp-gpio-pull-test test/dsp_gpio_pull_test.cpp)
  target_link_libraries(blofeld-dsp-gpio-pull-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_gpio_pull COMMAND blofeld-dsp-gpio-pull-test)
  set_tests_properties(dsp_gpio_pull PROPERTIES TIMEOUT 30)
  add_dependencies(quality blofeld-dsp-gpio-pull-test)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-nand-sequential-read-test test/nand_sequential_read_test.cpp)
  target_link_libraries(blofeld-nand-sequential-read-test PRIVATE blofeld_nand_operation_clock)
  add_test(NAME nand_sequential_read COMMAND blofeld-nand-sequential-read-test)
  set_tests_properties(nand_sequential_read PROPERTIES TIMEOUT 30)
  add_dependencies(quality blofeld-nand-sequential-read-test)
  add_executable(blofeld-nand-ce-pause-test test/nand_ce_pause_test.cpp)
  target_link_libraries(blofeld-nand-ce-pause-test PRIVATE blofeld_nand_operation_clock)
  add_test(NAME nand_ce_pause COMMAND blofeld-nand-ce-pause-test)
  set_tests_properties(nand_ce_pause PROPERTIES TIMEOUT 30)
  add_dependencies(quality blofeld-nand-ce-pause-test)
endif()

if(BUILD_TESTING)

  include(cmake/MasterHistoryTests.cmake)

endif()

if(BUILD_TESTING)
  add_executable(blofeld-gpio-projection-test test/runtime/gpio_projection_test.cpp)
  target_include_directories(blofeld-gpio-projection-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  target_link_libraries(blofeld-gpio-projection-test PRIVATE blofeld_vi)
  add_test(NAME gpio_projections COMMAND blofeld-gpio-projection-test)
  set_tests_properties(gpio_projections PROPERTIES TIMEOUT 10)
  add_dependencies(quality blofeld-gpio-projection-test)

  add_executable(blofeld-audio-deadline-test test/runtime/audio_deadline_test.cpp)
  target_include_directories(blofeld-audio-deadline-test PRIVATE
    "${BLOFELD_CPU12_ROOT}/test/support" "${BLOFELD_DSP56300_ROOT}/test")
  target_link_libraries(blofeld-audio-deadline-test PRIVATE blofeld_vi)
  add_test(NAME instrument_audio_deadlines COMMAND blofeld-audio-deadline-test)
  set_tests_properties(instrument_audio_deadlines PROPERTIES TIMEOUT 10)
  add_dependencies(quality blofeld-audio-deadline-test)

  add_executable(blofeld-mcu-dsp-gpio-peer-test test/mcu_dsp_gpio_peer_test.cpp)
  target_include_directories(blofeld-mcu-dsp-gpio-peer-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  target_link_libraries(blofeld-mcu-dsp-gpio-peer-test PRIVATE blofeld_vi)
  add_test(NAME mcu_dsp_gpio_peer COMMAND blofeld-mcu-dsp-gpio-peer-test)
  set_tests_properties(mcu_dsp_gpio_peer PROPERTIES TIMEOUT 30)

  add_dependencies(quality blofeld-mcu-dsp-gpio-peer-test)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-nand-write-store-test test/nand_write_store_test.cpp)
  target_link_libraries(blofeld-nand-write-store-test PRIVATE blofeld_nand_write_store)
  add_test(NAME nand_write_store COMMAND blofeld-nand-write-store-test)
  set_tests_properties(nand_write_store PROPERTIES TIMEOUT 30)

  add_dependencies(quality blofeld-nand-write-store-test
    )
endif()

include(cmake/McuInspectionTests.cmake)

include(cmake/TelemetryTests.cmake)

if(BUILD_TESTING)
  get_property(blofeld_build_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
  set(blofeld_hardened_executables "")
  foreach(target IN LISTS blofeld_build_targets)
    get_target_property(kind ${target} TYPE)
    if(kind STREQUAL "EXECUTABLE")
      add_dependencies(quality ${target})
      string(APPEND blofeld_hardened_executables "$<TARGET_FILE:${target}>\n")
    endif()
  endforeach()
  file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/hardening-executables.txt"
    CONTENT "${blofeld_hardened_executables}")
endif()
