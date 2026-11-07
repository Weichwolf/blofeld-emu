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
set(GHIDRA_ROOT "$ENV{GHIDRA_ROOT}" CACHE PATH "Ghidra distribution with native SLEIGH sources")
if(NOT GHIDRA_ROOT)
  file(GLOB GHIDRA_CANDIDATES "$ENV{HOME}/.local/opt/ghidra_*_PUBLIC")
  list(SORT GHIDRA_CANDIDATES COMPARE NATURAL ORDER DESCENDING)
  foreach(candidate IN LISTS GHIDRA_CANDIDATES)
    if(EXISTS "${candidate}/Ghidra/Features/Decompiler/src/decompile/cpp/sleigh.cc")
      set(GHIDRA_ROOT "${candidate}")
      break()
    endif()
  endforeach()
endif()
set(SLEIGH_SOURCE "${GHIDRA_ROOT}/Ghidra/Features/Decompiler/src/decompile/cpp")
if(NOT EXISTS "${SLEIGH_SOURCE}/sleigh.cc")
  message(FATAL_ERROR "Set GHIDRA_ROOT to a Ghidra distribution containing native SLEIGH sources")
endif()
include(cmake/Zlib.cmake)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Same native library source set as Ghidra's LIBSLA_NAMES; no JVM is needed.
set(SLEIGH_NAMES xml marshal space float address pcoderaw translate opcodes
    globalcontext sleigh pcodeparse pcodecompile sleighbase slghsymbol
    slghpatexpress slghpattern semantics context slaformat compression
    filemanage loadimage emulate opbehavior)
set(SLEIGH_FILES)
foreach(name IN LISTS SLEIGH_NAMES)
  list(APPEND SLEIGH_FILES "${SLEIGH_SOURCE}/${name}.cc")
endforeach()
find_program(BLOFELD_PATCH NAMES patch REQUIRED)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(SHA256 "${SLEIGH_SOURCE}/memstate.cc" BLOFELD_MEMORY_SHA256)
if(NOT BLOFELD_MEMORY_SHA256 STREQUAL "ef68ee18511cac1bec55a602c64e43f57bc4755ca00e8204f411434d51d27f92")
  message(FATAL_ERROR "Native SLEIGH memory patch requires the verified Ghidra 12.1.2 source")
endif()
set(BLOFELD_MEMORY_PATCH "${CMAKE_CURRENT_SOURCE_DIR}/patches/ghidra-byte-memory.patch")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${BLOFELD_MEMORY_PATCH}" "${SLEIGH_SOURCE}/memstate.cc")
execute_process(
  COMMAND "${BLOFELD_PATCH}" --batch --silent
    --input "${BLOFELD_MEMORY_PATCH}"
    --output "${CMAKE_CURRENT_BINARY_DIR}/generated/memstate.cc"
    "${SLEIGH_SOURCE}/memstate.cc"
  COMMAND_ERROR_IS_FATAL ANY)
list(APPEND SLEIGH_FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/memstate.cc")
add_library(blofeld_sleigh STATIC ${SLEIGH_FILES})
target_include_directories(blofeld_sleigh SYSTEM PUBLIC "${SLEIGH_SOURCE}")
target_link_libraries(blofeld_sleigh PUBLIC ZLIB::ZLIB)
blofeld_harden_native(blofeld_sleigh)

if(DEFINED blofeld_dependency_sanitizers)
  target_compile_options(blofeld_sleigh PRIVATE
    "-fsanitize=${blofeld_dependency_sanitizers}" -fno-sanitize-recover=all -fno-omit-frame-pointer -g)
endif()
# Apply clang-tidy to our targets, after the third-party target was created.
find_package(Git REQUIRED)
find_package(Threads REQUIRED)
include(cmake/Dsp56300.cmake)
include(cmake/PulseAudio.cmake)
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

include(cmake/Hc12Spec.cmake)
include(cmake/Hcs08Spec.cmake)
include(cmake/DspComponents.cmake)
include(cmake/ReferenceRomFixtures.cmake)
include(cmake/SerialPcm.cmake)
add_library(blofeld_hcs08 src/keyboard/hcs08.cpp src/keyboard/hcs08_interrupts.cpp src/keyboard/hcs08_registers.cpp src/keyboard/hcs08_ccr.cpp)
target_include_directories(blofeld_hcs08 PUBLIC src)
target_link_libraries(blofeld_hcs08 PUBLIC blofeld_firmware PRIVATE blofeld_sleigh)
add_dependencies(blofeld_hcs08 blofeld_hcs08_spec)
add_library(blofeld_hcs08_icg src/keyboard/hcs08_icg.cpp src/keyboard/hcs08_icg_events.cpp)
target_include_directories(blofeld_hcs08_icg PUBLIC src)
target_link_libraries(blofeld_hcs08_icg PUBLIC blofeld_firmware)
add_library(blofeld_hcs08_icg_clocks src/keyboard/hcs08_icg_clocks.cpp)
target_include_directories(blofeld_hcs08_icg_clocks PUBLIC src)
target_link_libraries(blofeld_hcs08_icg_clocks PUBLIC blofeld_firmware)
add_library(blofeld_hcs08_icg_status src/keyboard/hcs08_icg_status.cpp)
target_include_directories(blofeld_hcs08_icg_status PUBLIC src)
target_link_libraries(blofeld_hcs08_icg_status PUBLIC blofeld_hcs08_icg)
add_library(blofeld_hcs08_icg_history src/keyboard/hcs08_icg_history.cpp)
target_include_directories(blofeld_hcs08_icg_history PUBLIC src)
target_link_libraries(blofeld_hcs08_icg_history PUBLIC blofeld_hcs08_icg_status)
add_library(blofeld_hcs08_tpm src/keyboard/hcs08_tpm.cpp)
target_include_directories(blofeld_hcs08_tpm PUBLIC src)
target_link_libraries(blofeld_hcs08_tpm PUBLIC blofeld_firmware)
add_library(blofeld_hcs08_cop src/keyboard/hcs08_cop.cpp)
target_include_directories(blofeld_hcs08_cop PUBLIC src)
target_link_libraries(blofeld_hcs08_cop PUBLIC blofeld_firmware)

add_library(blofeld_hcs08_spi src/keyboard/hcs08_spi.cpp src/keyboard/hcs08_spi_registers.cpp)
target_include_directories(blofeld_hcs08_spi PUBLIC src)
target_link_libraries(blofeld_hcs08_spi PUBLIC blofeld_firmware)
add_library(blofeld_hcs08_spi_clock src/keyboard/hcs08_spi_clock.cpp)
target_include_directories(blofeld_hcs08_spi_clock PUBLIC src)
target_link_libraries(blofeld_hcs08_spi_clock PUBLIC blofeld_hcs08_spi)
add_library(blofeld_hcs08_adc src/keyboard/hcs08_adc.cpp src/keyboard/hcs08_adc_registers.cpp)
target_include_directories(blofeld_hcs08_adc PUBLIC src)
target_link_libraries(blofeld_hcs08_adc PUBLIC blofeld_firmware)
add_library(blofeld_hcs08_flash src/keyboard/hcs08_flash.cpp src/keyboard/hcs08_flash_registers.cpp
  src/keyboard/hcs08_flash_array.cpp src/keyboard/hcs08_flash_reset.cpp
  src/keyboard/hcs08_flash_vectors.cpp)
target_include_directories(blofeld_hcs08_flash PUBLIC src)
target_link_libraries(blofeld_hcs08_flash PUBLIC blofeld_firmware)
add_library(blofeld_hcs08_bdc src/keyboard/hcs08_bdc.cpp src/keyboard/hcs08_bdc_commands.cpp)
target_include_directories(blofeld_hcs08_bdc PUBLIC src)
target_link_libraries(blofeld_hcs08_bdc PUBLIC blofeld_hcs08 blofeld_hcs08_flash)
add_library(blofeld_hcs08_bdc_wire src/keyboard/hcs08_bdc_wire.cpp src/keyboard/hcs08_bdc_sync.cpp)
target_include_directories(blofeld_hcs08_bdc_wire PUBLIC src)
target_link_libraries(blofeld_hcs08_bdc_wire PUBLIC blofeld_hcs08_bdc)
add_library(blofeld_hcs08_reset src/keyboard/hcs08_reset.cpp)
target_include_directories(blofeld_hcs08_reset PUBLIC src)
target_link_libraries(blofeld_hcs08_reset PUBLIC blofeld_hcs08_bdc)
add_library(blofeld_hcs08_run_clock src/keyboard/hcs08_instruction_timing.cpp src/keyboard/hcs08_run_clock.cpp)
target_include_directories(blofeld_hcs08_run_clock PUBLIC src)
target_link_libraries(blofeld_hcs08_run_clock PUBLIC blofeld_hcs08_bdc)
add_library(blofeld_hcs08_spi_run_clock src/keyboard/hcs08_spi_run_clock.cpp)
target_include_directories(blofeld_hcs08_spi_run_clock PUBLIC src)
target_link_libraries(blofeld_hcs08_spi_run_clock PUBLIC blofeld_hcs08_bdc blofeld_hcs08_spi_clock blofeld_hcs08_run_clock)
add_library(blofeld_hcs08_gpio src/keyboard/hcs08_gpio.cpp)
target_include_directories(blofeld_hcs08_gpio PUBLIC src)
target_link_libraries(blofeld_hcs08_gpio PUBLIC blofeld_gpio_pads)
add_library(blofeld_hcs08_board_memory src/board/hcs08_board_memory.cpp
  src/board/hcs08_board_peripherals.cpp src/board/hcs08_board_clocks.cpp
  src/board/hcs08_board_bdc.cpp)
target_include_directories(blofeld_hcs08_board_memory PUBLIC src)
target_link_libraries(blofeld_hcs08_board_memory PUBLIC blofeld_hcs08_gpio
  blofeld_hcs08_icg_history blofeld_hcs08_adc blofeld_hcs08_spi blofeld_hcs08_tpm
  blofeld_hcs08_cop blofeld_hcs08_flash)
add_library(blofeld_hcs08_board src/board/hcs08_board.cpp src/board/hcs08_board_reset.cpp)
target_include_directories(blofeld_hcs08_board PUBLIC src)
target_link_libraries(blofeld_hcs08_board PUBLIC blofeld_hcs08_board_memory blofeld_hcs08_run_clock)
add_library(blofeld_hcs08_board_wire src/board/hcs08_board_wire.cpp)
target_include_directories(blofeld_hcs08_board_wire PUBLIC src)
target_link_libraries(blofeld_hcs08_board_wire PUBLIC blofeld_hcs08_board blofeld_hcs08_bdc_wire)
add_library(blofeld_hcs08_board_serial src/board/hcs08_board_serial.cpp)
target_include_directories(blofeld_hcs08_board_serial PUBLIC src)
target_link_libraries(blofeld_hcs08_board_serial PUBLIC blofeld_hcs08_board blofeld_hcs08_spi_run_clock)
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
add_library(blofeld_nand_read_timing src/storage/nand_read_timing.cpp src/storage/nand_read_ce_window.cpp
  src/storage/nand_status_read.cpp)
target_include_directories(blofeld_nand_read_timing PUBLIC src)
target_link_libraries(blofeld_nand_read_timing PUBLIC blofeld_nand)
add_library(blofeld_nand_write_timing src/storage/nand_write_timing.cpp src/storage/nand_write_ce_window.cpp)
target_include_directories(blofeld_nand_write_timing PUBLIC src)
target_link_libraries(blofeld_nand_write_timing PUBLIC blofeld_nand)
add_library(blofeld_nand_read_setup src/storage/nand_read_setup.cpp src/storage/nand_setup_ce_window.cpp)
target_include_directories(blofeld_nand_read_setup PUBLIC src)
target_link_libraries(blofeld_nand_read_setup PUBLIC blofeld_nand)
add_library(blofeld_nand_store src/storage/nand_file.cpp src/storage/nand_store.cpp)
target_include_directories(blofeld_nand_store PUBLIC src)
target_link_libraries(blofeld_nand_store PUBLIC blofeld_nand PRIVATE blofeld_private_files)
add_library(blofeld_nand_write_store src/storage/nand_write_store.cpp)
target_include_directories(blofeld_nand_write_store PUBLIC src)
target_link_libraries(blofeld_nand_write_store PUBLIC blofeld_nand_write_medium blofeld_nand_store
  PRIVATE blofeld_private_files)
add_library(blofeld_nand_peer src/board/nand_peer.cpp src/board/nand_peer_medium.cpp
  src/board/nand_bus_decode.cpp)
target_include_directories(blofeld_nand_peer PUBLIC src)
target_link_libraries(blofeld_nand_peer PUBLIC blofeld_dsp_machine
  blofeld_nand_write_medium blofeld_nand_write_store blofeld_nand_read_timing blofeld_nand_write_timing
  blofeld_nand_read_setup blofeld_dsp_peripheral_calendar)
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
  src/usb/usb_device_probe.cpp src/usb/usb_descriptors.cpp)
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
add_library(blofeld_hc12_cycles src/mcu/hc12_cycle_plan.cpp src/mcu/hc12_cycle_page2.cpp src/mcu/hc12_cycle_bus.cpp)
target_include_directories(blofeld_hc12_cycles PUBLIC src)
target_link_libraries(blofeld_hc12_cycles PUBLIC blofeld_firmware)
add_library(blofeld_midi_serial src/midi/midi_serial.cpp src/midi/midi_serial_receiver.cpp)
target_include_directories(blofeld_midi_serial PUBLIC src)
target_link_libraries(blofeld_midi_serial PUBLIC blofeld_firmware)
add_library(blofeld_midi_timed src/midi/midi_timed.cpp)
target_include_directories(blofeld_midi_timed PUBLIC src)
target_link_libraries(blofeld_midi_timed PUBLIC blofeld_midi_serial)
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
add_library(blofeld_mcu_midi src/mcu/mcu_midi.cpp src/mcu/mcu_clocked_midi.cpp)
target_include_directories(blofeld_mcu_midi PUBLIC src)
target_link_libraries(blofeld_mcu_midi PUBLIC blofeld_mcu blofeld_midi_serial)
add_library(blofeld_mcu_midi_output src/mcu/mcu_midi_output.cpp)
target_include_directories(blofeld_mcu_midi_output PUBLIC src)
target_link_libraries(blofeld_mcu_midi_output PUBLIC blofeld_mcu blofeld_midi_serial)
add_dependencies(blofeld_core blofeld_hc12_spec)
target_include_directories(blofeld_core PUBLIC src)
target_link_libraries(blofeld_core PUBLIC blofeld_mcu)
set(blofeld_independent_native_args --native dsp56300_core=$<TARGET_FILE:dsp56300_core>)
target_link_libraries(blofeld_core PUBLIC cpu12::core)
list(APPEND blofeld_independent_native_args --native cpu12_core=$<TARGET_FILE:cpu12_core>)
target_compile_definitions(blofeld_core PUBLIC
  BLOFELD_DEFAULT_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
add_library(blofeld_host_midi src/midi/host_midi.cpp)
target_include_directories(blofeld_host_midi PUBLIC src)
target_link_libraries(blofeld_host_midi PUBLIC blofeld_quality)
add_library(blofeld_host_midi_port src/midi/host_midi_port.cpp
  src/midi/host_midi_port_setup.cpp src/midi/host_midi_port_io.cpp
  src/midi/sequence_encode.cpp src/midi/sequence_decode.cpp src/midi/sequence_pool.cpp)
target_include_directories(blofeld_host_midi_port PUBLIC src)
target_link_libraries(blofeld_host_midi_port PUBLIC blofeld_quality)
add_library(blofeld_reference_board src/board/mcu_dsp_gpio_peer.cpp src/board/reference_gpio_peer.cpp
  src/board/reference_config.cpp src/board/reference_square_clock.cpp src/board/reference_audio_clock.cpp
  src/board/reference_board.cpp src/board/reference_pipeline.cpp src/board/reference_wiring.cpp src/board/reference_returns.cpp src/board/reference_execution.cpp
  src/board/reference_reset_source.cpp src/board/reference_lcd.cpp src/board/reference_b_pulls.cpp
  src/board/reference_dsp_inputs.cpp src/board/reference_dsp_clocks.cpp src/board/reference_pcm.cpp src/board/reference_audio.cpp
  src/board/reference_midi.cpp src/board/reference_midi_output.cpp src/board/reference_dsp_midi.cpp)
target_include_directories(blofeld_reference_board PUBLIC src)
target_link_libraries(blofeld_reference_board PUBLIC blofeld_mcu_dsp_runtime blofeld_private_files
  blofeld_pcm_transport blofeld_mcu_midi blofeld_mcu_midi_output blofeld_midi_timed blofeld_usbip_tcp
  blofeld_nand_peer blofeld_nand_store blofeld_lcd_export)
add_executable(blofeld-headless src/app/main.cpp src/app/host_midi_runner.cpp src/app/host_display_pump.cpp)
target_link_libraries(blofeld-headless PRIVATE blofeld_reference_board blofeld_host_midi blofeld_host_midi_session blofeld_audio_runtime blofeld_sample_library blofeld_device_identity)

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
  # One checked translation unit for the shared target-memory reference fixture.
  add_library(blofeld_hcs08_programmer_fixture OBJECT test/hcs08_programmer_fixture.cpp)
  target_link_libraries(blofeld_hcs08_programmer_fixture PUBLIC
    blofeld_hcs08 blofeld_hcs08_flash blofeld_hcs08_icg)
  add_executable(blofeld-hcs08-core-test test/hcs08_core_test.cpp)
  target_link_libraries(blofeld-hcs08-core-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-core-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_core COMMAND blofeld-hcs08-core-test)
  add_executable(blofeld-hcs08-board-memory-test test/hcs08_board_memory_test.cpp)
  target_link_libraries(blofeld-hcs08-board-memory-test PRIVATE blofeld_hcs08_board_memory)
  add_test(NAME hcs08_board_memory COMMAND blofeld-hcs08-board-memory-test)
  add_executable(blofeld-original-hcs08-board-memory-test test/original_hcs08_board_memory_test.cpp)
  target_link_libraries(blofeld-original-hcs08-board-memory-test PRIVATE
    blofeld_hcs08_board_memory blofeld_hcs08)
  target_compile_definitions(blofeld-original-hcs08-board-memory-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_board_memory COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_board_memory.py"
    $<TARGET_FILE:blofeld-original-hcs08-board-memory-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-board-memory.trace")
  set_tests_properties(original_hcs08_board_memory PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-hcs08-board-test test/hcs08_board_test.cpp)
  target_link_libraries(blofeld-hcs08-board-test PRIVATE blofeld_hcs08_board)
  target_compile_definitions(blofeld-hcs08-board-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_board COMMAND blofeld-hcs08-board-test)
  set_tests_properties(hcs08_board PROPERTIES TIMEOUT 90)
  add_executable(blofeld-hcs08-board-irq-test test/hcs08_board_irq_test.cpp)
  target_link_libraries(blofeld-hcs08-board-irq-test PRIVATE blofeld_hcs08_board_memory)
  add_test(NAME hcs08_board_irq COMMAND blofeld-hcs08-board-irq-test)
  set_tests_properties(hcs08_board_irq PROPERTIES TIMEOUT 90)
  add_executable(blofeld-hcs08-board-irq-native-test test/hcs08_board_irq_native_test.cpp)
  target_link_libraries(blofeld-hcs08-board-irq-native-test PRIVATE blofeld_hcs08_board_serial)
  target_compile_definitions(blofeld-hcs08-board-irq-native-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_board_irq_native COMMAND blofeld-hcs08-board-irq-native-test)
  set_tests_properties(hcs08_board_irq_native PROPERTIES TIMEOUT 180)
  add_executable(blofeld-hcs08-flash-vector-test test/hcs08_flash_vector_test.cpp)
  target_link_libraries(blofeld-hcs08-flash-vector-test PRIVATE blofeld_hcs08_flash)
  add_test(NAME hcs08_flash_vector COMMAND blofeld-hcs08-flash-vector-test)
  set_tests_properties(hcs08_flash_vector PROPERTIES TIMEOUT 90)
  add_executable(blofeld-hcs08-vector-irq-test test/hcs08_vector_irq_test.cpp)
  target_link_libraries(blofeld-hcs08-vector-irq-test PRIVATE blofeld_hcs08_board_serial)
  target_compile_definitions(blofeld-hcs08-vector-irq-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_vector_irq COMMAND blofeld-hcs08-vector-irq-test)
  set_tests_properties(hcs08_vector_irq PROPERTIES TIMEOUT 180)
  add_executable(blofeld-hcs08-vector-atomic-test test/hcs08_vector_atomic_test.cpp)
  target_link_libraries(blofeld-hcs08-vector-atomic-test PRIVATE blofeld_hcs08_board_serial)
  target_compile_definitions(blofeld-hcs08-vector-atomic-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_vector_atomic COMMAND blofeld-hcs08-vector-atomic-test)
  set_tests_properties(hcs08_vector_atomic PROPERTIES TIMEOUT 180)
  add_executable(blofeld-hcs08-decimal-test test/hcs08_decimal_test.cpp)
  target_link_libraries(blofeld-hcs08-decimal-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-decimal-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_decimal COMMAND blofeld-hcs08-decimal-test)
  set_tests_properties(hcs08_decimal PROPERTIES TIMEOUT 90)
  foreach(swi_test IN ITEMS swi swi_boundary)
    string(REPLACE "_" "-" swi_target "${swi_test}")
    add_executable(blofeld-hcs08-${swi_target}-test test/hcs08_${swi_test}_test.cpp)
    target_link_libraries(blofeld-hcs08-${swi_target}-test PRIVATE blofeld_hcs08_flash blofeld_hcs08)
    target_compile_definitions(blofeld-hcs08-${swi_target}-test PRIVATE
      BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
    add_test(NAME hcs08_${swi_test} COMMAND blofeld-hcs08-${swi_target}-test)
    set_tests_properties(hcs08_${swi_test} PROPERTIES TIMEOUT 90)
  endforeach()
  add_executable(blofeld-hcs08-serial-foreground-test test/hcs08_serial_foreground_test.cpp)
  target_link_libraries(blofeld-hcs08-serial-foreground-test PRIVATE blofeld_hcs08_board_serial)
  target_compile_definitions(blofeld-hcs08-serial-foreground-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_serial_foreground COMMAND blofeld-hcs08-serial-foreground-test)
  set_tests_properties(hcs08_serial_foreground PROPERTIES TIMEOUT 180)
  add_executable(blofeld-original-hcs08-serial-foreground-test test/original_hcs08_serial_foreground_test.cpp)
  target_link_libraries(blofeld-original-hcs08-serial-foreground-test PRIVATE blofeld_hcs08_board_serial)
  target_compile_definitions(blofeld-original-hcs08-serial-foreground-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_serial_foreground COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_serial_foreground.py"
    $<TARGET_FILE:blofeld-original-hcs08-serial-foreground-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-serial-foreground.trace")
  set_tests_properties(original_hcs08_serial_foreground PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 960)
  add_executable(blofeld-original-hcs08-board-test test/original_hcs08_board_test.cpp)
  target_link_libraries(blofeld-original-hcs08-board-test PRIVATE blofeld_hcs08_board)
  target_compile_definitions(blofeld-original-hcs08-board-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_board COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_board.py"
    $<TARGET_FILE:blofeld-original-hcs08-board-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-board.trace")
  set_tests_properties(original_hcs08_board PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 960)
  add_executable(blofeld-hcs08-board-reset-test test/hcs08_board_reset_test.cpp)
  target_link_libraries(blofeld-hcs08-board-reset-test PRIVATE blofeld_hcs08_board)
  target_compile_definitions(blofeld-hcs08-board-reset-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_board_reset COMMAND blofeld-hcs08-board-reset-test)
  set_tests_properties(hcs08_board_reset PROPERTIES TIMEOUT 90)
  add_executable(blofeld-original-hcs08-board-reset-test test/original_hcs08_board_reset_test.cpp)
  target_link_libraries(blofeld-original-hcs08-board-reset-test PRIVATE blofeld_hcs08_board)
  target_compile_definitions(blofeld-original-hcs08-board-reset-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_board_resets COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_board_resets.py"
    $<TARGET_FILE:blofeld-original-hcs08-board-reset-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-board-resets.trace")
  set_tests_properties(original_hcs08_board_resets PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-hcs08-board-wire-test test/hcs08_board_wire_test.cpp)
  target_link_libraries(blofeld-hcs08-board-wire-test PRIVATE blofeld_hcs08_board_wire)
  target_compile_definitions(blofeld-hcs08-board-wire-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_board_wire COMMAND blofeld-hcs08-board-wire-test)
  set_tests_properties(hcs08_board_wire PROPERTIES TIMEOUT 90)
  add_executable(blofeld-original-keyboard-board-reset-test
    test/original_keyboard_board_reset_test.cpp test/keyboard_board_reset_fixture.cpp)
  target_link_libraries(blofeld-original-keyboard-board-reset-test PRIVATE
    blofeld_hcs08_board_wire blofeld_hcs08_reset blofeld_core)
  target_compile_definitions(blofeld-original-keyboard-board-reset-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_keyboard_board_reset COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_board_reset.py"
    $<TARGET_FILE:blofeld-original-keyboard-board-reset-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-board-reset.trace")
  set_tests_properties(original_keyboard_board_reset PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 420)
  add_executable(blofeld-hcs08-board-status-test test/hcs08_board_status_test.cpp)
  target_link_libraries(blofeld-hcs08-board-status-test PRIVATE blofeld_hcs08_board_wire)
  target_compile_definitions(blofeld-hcs08-board-status-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_board_status COMMAND blofeld-hcs08-board-status-test)
  set_tests_properties(hcs08_board_status PROPERTIES TIMEOUT 90)
  add_executable(blofeld-hcs08-board-serial-test test/hcs08_board_serial_test.cpp)
  target_link_libraries(blofeld-hcs08-board-serial-test PRIVATE blofeld_hcs08_board_serial)
  target_compile_definitions(blofeld-hcs08-board-serial-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_board_serial COMMAND blofeld-hcs08-board-serial-test)
  set_tests_properties(hcs08_board_serial PROPERTIES TIMEOUT 120)
  add_executable(blofeld-original-keyboard-board-upload-test
    test/original_keyboard_board_upload_test.cpp test/keyboard_board_upload_fixture.cpp)
  target_link_libraries(blofeld-original-keyboard-board-upload-test PRIVATE
    blofeld_hcs08_board_wire blofeld_core)
  target_compile_definitions(blofeld-original-keyboard-board-upload-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_keyboard_board_upload COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_board_upload.py"
    $<TARGET_FILE:blofeld-original-keyboard-board-upload-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-board-upload.trace")
  set_tests_properties(original_keyboard_board_upload PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_executable(blofeld-hcs08-arithmetic-test test/hcs08_arithmetic_test.cpp)
  target_link_libraries(blofeld-hcs08-arithmetic-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-arithmetic-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_arithmetic COMMAND blofeld-hcs08-arithmetic-test)
  set_tests_properties(hcs08_arithmetic PROPERTIES TIMEOUT 120)
  add_executable(blofeld-original-hcs08-test test/original_hcs08_test.cpp)
  target_link_libraries(blofeld-original-hcs08-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-original-hcs08-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08 COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08.py"
    $<TARGET_FILE:blofeld-original-hcs08-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08.trace")
  set_tests_properties(original_hcs08 PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-hcs08-gpio-test test/hcs08_gpio_test.cpp)
  target_link_libraries(blofeld-hcs08-gpio-test PRIVATE blofeld_hcs08_gpio)
  add_test(NAME hcs08_gpio COMMAND blofeld-hcs08-gpio-test)
  add_executable(blofeld-original-hcs08-gpio-test test/original_hcs08_gpio_test.cpp)
  target_link_libraries(blofeld-original-hcs08-gpio-test PRIVATE blofeld_hcs08 blofeld_hcs08_gpio)
  target_compile_definitions(blofeld-original-hcs08-gpio-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_gpio COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_gpio.py"
    $<TARGET_FILE:blofeld-original-hcs08-gpio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-gpio.trace")
  set_tests_properties(original_hcs08_gpio PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-original-hcs08-clock-startup-test test/original_hcs08_clock_startup_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-clock-startup-test PRIVATE blofeld_hcs08
    blofeld_hcs08_flash blofeld_hcs08_gpio blofeld_hcs08_icg blofeld_hcs08_spi blofeld_hcs08_adc)
  target_compile_definitions(blofeld-original-hcs08-clock-startup-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_clock_startup COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_clock_startup.py"
    $<TARGET_FILE:blofeld-original-hcs08-clock-startup-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-clock-startup.trace")
  set_tests_properties(original_hcs08_clock_startup PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_test(NAME original_hcs08_cold_startup COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_clock_startup.py"
    $<TARGET_FILE:blofeld-original-hcs08-clock-startup-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-cold-startup.trace" power-on)
  set_tests_properties(original_hcs08_cold_startup PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-hcs08-spi-test test/hcs08_spi_test.cpp)
  target_link_libraries(blofeld-hcs08-spi-test PRIVATE blofeld_hcs08_spi blofeld_dsp_peripherals)
  add_executable(blofeld-hcs08-spi-clock-test test/hcs08_spi_clock_test.cpp)
  target_link_libraries(blofeld-hcs08-spi-clock-test PRIVATE blofeld_hcs08_spi_clock blofeld_dsp_peripherals)
  add_test(NAME hcs08_spi_clock COMMAND blofeld-hcs08-spi-clock-test)
  add_executable(blofeld-hcs08-spi-run-clock-test test/hcs08_spi_run_clock_test.cpp)
  target_link_libraries(blofeld-hcs08-spi-run-clock-test PRIVATE blofeld_hcs08_spi_run_clock)
  target_compile_definitions(blofeld-hcs08-spi-run-clock-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_spi_run_clock COMMAND blofeld-hcs08-spi-run-clock-test)
  add_executable(blofeld-original-hcs08-spi-run-clock-test test/original_hcs08_spi_run_clock_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-spi-run-clock-test PRIVATE blofeld_hcs08_spi_run_clock
    blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio blofeld_hcs08_adc blofeld_dsp_peripherals
    blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-spi-run-clock-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi_run_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_spi_run_clock.py"
    $<TARGET_FILE:blofeld-original-hcs08-spi-run-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-run-clock.trace")
  set_tests_properties(original_hcs08_spi_run_clock PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 210)
  add_test(NAME original_hcs08_spi_jump COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_spi_jump.py"
    $<TARGET_FILE:blofeld-original-hcs08-spi-run-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-jump.trace")
  set_tests_properties(original_hcs08_spi_jump PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 210)
  add_test(NAME original_hcs08_spi_scanner COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_spi_scanner.py"
    $<TARGET_FILE:blofeld-original-hcs08-spi-run-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-scanner.trace")
  set_tests_properties(original_hcs08_spi_scanner PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 270)
  foreach(store IN ITEMS word move)
    add_executable(blofeld-original-hcs08-spi-${store}-test
      test/original_hcs08_spi_${store}_test.cpp test/hcs08_clock_startup_fixture.cpp)
    target_link_libraries(blofeld-original-hcs08-spi-${store}-test PRIVATE
      blofeld_hcs08_spi_run_clock blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio
      blofeld_hcs08_adc blofeld_dsp_peripherals blofeld_private_files)
    target_compile_definitions(blofeld-original-hcs08-spi-${store}-test PRIVATE
      BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
    add_test(NAME original_hcs08_spi_${store} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_spi_stores.py"
      $<TARGET_FILE:blofeld-original-hcs08-spi-${store}-test>
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-${store}.trace" ${store})
  endforeach()
  set_tests_properties(original_hcs08_spi_word PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 330)
  set_tests_properties(original_hcs08_spi_move PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 630)
  add_test(NAME hcs08_spi COMMAND blofeld-hcs08-spi-test)
  add_executable(blofeld-original-hcs08-spi-test test/original_hcs08_spi_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-spi-test PRIVATE blofeld_hcs08_programmer_fixture blofeld_hcs08
    blofeld_hcs08_flash blofeld_hcs08_gpio blofeld_hcs08_icg blofeld_hcs08_spi blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-spi-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_spi.py"
    $<TARGET_FILE:blofeld-original-hcs08-spi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi.trace")
  set_tests_properties(original_hcs08_spi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-hcs08-adc-test test/hcs08_adc_test.cpp)
  add_executable(blofeld-hcs08-adc-flags-test test/hcs08_adc_flags_test.cpp)
  target_link_libraries(blofeld-hcs08-adc-flags-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-adc-flags-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_adc_flags COMMAND blofeld-hcs08-adc-flags-test)
  target_link_libraries(blofeld-hcs08-adc-test PRIVATE blofeld_hcs08_adc)
  add_test(NAME hcs08_adc COMMAND blofeld-hcs08-adc-test)
  add_executable(blofeld-original-hcs08-adc-test test/original_hcs08_adc_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-adc-test PRIVATE blofeld_hcs08_programmer_fixture
    blofeld_hcs08_gpio blofeld_hcs08_spi blofeld_hcs08_adc blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-adc-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_adc COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_adc.py"
    $<TARGET_FILE:blofeld-original-hcs08-adc-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-adc.trace")
  set_tests_properties(original_hcs08_adc PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)

  add_executable(blofeld-original-hcs08-adc-clock-test test/original_hcs08_adc_clock_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-adc-clock-test PRIVATE blofeld_hcs08_programmer_fixture
    blofeld_hcs08_run_clock blofeld_hcs08_gpio blofeld_hcs08_spi blofeld_hcs08_adc blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-adc-clock-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_adc_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_adc_clock.py"
    $<TARGET_FILE:blofeld-original-hcs08-adc-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-adc-clock.trace")
  set_tests_properties(original_hcs08_adc_clock PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)

  add_executable(blofeld-original-hcs08-pitch-event-test test/original_hcs08_pitch_event_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-pitch-event-test PRIVATE blofeld_hcs08_programmer_fixture
    blofeld_hcs08_run_clock blofeld_hcs08_gpio blofeld_hcs08_spi blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-pitch-event-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_pitch_event COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_pitch_event.py"
    $<TARGET_FILE:blofeld-original-hcs08-pitch-event-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-pitch-event.trace")
  set_tests_properties(original_hcs08_pitch_event PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_pitch_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_pitch_clock.py"
    $<TARGET_FILE:blofeld-original-hcs08-pitch-event-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-pitch-clock.trace")
  set_tests_properties(original_hcs08_pitch_clock PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-original-hcs08-calibration-test test/original_hcs08_calibration_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-calibration-test PRIVATE blofeld_hcs08_programmer_fixture
    blofeld_hcs08_run_clock blofeld_hcs08_gpio blofeld_hcs08_spi blofeld_hcs08_adc blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-calibration-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_calibration COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_calibration.py"
    $<TARGET_FILE:blofeld-original-hcs08-calibration-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-calibration.trace")
  set_tests_properties(original_hcs08_calibration PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)

  add_executable(blofeld-original-device-identity-test test/original_device_identity_test.cpp)
  target_link_libraries(blofeld-original-device-identity-test PRIVATE blofeld_device_identity blofeld_core)
  add_dependencies(blofeld-original-device-identity-test blofeld_hc12_spec)
  target_compile_definitions(blofeld-original-device-identity-test PRIVATE
    BLOFELD_DEFAULT_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
  add_test(NAME original_device_identity COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_device_identity.py"
    $<TARGET_FILE:blofeld-original-device-identity-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
  set_tests_properties(original_device_identity PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-original-startup-identity-test test/original_startup_identity_test.cpp)
  target_link_libraries(blofeld-original-startup-identity-test PRIVATE
    blofeld_device_identity blofeld_parallel_fixture blofeld_lcd_fixture blofeld_lcd_export)
  target_compile_definitions(blofeld-original-startup-identity-test PRIVATE
    BLOFELD_DEFAULT_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
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
  add_executable(blofeld-hcs08-instruction-reservation-test test/hcs08_instruction_reservation_test.cpp)
  add_executable(blofeld-hcs08-irq-enable-delay-test test/hcs08_irq_enable_delay_test.cpp)
  target_link_libraries(blofeld-hcs08-irq-enable-delay-test PRIVATE blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-hcs08-irq-enable-delay-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_irq_enable_delay COMMAND blofeld-hcs08-irq-enable-delay-test)
  add_executable(blofeld-hcs08-prefetch-test test/hcs08_prefetch_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-test PRIVATE blofeld_hcs08_bdc)
  target_compile_definitions(blofeld-hcs08-prefetch-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch COMMAND blofeld-hcs08-prefetch-test)
  add_executable(blofeld-hcs08-prefetch-data-test test/hcs08_prefetch_data_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-data-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-data-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_data COMMAND blofeld-hcs08-prefetch-data-test)
  add_executable(blofeld-hcs08-prefetch-branch-test test/hcs08_prefetch_branch_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-branch-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-branch-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_branch COMMAND blofeld-hcs08-prefetch-branch-test)
  add_executable(blofeld-hcs08-prefetch-jump-test test/hcs08_prefetch_jump_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-jump-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-jump-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_jump COMMAND blofeld-hcs08-prefetch-jump-test)
  add_executable(blofeld-hcs08-prefetch-scanner-test test/hcs08_prefetch_scanner_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-scanner-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-scanner-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_scanner COMMAND blofeld-hcs08-prefetch-scanner-test)
  add_executable(blofeld-hcs08-prefetch-peripheral-test test/hcs08_prefetch_peripheral_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-peripheral-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-peripheral-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_peripheral COMMAND blofeld-hcs08-prefetch-peripheral-test)
  add_executable(blofeld-hcs08-prefetch-return-test test/hcs08_prefetch_return_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-return-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-return-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_return COMMAND blofeld-hcs08-prefetch-return-test)
  add_executable(blofeld-hcs08-prefetch-store-test
    test/hcs08_prefetch_store_test.cpp test/hcs08_prefetch_move_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-store-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-store-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_stores COMMAND blofeld-hcs08-prefetch-store-test)
  add_executable(blofeld-hcs08-subroutine-cycle-test test/subroutine_test.cpp)
  target_link_libraries(blofeld-hcs08-subroutine-cycle-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-subroutine-cycle-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_subroutine COMMAND blofeld-hcs08-subroutine-cycle-test)
  add_executable(blofeld-original-hcs08-subroutine-test
    test/original_subroutine_test.cpp test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-subroutine-test PRIVATE blofeld_hcs08_spi_run_clock
    blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-subroutine-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi_subroutine COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_subroutine.py"
    $<TARGET_FILE:blofeld-original-hcs08-subroutine-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-subroutine.trace")
  set_tests_properties(original_hcs08_spi_subroutine PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-hcs08-call-cycle-test test/call_test.cpp)
  target_link_libraries(blofeld-hcs08-call-cycle-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-call-cycle-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_call COMMAND blofeld-hcs08-call-cycle-test)
  add_executable(blofeld-original-hcs08-call-test
    test/original_call_test.cpp test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-call-test PRIVATE blofeld_hcs08_spi_run_clock
    blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-call-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi_call COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_call.py"
    $<TARGET_FILE:blofeld-original-hcs08-call-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-call.trace")
  set_tests_properties(original_hcs08_spi_call PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-hcs08-rmw-cycle-test test/modify_test.cpp test/modify_inherit.cpp)
  target_link_libraries(blofeld-hcs08-rmw-cycle-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-rmw-cycle-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_modify COMMAND blofeld-hcs08-rmw-cycle-test)
  add_executable(blofeld-original-hcs08-modify-test
    test/original_modify_test.cpp test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-modify-test PRIVATE blofeld_hcs08_spi_run_clock
    blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-modify-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi_modify COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_modify.py"
    $<TARGET_FILE:blofeld-original-hcs08-modify-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-modify.trace")
  set_tests_properties(original_hcs08_spi_modify PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-hcs08-clear-cycle-test test/clear_test.cpp test/clear_inherit.cpp)
  target_link_libraries(blofeld-hcs08-clear-cycle-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-clear-cycle-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_clear COMMAND blofeld-hcs08-clear-cycle-test)
  add_executable(blofeld-original-hcs08-clear-test
    test/original_clear_test.cpp test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-clear-test PRIVATE blofeld_hcs08_spi_run_clock
    blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-clear-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi_clear COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_clear.py"
    $<TARGET_FILE:blofeld-original-hcs08-clear-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-clear.trace")
  set_tests_properties(original_hcs08_spi_clear PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-hcs08-indexed-store-test test/indexed_test.cpp test/indexed_inherit.cpp)
  target_link_libraries(blofeld-hcs08-indexed-store-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-indexed-store-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_indexed_store COMMAND blofeld-hcs08-indexed-store-test)
  add_executable(blofeld-original-hcs08-indexed-test
    test/original_receiver_test.cpp test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-indexed-test PRIVATE blofeld_hcs08_spi_run_clock
    blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-indexed-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi_indexed_store COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_spi_indexed.py"
    $<TARGET_FILE:blofeld-original-hcs08-indexed-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-indexed.trace")
  set_tests_properties(original_hcs08_spi_indexed_store PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-hcs08-prefetch-irq-test test/hcs08_prefetch_irq_test.cpp)
  target_link_libraries(blofeld-hcs08-prefetch-irq-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-prefetch-irq-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_prefetch_irq COMMAND blofeld-hcs08-prefetch-irq-test)
  add_executable(blofeld-original-hcs08-prefetch-test test/original_hcs08_prefetch_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-prefetch-test PRIVATE blofeld_hcs08_programmer_fixture
    blofeld_hcs08_run_clock blofeld_hcs08_gpio blofeld_hcs08_spi blofeld_hcs08_adc
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-prefetch-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_prefetch COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_prefetch.py"
    $<TARGET_FILE:blofeld-original-hcs08-prefetch-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-prefetch.trace")
  set_tests_properties(original_hcs08_prefetch PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_prefetch_data COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_prefetch_data.py"
    $<TARGET_FILE:blofeld-original-hcs08-prefetch-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-prefetch-data.trace")
  set_tests_properties(original_hcs08_prefetch_data PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_prefetch_branch COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_prefetch_branch.py"
    $<TARGET_FILE:blofeld-original-hcs08-prefetch-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-prefetch-branch.trace")
  set_tests_properties(original_hcs08_prefetch_branch PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_prefetch_peripheral COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_prefetch_peripheral.py"
    $<TARGET_FILE:blofeld-original-hcs08-prefetch-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-prefetch-peripheral.trace")
  set_tests_properties(original_hcs08_prefetch_peripheral PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_prefetch_return COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_prefetch_return.py"
    $<TARGET_FILE:blofeld-original-hcs08-prefetch-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-prefetch-return.trace")
  set_tests_properties(original_hcs08_prefetch_return PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_prefetch_irq COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_prefetch_irq.py"
    $<TARGET_FILE:blofeld-original-hcs08-prefetch-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-prefetch-irq.trace")
  set_tests_properties(original_hcs08_prefetch_irq PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  target_link_libraries(blofeld-hcs08-instruction-reservation-test PRIVATE blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-hcs08-instruction-reservation-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_instruction_reservation COMMAND blofeld-hcs08-instruction-reservation-test)
  add_executable(blofeld-hcs08-irq-bus-test test/hcs08_irq_bus_test.cpp)
  target_link_libraries(blofeld-hcs08-irq-bus-test PRIVATE blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-hcs08-irq-bus-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_irq_bus COMMAND blofeld-hcs08-irq-bus-test)
  add_executable(blofeld-hcs08-irq-test test/hcs08_irq_test.cpp)
  target_link_libraries(blofeld-hcs08-irq-test PRIVATE blofeld_hcs08_bdc)
  target_compile_definitions(blofeld-hcs08-irq-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_irq COMMAND blofeld-hcs08-irq-test)
  add_executable(blofeld-original-hcs08-spi-irq-test test/original_hcs08_spi_irq_test.cpp
    test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-spi-irq-test PRIVATE blofeld_hcs08_programmer_fixture
    blofeld_hcs08_adc blofeld_hcs08
    blofeld_hcs08_flash blofeld_hcs08_gpio blofeld_hcs08_icg blofeld_hcs08_spi
    blofeld_dsp_peripherals blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-spi-irq-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_spi_irq COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_spi_irq.py"
    $<TARGET_FILE:blofeld-original-hcs08-spi-irq-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-spi-irq.trace")
  set_tests_properties(original_hcs08_spi_irq PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_irq_bus COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_irq_bus.py"
    $<TARGET_FILE:blofeld-original-hcs08-spi-irq-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-irq-bus.trace")
  set_tests_properties(original_hcs08_irq_bus PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)

  add_executable(blofeld-hcs08-flash-test test/hcs08_flash_test.cpp)
  target_link_libraries(blofeld-hcs08-flash-test PRIVATE blofeld_hcs08_flash)
  add_test(NAME hcs08_flash COMMAND blofeld-hcs08-flash-test)
  add_executable(blofeld-hcs08-flash-recovery-test test/hcs08_flash_recovery_test.cpp)
  target_link_libraries(blofeld-hcs08-flash-recovery-test PRIVATE blofeld_hcs08_flash)
  add_test(NAME hcs08_flash_recovery COMMAND blofeld-hcs08-flash-recovery-test)
  add_executable(blofeld-original-hcs08-icg-test test/original_hcs08_icg_test.cpp)
  target_link_libraries(blofeld-original-hcs08-icg-test PRIVATE
    blofeld_hcs08_programmer_fixture blofeld_hcs08_gpio blofeld_private_files blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-original-hcs08-icg-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_icg COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_icg.py"
    $<TARGET_FILE:blofeld-original-hcs08-icg-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-icg.trace")
  set_tests_properties(original_hcs08_icg PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_periods COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_periods.py"
    $<TARGET_FILE:blofeld-original-hcs08-icg-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-periods.trace")
  set_tests_properties(original_hcs08_periods PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-hcs08-flash-clock-test test/hcs08_flash_clock_test.cpp)
  target_link_libraries(blofeld-hcs08-flash-clock-test PRIVATE blofeld_hcs08_flash)
  add_test(NAME hcs08_flash_clock COMMAND blofeld-hcs08-flash-clock-test)
  add_executable(blofeld-hcs08-icg-test test/hcs08_icg_test.cpp)
  target_link_libraries(blofeld-hcs08-icg-test PRIVATE blofeld_hcs08_icg)
  add_test(NAME hcs08_icg COMMAND blofeld-hcs08-icg-test)
  add_executable(blofeld-hcs08-icg-events-test test/hcs08_icg_events_test.cpp)
  target_link_libraries(blofeld-hcs08-icg-events-test PRIVATE blofeld_hcs08_icg)
  add_test(NAME hcs08_icg_events COMMAND blofeld-hcs08-icg-events-test)
  add_executable(blofeld-hcs08-icg-clocks-test test/hcs08_icg_clocks_test.cpp)
  target_link_libraries(blofeld-hcs08-icg-clocks-test PRIVATE blofeld_hcs08_icg_clocks)
  add_test(NAME hcs08_icg_clock_routes COMMAND blofeld-hcs08-icg-clocks-test)
  add_executable(blofeld-hcs08-icg-status-test test/hcs08_icg_status_test.cpp)
  target_link_libraries(blofeld-hcs08-icg-status-test PRIVATE blofeld_hcs08_icg_status)
  add_test(NAME hcs08_icg_status COMMAND blofeld-hcs08-icg-status-test)
  add_executable(blofeld-hcs08-icg-history-test test/hcs08_icg_history_test.cpp)
  target_link_libraries(blofeld-hcs08-icg-history-test PRIVATE blofeld_hcs08_icg_history)
  add_test(NAME hcs08_icg_history COMMAND blofeld-hcs08-icg-history-test)
  add_executable(blofeld-hcs08-known-interrupts-test test/hcs08_known_interrupts_test.cpp)
  target_link_libraries(blofeld-hcs08-known-interrupts-test PRIVATE
    blofeld_hcs08 blofeld_hcs08_icg_history)
  target_compile_definitions(blofeld-hcs08-known-interrupts-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_known_interrupts COMMAND blofeld-hcs08-known-interrupts-test)
  add_executable(blofeld-original-hcs08-icg-history-test test/original_history_startup_test.cpp
    test/history_startup_memory.cpp test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-icg-history-test PRIVATE
    blofeld_hcs08 blofeld_hcs08_icg_history blofeld_hcs08_adc blofeld_hcs08_gpio
    blofeld_hcs08_flash blofeld_hcs08_spi blofeld_hcs08_tpm)
  target_compile_definitions(blofeld-original-hcs08-icg-history-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_icg_history COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_icg_history.py"
    $<TARGET_FILE:blofeld-original-hcs08-icg-history-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-icg-history.trace")
  set_tests_properties(original_hcs08_icg_history PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-original-hcs08-clock-prefix-test test/original_clock_prefix_test.cpp
    test/clock_prefix_memory.cpp test/history_startup_memory.cpp test/hcs08_clock_startup_fixture.cpp)
  target_link_libraries(blofeld-original-hcs08-clock-prefix-test PRIVATE
    blofeld_hcs08_run_clock blofeld_hcs08_cop blofeld_hcs08_icg_history
    blofeld_hcs08_adc blofeld_hcs08_gpio blofeld_hcs08_flash blofeld_hcs08_spi blofeld_hcs08_tpm)
  target_compile_definitions(blofeld-original-hcs08-clock-prefix-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_clock_prefix COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_clock_prefix.py"
    $<TARGET_FILE:blofeld-original-hcs08-clock-prefix-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-clock-prefix.trace")
  set_tests_properties(original_hcs08_clock_prefix PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 960)
  add_executable(blofeld-hcs08-bit-known-test test/hcs08_bit_known_test.cpp)
  target_link_libraries(blofeld-hcs08-bit-known-test PRIVATE blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-hcs08-bit-known-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_bit_known COMMAND blofeld-hcs08-bit-known-test)
  set_tests_properties(hcs08_bit_known PROPERTIES TIMEOUT 180)
  add_executable(blofeld-hcs08-gpio-known-test test/hcs08_gpio_known_test.cpp)
  target_link_libraries(blofeld-hcs08-gpio-known-test PRIVATE blofeld_hcs08_gpio)
  add_test(NAME hcs08_gpio_known COMMAND blofeld-hcs08-gpio-known-test)
  add_executable(blofeld-hcs08-known-load-test test/hcs08_known_load_test.cpp)
  target_link_libraries(blofeld-hcs08-known-load-test PRIVATE blofeld_hcs08 blofeld_hcs08_cop)
  target_compile_definitions(blofeld-hcs08-known-load-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_known_load COMMAND blofeld-hcs08-known-load-test)
  add_executable(blofeld-hcs08-captured-known-test test/hcs08_captured_known_test.cpp)
  target_link_libraries(blofeld-hcs08-captured-known-test PRIVATE
    blofeld_hcs08 blofeld_hcs08_cop blofeld_hcs08_icg_status)
  target_compile_definitions(blofeld-hcs08-captured-known-test PRIVATE
    BLOFELD_HCS08_SLA=\"${CMAKE_BINARY_DIR}/share/HCS08.sla\")
  add_test(NAME hcs08_captured_known COMMAND blofeld-hcs08-captured-known-test)
  add_executable(blofeld-hcs08-tpm-test test/hcs08_tpm_test.cpp)
  target_link_libraries(blofeld-hcs08-tpm-test PRIVATE blofeld_hcs08_tpm)
  add_test(NAME hcs08_tpm COMMAND blofeld-hcs08-tpm-test)
  add_executable(blofeld-hcs08-cop-test test/hcs08_cop_test.cpp)
  target_link_libraries(blofeld-hcs08-cop-test PRIVATE blofeld_hcs08_cop)
  add_test(NAME hcs08_cop COMMAND blofeld-hcs08-cop-test)
  add_executable(blofeld-original-hcs08-timer-wait-test test/original_hcs08_timer_wait_test.cpp)
  target_link_libraries(blofeld-original-hcs08-timer-wait-test PRIVATE blofeld_hcs08_programmer_fixture
    blofeld_hcs08_tpm blofeld_hcs08_cop blofeld_hcs08_spi blofeld_hcs08_run_clock blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-timer-wait-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_timer_wait COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_timer_wait.py"
    $<TARGET_FILE:blofeld-original-hcs08-timer-wait-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-timer-wait.trace")
  set_tests_properties(original_hcs08_timer_wait PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_executable(blofeld-hcs08-icg-native-test test/hcs08_icg_native_test.cpp)
  target_link_libraries(blofeld-hcs08-icg-native-test PRIVATE blofeld_hcs08_icg blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-icg-native-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_icg_native COMMAND blofeld-hcs08-icg-native-test)
  add_executable(blofeld-hcs08-peripheral-reset-test test/hcs08_peripheral_reset_test.cpp)
  target_link_libraries(blofeld-hcs08-peripheral-reset-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-hcs08-peripheral-reset-test PRIVATE blofeld_hcs08
    blofeld_hcs08_flash)
  add_test(NAME hcs08_peripheral_reset COMMAND blofeld-hcs08-peripheral-reset-test)
  add_executable(blofeld-original-hcs08-flash-test test/original_hcs08_flash_test.cpp)
  target_link_libraries(blofeld-original-hcs08-flash-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-hcs08-flash-test PRIVATE blofeld_hcs08
    blofeld_hcs08_flash blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-flash-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_flash COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_flash.py"
    $<TARGET_FILE:blofeld-original-hcs08-flash-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-flash.trace")
  set_tests_properties(original_hcs08_flash PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 270)
  add_executable(blofeld-hcs08-bdc-test test/hcs08_bdc_test.cpp)
  target_link_libraries(blofeld-hcs08-bdc-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-hcs08-bdc-test PRIVATE blofeld_hcs08_bdc)
  target_compile_definitions(blofeld-hcs08-bdc-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_bdc COMMAND blofeld-hcs08-bdc-test)
  add_executable(blofeld-original-hcs08-bdc-test test/original_hcs08_bdc_test.cpp)
  target_link_libraries(blofeld-original-hcs08-bdc-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-hcs08-bdc-test PRIVATE blofeld_hcs08_bdc
    blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-bdc-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_bdc COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_bdc.py"
    $<TARGET_FILE:blofeld-original-hcs08-bdc-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-bdc.trace")
  set_tests_properties(original_hcs08_bdc PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 270)
  add_executable(blofeld-original-hcs08-reset-entry-test test/original_hcs08_reset_entry_test.cpp)
  target_link_libraries(blofeld-original-hcs08-reset-entry-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-hcs08-reset-entry-test PRIVATE blofeld_hcs08_run_clock
    blofeld_private_files)
  target_compile_definitions(blofeld-original-hcs08-reset-entry-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_hcs08_reset_entry COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_reset_entry.py"
    $<TARGET_FILE:blofeld-original-hcs08-reset-entry-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-reset-entry.trace")
  set_tests_properties(original_hcs08_reset_entry PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_test(NAME original_hcs08_cold_programmer COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_hcs08_cold_programmer.py"
    $<TARGET_FILE:blofeld-original-hcs08-reset-entry-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-hcs08-cold-programmer.trace")
  set_tests_properties(original_hcs08_cold_programmer PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-hcs08-bdc-wire-test test/hcs08_bdc_wire_test.cpp)
  target_link_libraries(blofeld-hcs08-bdc-wire-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-hcs08-bdc-wire-test PRIVATE blofeld_hcs08_bdc_wire)
  target_compile_definitions(blofeld-hcs08-bdc-wire-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_bdc_wire COMMAND blofeld-hcs08-bdc-wire-test)
  add_executable(blofeld-original-keyboard-bdc-wire-test test/original_keyboard_bdc_wire_test.cpp)
  target_link_libraries(blofeld-original-keyboard-bdc-wire-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-keyboard-bdc-wire-test PRIVATE blofeld_core
    blofeld_hcs08_bdc_wire)
  target_compile_definitions(blofeld-original-keyboard-bdc-wire-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_keyboard_bdc_wire COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_bdc_wire.py"
    $<TARGET_FILE:blofeld-original-keyboard-bdc-wire-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-bdc-wire.trace")
  set_tests_properties(original_keyboard_bdc_wire PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 360)
  add_executable(blofeld-original-keyboard-reset-sync-test test/original_keyboard_reset_sync_test.cpp)
  target_link_libraries(blofeld-original-keyboard-reset-sync-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-keyboard-reset-sync-test PRIVATE blofeld_core blofeld_hcs08_bdc_wire)
  target_compile_definitions(blofeld-original-keyboard-reset-sync-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_keyboard_reset_sync COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_reset_sync.py"
    $<TARGET_FILE:blofeld-original-keyboard-reset-sync-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-reset-sync.trace")
  set_tests_properties(original_keyboard_reset_sync PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-hcs08-run-clock-test test/hcs08_run_clock_test.cpp)
  target_link_libraries(blofeld-hcs08-run-clock-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-hcs08-run-clock-test PRIVATE blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-hcs08-run-clock-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_run_clock COMMAND blofeld-hcs08-run-clock-test)
  add_executable(blofeld-hcs08-instruction-timing-test test/hcs08_instruction_timing_test.cpp)
  target_link_libraries(blofeld-hcs08-instruction-timing-test PRIVATE blofeld_hcs08_run_clock)
  add_test(NAME hcs08_instruction_timing COMMAND blofeld-hcs08-instruction-timing-test)
  add_executable(blofeld-hcs08-divide-test test/hcs08_divide_test.cpp)
  target_link_libraries(blofeld-hcs08-divide-test PRIVATE blofeld_hcs08_run_clock)
  add_test(NAME hcs08_divide COMMAND blofeld-hcs08-divide-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_executable(blofeld-original-keyboard-target-test test/original_keyboard_target_test.cpp
    test/keyboard_target_fixture.cpp)
  target_link_libraries(blofeld-original-keyboard-target-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-keyboard-target-test PRIVATE blofeld_core
    blofeld_hcs08_bdc_wire blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-original-keyboard-target-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_keyboard_target COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_target.py"
    $<TARGET_FILE:blofeld-original-keyboard-target-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-target.trace")
  set_tests_properties(original_keyboard_target PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_executable(blofeld-hcs08-register-knowledge-test test/hcs08_register_knowledge_test.cpp)
  target_link_libraries(blofeld-hcs08-register-knowledge-test PRIVATE blofeld_hcs08_bdc)
  target_compile_definitions(blofeld-hcs08-register-knowledge-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_register_knowledge COMMAND blofeld-hcs08-register-knowledge-test)
  add_executable(blofeld-hcs08-reset-test test/hcs08_reset_test.cpp)
  target_link_libraries(blofeld-hcs08-reset-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-hcs08-reset-test PRIVATE blofeld_hcs08_reset)
  target_compile_definitions(blofeld-hcs08-reset-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_reset COMMAND blofeld-hcs08-reset-test)
  add_executable(blofeld-hcs08-reset-security-test test/hcs08_reset_security_test.cpp)
  target_link_libraries(blofeld-hcs08-reset-security-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-hcs08-reset-security-test PRIVATE blofeld_hcs08_bdc)
  target_compile_definitions(blofeld-hcs08-reset-security-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_reset_security COMMAND blofeld-hcs08-reset-security-test)
  add_executable(blofeld-hcs08-background-reset-test test/hcs08_background_reset_test.cpp)
  target_link_libraries(blofeld-hcs08-background-reset-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-hcs08-background-reset-test PRIVATE blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-hcs08-background-reset-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_background_reset COMMAND blofeld-hcs08-background-reset-test)
  add_executable(blofeld-original-keyboard-reset-test test/original_keyboard_reset_test.cpp
    test/keyboard_reset_fixture.cpp)
  target_link_libraries(blofeld-original-keyboard-reset-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-keyboard-reset-test PRIVATE blofeld_core
    blofeld_hcs08_reset blofeld_hcs08_bdc_wire blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-original-keyboard-reset-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_keyboard_reset COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_reset.py"
    $<TARGET_FILE:blofeld-original-keyboard-reset-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-reset.trace")
  set_tests_properties(original_keyboard_reset PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 360)

  add_executable(blofeld-audio-runtime-test test/audio_runtime_test.cpp
    test/audio_runtime_failure.cpp test/audio_runtime_board.cpp test/audio_runtime_underrun.cpp)
  target_link_libraries(blofeld-audio-runtime-test PRIVATE blofeld_audio_runtime blofeld_reference_board blofeld_pcm_reference)
  add_test(NAME audio_runtime COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/audio_runtime.py"
    $<TARGET_FILE:blofeld-audio-runtime-test> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/audio-runtime")
  set(blofeld_audio_test_timeout 120)
  if(BLOFELD_SANITIZER STREQUAL "thread")
    set(blofeld_audio_test_timeout 300)
  endif()
  set_tests_properties(audio_runtime PROPERTIES SKIP_RETURN_CODE 77
    TIMEOUT ${blofeld_audio_test_timeout} ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_executable(blofeld-pcm-queue-test test/pcm_queue_test.cpp)
  target_link_libraries(blofeld-pcm-queue-test PRIVATE blofeld_pcm_queue Threads::Threads blofeld_pcm_reference)
  add_test(NAME pcm_queue COMMAND blofeld-pcm-queue-test)
  set_tests_properties(pcm_queue PROPERTIES TIMEOUT 40)

  add_executable(blofeld-hc12-ccr-test test/hc12_ccr_test.cpp)
  target_link_libraries(blofeld-hc12-ccr-test PRIVATE blofeld_core)
  add_test(NAME hc12_ccr COMMAND blofeld-hc12-ccr-test)
  add_executable(blofeld-pcode-storage-test test/pcode_storage_test.cpp)
  target_link_libraries(blofeld-pcode-storage-test PRIVATE blofeld_sleigh blofeld_firmware)
  add_test(NAME pcode_storage COMMAND blofeld-pcode-storage-test)
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
  add_test(NAME reference_usb_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_usb_cli.py"
    $<TARGET_FILE:blofeld-headless> "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-usb")
  set_tests_properties(reference_usb_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
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
  add_executable(blofeld-sample-wave-test test/sample_wave_test.cpp)
  target_link_libraries(blofeld-sample-wave-test PRIVATE blofeld_sample_library)
  add_test(NAME sample_wave COMMAND blofeld-sample-wave-test)
  add_test(NAME sample_wave_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_wave_cli.py" $<TARGET_FILE:blofeld-headless>)
  add_executable(blofeld-sample-library-test test/sample_library_test.cpp)
  target_link_libraries(blofeld-sample-library-test PRIVATE blofeld_sample_library)
  add_test(NAME sample_library COMMAND blofeld-sample-library-test)
  add_test(NAME sample_library_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_library_cli.py" $<TARGET_FILE:blofeld-headless>)
  add_executable(blofeld-midi-timed-test test/midi_timed_test.cpp)
  target_link_libraries(blofeld-midi-timed-test PRIVATE blofeld_midi_timed)
  add_test(NAME midi_timed COMMAND blofeld-midi-timed-test)
  add_test(NAME reference_dsp_host_midi_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_dsp_host_midi_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-original-midi-rom>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-dsp-host-midi")
  # Four native message runs, including a coincident independent timer input.
  set_tests_properties(reference_dsp_host_midi_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 360)
  add_executable(blofeld-sequence-codec-test test/sequence_codec_test.cpp)
  target_link_libraries(blofeld-sequence-codec-test PRIVATE blofeld_host_midi_port)
  add_test(NAME sequence_codec COMMAND blofeld-sequence-codec-test)
  add_executable(blofeld-host-midi-port-test test/host_midi_port_test.cpp)
  target_link_libraries(blofeld-host-midi-port-test PRIVATE blofeld_host_midi_port)
  add_test(NAME host_midi_port COMMAND blofeld-host-midi-port-test)
  set_tests_properties(host_midi_port PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 30)
  add_test(NAME host_midi_port_discovery COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/host_midi_port_discovery.py"
    $<TARGET_FILE:blofeld-host-midi-port-test>
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/host-midi-port-discovery.json")
  set_tests_properties(host_midi_port_discovery PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 30)
  add_executable(blofeld-host-midi-test test/host_midi_test.cpp)
  target_link_libraries(blofeld-host-midi-test PRIVATE blofeld_host_midi blofeld_firmware)
  add_test(NAME host_midi COMMAND blofeld-host-midi-test
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/host-midi")
  add_test(NAME reference_midi_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_midi_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-midi-cli")
  set_tests_properties(reference_midi_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
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

  add_executable(blofeld-peripheral-calendar-test test/dsp_peripheral_calendar_test.cpp)
  target_link_libraries(blofeld-peripheral-calendar-test PRIVATE blofeld_dsp_peripheral_calendar blofeld_nand_read_timing)
  add_test(NAME dsp_peripheral_calendar COMMAND blofeld-peripheral-calendar-test)

  add_executable(blofeld-vm-info-test test/dsp_vm_info_test.cpp)
  target_include_directories(blofeld-vm-info-test SYSTEM PRIVATE "${BLOFELD_DSP_COPY}/source/asmjit/src")
  target_link_libraries(blofeld-vm-info-test PRIVATE blofeld_quality asmjit Threads::Threads ${CMAKE_DL_LIBS} rt)
  add_test(NAME dsp_vm_info COMMAND blofeld-vm-info-test)
  set_tests_properties(dsp_vm_info PROPERTIES TIMEOUT 120)

  add_test(NAME reference_dsp_inputs_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_dsp_inputs_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-dsp-inputs")
  set_tests_properties(reference_dsp_inputs_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_test(NAME reference_esai_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_esai_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-esai")
  set_tests_properties(reference_esai_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_test(NAME reference_pcm_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_pcm_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-pcm")
  set_tests_properties(reference_pcm_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_test(NAME reference_wave_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_wave_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-wave")
  set_tests_properties(reference_wave_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)

  add_test(NAME reference_original_midi_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_original_midi_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-original-midi-rom>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-original-midi")
  # Two baseline runs plus two three-branch replays;90 seconds per native branch.
  set_tests_properties(reference_original_midi_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 800)
  add_executable(blofeld-reference-rom-fixture test/reference_rom.cpp)
  target_link_libraries(blofeld-reference-rom-fixture PRIVATE blofeld_firmware)
  add_test(NAME reference_shi_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_shi_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-shi-cli" signals)
  set_tests_properties(reference_shi_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_test(NAME reference_echo_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_shi_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-echo-cli" echo)
  set_tests_properties(reference_echo_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-startup-probe EXCLUDE_FROM_ALL test/instrument/startup_probe.cpp)
  target_link_libraries(blofeld-startup-probe PRIVATE blofeld_reference_board blofeld_device_identity)

  add_executable(blofeld-mcu-dsp-execution-test test/board/mcu_dsp_execution_test.cpp)
  target_include_directories(blofeld-mcu-dsp-execution-test PRIVATE test
    "${BLOFELD_CPU12_ROOT}/test/support" "${BLOFELD_DSP56300_ROOT}/test")
  target_link_libraries(blofeld-mcu-dsp-execution-test PRIVATE blofeld_mcu_dsp_runtime)
  add_test(NAME mcu_dsp_execution COMMAND blofeld-mcu-dsp-execution-test)

  add_executable(blofeld-mcu-native-execution-test test/mcu_native_execution_test.cpp)
  target_include_directories(blofeld-mcu-native-execution-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  target_link_libraries(blofeld-mcu-native-execution-test PRIVATE blofeld_core)
  add_test(NAME mcu_native_execution COMMAND blofeld-mcu-native-execution-test)

  add_executable(blofeld-mcu-power-on-test test/mcu_power_on_test.cpp)
  target_link_libraries(blofeld-mcu-power-on-test PRIVATE blofeld_core)
  add_test(NAME mcu_power_on COMMAND blofeld-mcu-power-on-test)

  add_executable(blofeld-hc12-cycle-test test/hc12_cycle_test.cpp)
  target_link_libraries(blofeld-hc12-cycle-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME hc12_cycles COMMAND blofeld-hc12-cycle-test)
  add_executable(blofeld-hc12-masked-gpio-test test/hc12_masked_gpio_test.cpp)
  target_link_libraries(blofeld-hc12-masked-gpio-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME hc12_masked_gpio COMMAND blofeld-hc12-masked-gpio-test)
  set_tests_properties(hc12_masked_gpio PROPERTIES TIMEOUT 120)
  add_executable(blofeld-hc12-cycle-extended-test test/hc12_cycle_extended_test.cpp)
  target_link_libraries(blofeld-hc12-cycle-extended-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME hc12_cycle_extended COMMAND blofeld-hc12-cycle-extended-test)
  add_executable(blofeld-hc12-compare-stack-test test/hc12_compare_stack_test.cpp)
  target_link_libraries(blofeld-hc12-compare-stack-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME hc12_compare_stack COMMAND blofeld-hc12-compare-stack-test)
  add_executable(blofeld-hc12-indexed-test test/hc12_indexed_test.cpp)
  target_link_libraries(blofeld-hc12-indexed-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME hc12_indexed COMMAND blofeld-hc12-indexed-test)
  add_executable(blofeld-hc12-minmax-test test/hc12_minmax_test.cpp
    test/hc12_minmax_fixture.cpp test/hc12_minmax_timing.cpp test/hc12_minmax_machine.cpp)
  target_link_libraries(blofeld-hc12-minmax-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME hc12_minmax COMMAND blofeld-hc12-minmax-test)
  set_tests_properties(hc12_minmax PROPERTIES TIMEOUT 180)
  set_tests_properties(hc12_indexed PROPERTIES TIMEOUT 120)
  add_executable(blofeld-hc12-flash-cycles-test test/hc12_flash_cycles_test.cpp)
  target_link_libraries(blofeld-hc12-flash-cycles-test PRIVATE blofeld_core)
  add_test(NAME hc12_flash_cycles COMMAND blofeld-hc12-flash-cycles-test)
  add_executable(blofeld-mcu-cycle-test test/mcu_cycle_test.cpp)
  target_link_libraries(blofeld-mcu-cycle-test PRIVATE blofeld_core blofeld_hc12_cycles blofeld_mcu_midi)
  add_test(NAME mcu_cycles COMMAND blofeld-mcu-cycle-test)
  add_executable(blofeld-mcu-run-clock-test test/mcu_run_clock_test.cpp)
  target_link_libraries(blofeld-mcu-run-clock-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME mcu_run_clock COMMAND blofeld-mcu-run-clock-test)
  add_executable(blofeld-mcu-wait-clock-test test/mcu_wait_clock_test.cpp)
  target_link_libraries(blofeld-mcu-wait-clock-test PRIVATE blofeld_core)
  add_test(NAME mcu_wait_clock COMMAND blofeld-mcu-wait-clock-test)
  add_executable(blofeld-original-cycle-boot-test test/original_cycle_boot_test.cpp)
  target_link_libraries(blofeld-original-cycle-boot-test PRIVATE blofeld_core blofeld_hc12_cycles blofeld_mcu_midi)
  add_test(NAME original_cycle_boot COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-cycle-boot-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-cycle-boot.trace")
  set_tests_properties(original_cycle_boot PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-cycle-application-test test/original_cycle_application_test.cpp)
  target_link_libraries(blofeld-original-cycle-application-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME original_cycle_application COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-cycle-application-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-cycle-application.trace")
  set_tests_properties(original_cycle_application PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-run-clock-test test/original_run_clock_test.cpp)
  target_link_libraries(blofeld-original-run-clock-test PRIVATE blofeld_core blofeld_hc12_cycles blofeld_lcd_fixture)
  add_test(NAME original_run_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-run-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-run-clock.trace")
  set_tests_properties(original_run_clock PROPERTIES SKIP_RETURN_CODE 77)
  set(BLOFELD_CLOCKED_STARTUP_REFERENCE
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    CACHE FILEPATH "Pinned original ZIP or identical complete source MID for clocked startup")
  add_executable(blofeld-original-owned-clock-test
    test/original_owned_clock_test.cpp test/original_owned_clock_fixture.cpp test/original_owned_midi.cpp)
  target_link_libraries(blofeld-original-owned-clock-test PRIVATE blofeld_core blofeld_lcd_fixture
    blofeld_mcu_midi)
  add_test(NAME original_owned_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_owned_clock.py"
    $<TARGET_FILE:blofeld-original-owned-clock-test>
    "${BLOFELD_CLOCKED_STARTUP_REFERENCE}"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-owned-clock.trace")
  set_tests_properties(original_owned_clock PROPERTIES SKIP_RETURN_CODE 77)
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
  add_executable(blofeld-midi-serial-test test/midi_serial_test.cpp)
  target_link_libraries(blofeld-midi-serial-test PRIVATE blofeld_midi_serial)
  add_test(NAME midi_serial COMMAND blofeld-midi-serial-test)
  add_executable(blofeld-midi-serial-receiver-test test/midi_serial_receiver_test.cpp)
  target_link_libraries(blofeld-midi-serial-receiver-test PRIVATE blofeld_midi_serial blofeld_mcu)
  add_test(NAME midi_serial_receiver COMMAND blofeld-midi-serial-receiver-test)
  add_executable(blofeld-mcu-midi-output-test test/mcu_midi_output_test.cpp)
  target_link_libraries(blofeld-mcu-midi-output-test PRIVATE blofeld_mcu_midi_output)
  add_test(NAME mcu_midi_output COMMAND blofeld-mcu-midi-output-test)

  add_executable(blofeld-mcu-midi-test test/mcu_midi_test.cpp)
  target_link_libraries(blofeld-mcu-midi-test PRIVATE blofeld_mcu_midi)
  add_test(NAME mcu_midi COMMAND blofeld-mcu-midi-test)
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
  add_executable(blofeld-nand-wide-timebase-test test/nand_wide_timebase_test.cpp)
  target_link_libraries(blofeld-nand-wide-timebase-test PRIVATE blofeld_nand_write_medium
    blofeld_nand_read_timing blofeld_nand_write_timing blofeld_nand_read_setup)
  add_test(NAME nand_wide_timebase COMMAND blofeld-nand-wide-timebase-test)
  add_executable(blofeld-nand-functional-timing-test test/nand_functional_timing_test.cpp)
  target_link_libraries(blofeld-nand-functional-timing-test PRIVATE blofeld_nand_write_medium
    blofeld_nand_read_timing blofeld_nand_write_timing blofeld_nand_read_setup)
  add_test(NAME nand_functional_timing COMMAND blofeld-nand-functional-timing-test)
  add_executable(blofeld-nand-ce-window-test test/nand_ce_window_test.cpp
    test/nand_ce_window_boundaries.cpp test/nand_ce_window_guards.cpp test/nand_ce_window_histories.cpp)
  target_link_libraries(blofeld-nand-ce-window-test PRIVATE
    blofeld_nand_read_timing blofeld_nand_write_timing blofeld_nand_read_setup)
  foreach(scope boundaries guards histories)
    add_test(NAME nand_ce_window_${scope} COMMAND blofeld-nand-ce-window-test ${scope})
    set_tests_properties(nand_ce_window_${scope} PROPERTIES TIMEOUT 30)
  endforeach()
  add_executable(blofeld-nand-read-transition-test test/nand_read_transition_test.cpp)
  target_link_libraries(blofeld-nand-read-transition-test PRIVATE blofeld_nand_read_timing)
  add_test(NAME nand_read_transition COMMAND blofeld-nand-read-transition-test)
  set_tests_properties(nand_read_transition PROPERTIES TIMEOUT 30)
  add_executable(blofeld-nand-standby-test test/nand_standby_test.cpp)
  target_link_libraries(blofeld-nand-standby-test PRIVATE
    blofeld_nand_read_timing blofeld_nand_write_timing blofeld_nand_read_setup)
  add_test(NAME nand_standby COMMAND blofeld-nand-standby-test)
  set_tests_properties(nand_standby PROPERTIES TIMEOUT 30)
  add_executable(blofeld-nand-read-exit-test test/nand_read_exit_test.cpp)
  target_link_libraries(blofeld-nand-read-exit-test PRIVATE
    blofeld_nand_read_timing blofeld_nand_write_timing blofeld_nand_read_setup)
  add_test(NAME nand_read_exit COMMAND blofeld-nand-read-exit-test)
  set_tests_properties(nand_read_exit PROPERTIES TIMEOUT 30)
  add_executable(blofeld-nand-initial-observation-test test/nand_initial_observation_test.cpp)
  target_link_libraries(blofeld-nand-initial-observation-test PRIVATE blofeld_nand_read_timing)
  add_test(NAME nand_initial_observation COMMAND blofeld-nand-initial-observation-test)
  set_tests_properties(nand_initial_observation PROPERTIES TIMEOUT 30)
  add_executable(blofeld-nand-read-timing-test test/nand_read_timing_test.cpp)
  target_link_libraries(blofeld-nand-read-timing-test PRIVATE blofeld_nand_read_timing)
  add_test(NAME nand_read_timing COMMAND blofeld-nand-read-timing-test)

  add_executable(blofeld-nand-write-timing-test test/nand_write_timing_test.cpp)
  target_link_libraries(blofeld-nand-write-timing-test PRIVATE blofeld_nand_write_timing)
  add_test(NAME nand_write_timing COMMAND blofeld-nand-write-timing-test)

  add_executable(blofeld-nand-read-setup-test test/nand_read_setup_test.cpp)
  target_link_libraries(blofeld-nand-read-setup-test PRIVATE blofeld_nand_read_setup)
  add_test(NAME nand_read_setup COMMAND blofeld-nand-read-setup-test)

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
  add_test(NAME flash_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/flash_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip")
  add_test(NAME device_state_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/device_state_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip")
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

  add_test(NAME reference_flash_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_flash_cli.py" $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip")
  set_tests_properties(reference_flash_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
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
  add_library(blofeld_application_fixture STATIC test/application_fixture.cpp test/application_midi_fixture.cpp
    test/application_update_fixture.cpp)
  target_link_libraries(blofeld_application_fixture PUBLIC blofeld_lcd_fixture blofeld_parallel_fixture blofeld_mcu_midi)
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
  add_executable(blofeld-original-mcu-scan-test test/original_mcu_scan_test.cpp)
  target_link_libraries(blofeld-original-mcu-scan-test PRIVATE blofeld_core)
  add_test(NAME original_mcu_scan COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_mcu_scan.py"
    $<TARGET_FILE:blofeld-original-mcu-scan-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-mcu-scan.trace")
  set_tests_properties(original_mcu_scan PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-usb-ique-control-test test/usb_ique_control_test.cpp)
  target_link_libraries(blofeld-usb-ique-control-test PRIVATE blofeld_mcu)
  add_test(NAME usb_ique_control COMMAND blofeld-usb-ique-control-test)
  add_executable(blofeld-usb-endpoint-reset-test test/usb_endpoint_reset_test.cpp test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-usb-endpoint-reset-test PRIVATE blofeld_core)
  add_test(NAME usb_endpoint_reset COMMAND blofeld-usb-endpoint-reset-test)
  add_executable(blofeld-original-usb-reset-test test/original_usb_reset_test.cpp)
  target_include_directories(blofeld-original-usb-reset-test PRIVATE src)
  target_link_libraries(blofeld-original-usb-reset-test PRIVATE blofeld_application_fixture blofeld_usbip_tcp)
  add_test(NAME original_usb_reset COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_usb_reset.py"
    $<TARGET_FILE:blofeld-original-usb-reset-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-usb-reset.trace")
  set_tests_properties(original_usb_reset PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-usb-bulk-fields-test test/usb_bulk_fields_test.cpp)
  target_link_libraries(blofeld-usb-bulk-fields-test PRIVATE blofeld_mcu)
  add_test(NAME usb_bulk_fields COMMAND blofeld-usb-bulk-fields-test)
  add_executable(blofeld-original-usb-bulk-test test/original_usb_bulk_test.cpp)
  target_include_directories(blofeld-original-usb-bulk-test PRIVATE src)
  target_link_libraries(blofeld-original-usb-bulk-test PRIVATE blofeld_application_fixture blofeld_usbip_tcp)
  add_test(NAME original_usb_bulk COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_usb_bulk.py"
    $<TARGET_FILE:blofeld-original-usb-bulk-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-usb-bulk.trace")
  set_tests_properties(original_usb_bulk PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-original-host-midi-test test/original_host_midi_test.cpp)
  target_link_libraries(blofeld-original-host-midi-test PRIVATE
    blofeld_application_fixture blofeld_mcu_midi blofeld_host_midi)
  add_test(NAME original_host_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_host_midi.py"
    $<TARGET_FILE:blofeld-original-host-midi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-host-midi.trace")
  set_tests_properties(original_host_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-application-test test/original_application_test.cpp)
  target_link_libraries(blofeld-original-application-test PRIVATE blofeld_application_fixture)
  add_test(NAME original_application COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-application-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-application.trace")
  set_tests_properties(original_application PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-identity-main-test test/original_identity_main_test.cpp)
  target_link_libraries(blofeld-original-identity-main-test PRIVATE blofeld_application_fixture
    blofeld_device_identity blofeld_lcd_export)
  target_compile_definitions(blofeld-original-identity-main-test PRIVATE
    BLOFELD_DEFAULT_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
  foreach(identity_mode baseline factory)
    add_test(NAME original_identity_main_${identity_mode} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/original_identity_main.py"
      $<TARGET_FILE:blofeld-original-identity-main-test>
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-identity-main-${identity_mode}.trace"
      "${identity_mode}")
    set_tests_properties(original_identity_main_${identity_mode} PROPERTIES
      SKIP_RETURN_CODE 77 TIMEOUT 240)
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

  add_executable(blofeld-original-keyboard-receiver-test test/original_keyboard_receiver_test.cpp)
  target_link_libraries(blofeld-original-keyboard-receiver-test PRIVATE blofeld_core)
  add_test(NAME original_keyboard_receiver COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_receiver.py"
    $<TARGET_FILE:blofeld-original-keyboard-receiver-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-receiver.trace")
  set_tests_properties(original_keyboard_receiver PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)

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

  add_test(NAME original_shi_routes COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_shi_routes.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-shi-routes")
  set_tests_properties(original_shi_routes PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_test(NAME original_reference_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_reference_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/build/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-reference-cli.trace")
  set_tests_properties(original_reference_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)

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

  add_executable(blofeld-clocked-midi-test test/mcu_clocked_midi_test.cpp)
  target_link_libraries(blofeld-clocked-midi-test PRIVATE blofeld_mcu_midi)
  add_test(NAME mcu_clocked_midi COMMAND blofeld-clocked-midi-test)
  add_executable(blofeld-timer-test test/mcu_timer_test.cpp)
  target_link_libraries(blofeld-timer-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_timer COMMAND blofeld-timer-test)
  add_executable(blofeld-mcu-timer-span-test test/mcu_timer_span_test.cpp)
  target_link_libraries(blofeld-mcu-timer-span-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_timer_span COMMAND blofeld-mcu-timer-span-test)
  add_executable(blofeld-pacnt-storage-test test/mcu_pacnt_storage_test.cpp)
  target_link_libraries(blofeld-pacnt-storage-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_pacnt_storage COMMAND blofeld-pacnt-storage-test)
  add_executable(blofeld-pacnt-native-test test/mcu_pacnt_native_test.cpp)
  target_link_libraries(blofeld-pacnt-native-test PRIVATE blofeld_core)
  add_test(NAME mcu_pacnt_native COMMAND blofeld-pacnt-native-test)
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
    ENVIRONMENT "BLOFELD_TEST_BINARY=$<TARGET_FILE:blofeld-headless>")

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
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla" DESTINATION share/blofeld-emu)

file(GLOB_RECURSE BLOFELD_FORMAT_SOURCES CONFIGURE_DEPENDS
  "src/*.hpp" "src/*.cpp" "test/*.cpp" "test/*.hpp")
add_custom_target(format-check
  COMMAND "${BLOFELD_CLANG_FORMAT}" --dry-run --Werror ${BLOFELD_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
add_custom_target(quality
  COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
  DEPENDS format-check blofeld-startup-probe
  USES_TERMINAL)

endif()

# Common-time SCI input and ordinary host sessions share no modeled OS state.
target_sources(blofeld_reference_board PRIVATE src/board/reference_sci_midi.cpp)
add_library(blofeld_host_midi_session src/app/host_midi_session.cpp)
target_include_directories(blofeld_host_midi_session PUBLIC src)
target_link_libraries(blofeld_host_midi_session PUBLIC blofeld_reference_board blofeld_host_midi_port)
if(BUILD_TESTING)

  add_test(NAME reference_midi_session_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_midi_session_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-midi-session-cli")
  set_tests_properties(reference_midi_session_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_test(NAME reference_audio_live_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_audio_live_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-audio-live-cli")
  set_tests_properties(reference_audio_live_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)

endif()

# Independent external waveforms cover receive stop/start overlap.
if(BUILD_TESTING)
  add_executable(blofeld-sci-fast-test test/mcu_sci_fast_test.cpp)
  target_link_libraries(blofeld-sci-fast-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_sci_fast COMMAND blofeld-sci-fast-test)
  add_executable(blofeld-sci-overlap-test test/mcu_sci_overlap_test.cpp)
  target_link_libraries(blofeld-sci-overlap-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_sci_overlap COMMAND blofeld-sci-overlap-test)
  add_dependencies(quality blofeld-sci-fast-test blofeld-sci-overlap-test  )
endif()

# Explicit multi-driver GPIO voltage and original PU0 release/readback controls.
if(BUILD_TESTING)
  add_executable(blofeld-gpio-multiple-drivers-test test/gpio_multiple_drivers_test.cpp)
  target_link_libraries(blofeld-gpio-multiple-drivers-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME gpio_multiple_drivers COMMAND blofeld-gpio-multiple-drivers-test
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
  set_tests_properties(gpio_multiple_drivers PROPERTIES TIMEOUT 60)
  add_executable(blofeld-original-gpio-reset-test test/original_gpio_reset_test.cpp)
  target_link_libraries(blofeld-original-gpio-reset-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME original_gpio_reset COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_reset.py"
    $<TARGET_FILE:blofeld-original-gpio-reset-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-reset.json")
  set_tests_properties(original_gpio_reset PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
  add_dependencies(quality blofeld-gpio-multiple-drivers-test blofeld-original-gpio-reset-test)
endif()

# Keep functional halt separate from the self-clearing software STALL command.
if(BUILD_TESTING)
  add_executable(blofeld-usb-functional-halt-test test/usb_functional_halt_test.cpp)
  target_link_libraries(blofeld-usb-functional-halt-test PRIVATE blofeld_mcu)
  add_test(NAME usb_functional_halt COMMAND blofeld-usb-functional-halt-test)
  add_dependencies(quality blofeld-usb-functional-halt-test)
endif()

# Native scalar/repeated full-accumulator stores and independent pipeline oracles.

# Automatic endpoint status remains separate from firmware-owned descriptor requests.
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

# Mn retains raw24 data while documented modulo arithmetic decodes only low16.
if(BUILD_TESTING)
  add_dependencies(quality blofeld-modifier-agu-test)
endif()

# The alternate host service is isolated from all user and physical audio devices.

# Raw modifier sources retain native register/cache phases and strict operand guards.

# Partial accumulator stores preserve raw data/CCR through ordinary and REP phases.
if(BUILD_TESTING)

  # Timer register accesses retain real native effects beside independent DMA.
  foreach(mode IN ITEMS native ports words reads serial)
    if(mode STREQUAL "native")
      set(timer_source test/dsp_timer_dma_test.cpp)
      set(timer_name )
    else()
      set(timer_source test/dsp_timer_dma_${mode}_test.cpp)
      set(timer_name dsp_timer_dma_${mode})
    endif()

  endforeach()

endif()

# Standalone accumulator loads retain native values, phases and transfer interlocks.

# Indexed/predecrement scalar moves retain native AGU ownership and explicit reference latency.

# Source dependencies and preupdate waits constrain the same earliest memory phase.

# Fixed AA6 loads retain native values and explicit non-AGU address ownership.

# Memory inputs preserve raw destination fields independently of supported SR scaling.

# Native REP counts and raw data/N bodies retain fixed reference-stage contracts.

# Native call/return frames retain independently qualified nominal windows.

# Raw middle-word transfers retain independently qualified source interlocks.

# GPIO inputs use the documented passive internal pull-downs.
if(BUILD_TESTING)
  add_executable(blofeld-dsp-gpio-pull-test test/dsp_gpio_pull_test.cpp)
  target_link_libraries(blofeld-dsp-gpio-pull-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_gpio_pull COMMAND blofeld-dsp-gpio-pull-test)
  set_tests_properties(dsp_gpio_pull PROPERTIES TIMEOUT 30)
  add_dependencies(quality blofeld-dsp-gpio-pull-test)
endif()

# Parallel raw middle words preserve old operands and ordinary ALU forwarding.

# Explicit P-package read continuation follows qualified output-end edges.
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

# Occurrence-bound loop/call forecasts retain unchanged software reference windows.

# Raw program-memory operands retain distinct reference bus/source-wait contracts.

# Native controller words and actual DMA holds share the explicit reference cycle.

# Explicit free-running audio input clocks retain one declared source and fanout.
if(BUILD_TESTING)
  add_executable(blofeld-square-clock-test test/reference_square_clock_test.cpp
    test/reference_audio_clock_test.cpp)
  target_link_libraries(blofeld-square-clock-test PRIVATE blofeld_reference_board)

  add_dependencies(quality blofeld-square-clock-test )
  add_test(NAME reference_square_clock_source COMMAND blofeld-square-clock-test source)
  add_test(NAME reference_square_clock_serial COMMAND blofeld-square-clock-test serial)
  add_test(NAME reference_square_clock_snapshots COMMAND blofeld-square-clock-test snapshots)

  set_tests_properties(reference_square_clock_source reference_square_clock_serial
    reference_square_clock_snapshots

    PROPERTIES LABELS reference_audio_clock TIMEOUT 300)
endif()

if(BUILD_TESTING)
  add_test(NAME reference_audio_clock_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_audio_clock_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-rom-fixture>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/test-artifacts/reference-audio-clock")
  set_tests_properties(reference_audio_clock_cli PROPERTIES LABELS reference_audio_clock TIMEOUT 300)
endif()

# Extension Decode overlaps opcode AG1 and retains the actual F2 word identity.

if(BUILD_TESTING)
  add_dependencies(quality blofeld-nand-ce-window-test blofeld-nand-initial-observation-test
    blofeld-nand-standby-test blofeld-nand-read-exit-test)
  add_dependencies(quality blofeld-nand-read-transition-test)
endif()

if(BUILD_TESTING)
  add_dependencies(quality blofeld-peripheral-calendar-test  )
endif()

# Short absolute Jcc uses owned CCR resolution and genuine native retirement.

# Causal cell verification and actual native GPIO status/write/readback paths.
if(BUILD_TESTING)

  include(cmake/MasterHistoryTests.cmake)

  add_executable(blofeld-nand-peer-timebase-test test/nand_peer_timebase_test.cpp)
  target_link_libraries(blofeld-nand-peer-timebase-test PRIVATE blofeld_nand_peer)
  add_test(NAME nand_peer_timebase COMMAND blofeld-nand-peer-timebase-test)
  set_tests_properties(nand_peer_timebase PROPERTIES TIMEOUT 30)
  add_dependencies(quality blofeld-nand-peer-timebase-test)
  add_executable(blofeld-nand-bus-decode-test test/nand_bus_decode_test.cpp)
  target_link_libraries(blofeld-nand-bus-decode-test PRIVATE blofeld_nand_peer)
  add_test(NAME nand_bus_decode COMMAND blofeld-nand-bus-decode-test)
  set_tests_properties(nand_bus_decode PROPERTIES TIMEOUT 30)
  add_dependencies(quality blofeld-nand-bus-decode-test)

  foreach(kind write_medium busy_status)
    string(REPLACE "_" "-" target_suffix "${kind}")
    set(target "blofeld-nand-${target_suffix}-test")

  endforeach()
endif()

# Parallel internal TFR and two-word absolute X/Y reads.

# Native core accesses across distinct physical RAM modules.

# Native full-accumulator MOVEP stores and their source/port ownership.

if(BUILD_TESTING)
  add_dependencies(quality blofeld-nand-wide-timebase-test)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-mcu-dsp-gpio-peer-test test/mcu_dsp_gpio_peer_test.cpp)
  target_link_libraries(blofeld-mcu-dsp-gpio-peer-test PRIVATE blofeld_reference_board)
  add_test(NAME mcu_dsp_gpio_peer COMMAND blofeld-mcu-dsp-gpio-peer-test)
  set_tests_properties(mcu_dsp_gpio_peer PROPERTIES TIMEOUT 30)

  add_dependencies(quality blofeld-mcu-dsp-gpio-peer-test )
endif()

if(BUILD_TESTING)
  add_test(NAME reference_display_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_display_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-display-cli")
  set_tests_properties(reference_display_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
endif()

# Durable ownership of the existing timed NAND model and actual native pin path.
if(BUILD_TESTING)
  add_executable(blofeld-nand-write-store-test test/nand_write_store_test.cpp)
  target_link_libraries(blofeld-nand-write-store-test PRIVATE blofeld_nand_write_store)
  add_test(NAME nand_write_store COMMAND blofeld-nand-write-store-test)
  set_tests_properties(nand_write_store PROPERTIES TIMEOUT 30)

  add_dependencies(quality blofeld-nand-write-store-test
    )
endif()

if(TARGET blofeld-dma-callback-allocation-test)
  add_dependencies(quality blofeld-dma-callback-allocation-test)
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
