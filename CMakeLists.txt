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
include(cmake/DspCore.cmake)
include(cmake/PulseAudio.cmake)
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

include(cmake/Hc12Spec.cmake)
include(cmake/Hcs08Spec.cmake)
add_library(blofeld_firmware src/firmware/firmware.cpp src/firmware/archive.cpp src/firmware/midi_file.cpp src/common/diagnostic.cpp src/common/bus.cpp)
target_include_directories(blofeld_firmware PUBLIC src)
target_link_libraries(blofeld_firmware PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_hcs08 src/keyboard/hcs08.cpp src/keyboard/hcs08_interrupts.cpp src/keyboard/hcs08_registers.cpp src/keyboard/hcs08_ccr.cpp)
target_include_directories(blofeld_hcs08 PUBLIC src)
target_link_libraries(blofeld_hcs08 PUBLIC blofeld_firmware PRIVATE blofeld_sleigh)
add_dependencies(blofeld_hcs08 blofeld_hcs08_spec)
add_library(blofeld_hcs08_icg src/keyboard/hcs08_icg.cpp)
target_include_directories(blofeld_hcs08_icg PUBLIC src)
target_link_libraries(blofeld_hcs08_icg PUBLIC blofeld_firmware)
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
  src/keyboard/hcs08_flash_array.cpp src/keyboard/hcs08_flash_reset.cpp)
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
target_link_libraries(blofeld_hcs08_spi_run_clock PUBLIC blofeld_hcs08_bdc blofeld_hcs08_spi_clock)
add_library(blofeld_gpio_pads src/board/gpio_pads.cpp)
target_include_directories(blofeld_gpio_pads PUBLIC src)
target_link_libraries(blofeld_gpio_pads PUBLIC blofeld_firmware)
add_library(blofeld_hcs08_gpio src/keyboard/hcs08_gpio.cpp)
target_include_directories(blofeld_hcs08_gpio PUBLIC src)
target_link_libraries(blofeld_hcs08_gpio PUBLIC blofeld_gpio_pads)
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
add_library(blofeld_nand src/storage/nand.cpp src/storage/nand_write.cpp src/storage/nand_array.cpp src/storage/nand_state.cpp
  src/storage/nand_image.cpp src/storage/nand_image_codec.cpp)
target_include_directories(blofeld_nand PUBLIC src)
target_link_libraries(blofeld_nand PUBLIC blofeld_firmware PRIVATE ZLIB::ZLIB)
add_library(blofeld_nand_operation_clock src/storage/nand_operation_clock.cpp)
target_include_directories(blofeld_nand_operation_clock PUBLIC src)
target_link_libraries(blofeld_nand_operation_clock PUBLIC blofeld_nand)
add_library(blofeld_nand_read_timing src/storage/nand_read_timing.cpp)
target_include_directories(blofeld_nand_read_timing PUBLIC src)
target_link_libraries(blofeld_nand_read_timing PUBLIC blofeld_nand)
add_library(blofeld_nand_write_timing src/storage/nand_write_timing.cpp)
target_include_directories(blofeld_nand_write_timing PUBLIC src)
target_link_libraries(blofeld_nand_write_timing PUBLIC blofeld_nand)
add_library(blofeld_nand_read_setup src/storage/nand_read_setup.cpp)
target_include_directories(blofeld_nand_read_setup PUBLIC src)
target_link_libraries(blofeld_nand_read_setup PUBLIC blofeld_nand)
add_library(blofeld_nand_store src/storage/nand_file.cpp src/storage/nand_store.cpp)
target_include_directories(blofeld_nand_store PUBLIC src)
target_link_libraries(blofeld_nand_store PUBLIC blofeld_nand PRIVATE blofeld_private_files)
add_library(blofeld_mcu src/mcu/mcu_bus.cpp src/mcu/mcu_bus_clone.cpp src/mcu/mcu_bus_usb.cpp src/mcu/mcu_code_cache.cpp src/mcu/mcu_memory.cpp src/mcu/mcu_gpio.cpp
  src/mcu/mcu_flash_config.cpp src/mcu/mcu_flash.cpp src/mcu/mcu_flash_backdoor.cpp src/mcu/mcu_flash_memory.cpp src/mcu/mcu_bus_flash.cpp
  src/mcu/mcu_clock.cpp src/mcu/mcu_cop.cpp src/mcu/mcu_reset.cpp src/mcu/mcu_run_clock.cpp src/mcu/mcu_sci.cpp src/mcu/mcu_timer.cpp src/mcu/mcu_interrupts.cpp
  src/mcu/mcu_usb.cpp src/mcu/mcu_usb_io.cpp src/mcu/mcu_usb_status.cpp src/mcu/mcu_usb_packets.cpp
  src/mcu/mcu_usb_control.cpp
  src/mcu/mcu_usb_ique_control.cpp src/mcu/mcu_usb_reset.cpp src/mcu/mcu_usb_host.cpp)
target_include_directories(blofeld_mcu PUBLIC src)
target_link_libraries(blofeld_mcu PUBLIC blofeld_firmware blofeld_storage blofeld_gpio_pads)
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
add_library(blofeld_core src/mcu/hc12.cpp src/mcu/hc12_decimal.cpp src/mcu/mcu_machine.cpp)
add_library(blofeld_hc12_cycles src/mcu/hc12_cycle_plan.cpp src/mcu/hc12_cycle_page2.cpp src/mcu/hc12_cycle_bus.cpp)
target_include_directories(blofeld_hc12_cycles PUBLIC src)
target_link_libraries(blofeld_hc12_cycles PUBLIC blofeld_firmware)
add_library(blofeld_midi_serial src/midi/midi_serial.cpp)
target_include_directories(blofeld_midi_serial PUBLIC src)
target_link_libraries(blofeld_midi_serial PUBLIC blofeld_firmware)
add_library(blofeld_midi_timed src/midi/midi_timed.cpp)
target_include_directories(blofeld_midi_timed PUBLIC src)
target_link_libraries(blofeld_midi_timed PUBLIC blofeld_midi_serial)
add_library(blofeld_serial_pcm src/audio/serial_pcm.cpp src/audio/pcm_sink.cpp)
target_include_directories(blofeld_serial_pcm PUBLIC src)
target_link_libraries(blofeld_serial_pcm PUBLIC blofeld_firmware)
add_library(blofeld_pcm_transport src/audio/pcm_recording.cpp src/audio/pcm_wave.cpp src/audio/pcm_wave_file.cpp)
target_include_directories(blofeld_pcm_transport PUBLIC src)
target_link_libraries(blofeld_pcm_transport PUBLIC blofeld_serial_pcm PRIVATE blofeld_private_files)
add_library(blofeld_host_audio src/audio/host_audio.cpp src/audio/host_audio_pcm.cpp)
target_include_directories(blofeld_host_audio PUBLIC src)
target_link_libraries(blofeld_host_audio PUBLIC blofeld_serial_pcm PRIVATE blofeld_pulse)
add_library(blofeld_pcm_queue src/audio/pcm_queue.cpp)
target_include_directories(blofeld_pcm_queue PUBLIC src)
target_link_libraries(blofeld_pcm_queue PUBLIC blofeld_serial_pcm)
add_library(blofeld_audio_runtime src/audio/audio_runtime.cpp src/audio/audio_worker.cpp)
target_include_directories(blofeld_audio_runtime PUBLIC src)
target_link_libraries(blofeld_audio_runtime PUBLIC blofeld_host_audio blofeld_pcm_queue PRIVATE Threads::Threads)
add_library(blofeld_mcu_midi src/mcu/mcu_midi.cpp src/mcu/mcu_clocked_midi.cpp)
target_include_directories(blofeld_mcu_midi PUBLIC src)
target_link_libraries(blofeld_mcu_midi PUBLIC blofeld_mcu blofeld_midi_serial)
add_library(blofeld_dsp_peripherals src/dsp/dsp_shi.cpp src/dsp/dsp_shi_spi.cpp src/dsp/dsp_bus.cpp src/dsp/dsp_pll.cpp
  src/dsp/dsp_esai.cpp src/dsp/dsp_esai_transmit.cpp src/dsp/dsp_esai_clock.cpp src/dsp/dsp_bus_esai.cpp
  src/dsp/dsp_dma.cpp src/dsp/dsp_dma_address.cpp src/dsp/dsp_dma_cycle.cpp src/dsp/dsp_bus_dma.cpp
  src/dsp/dsp_dax.cpp src/dsp/dsp_dax_clock.cpp src/dsp/dsp_bus_dax.cpp
  src/dsp/dsp_gpio.cpp src/dsp/dsp_timers.cpp src/dsp/dsp_timer_clock.cpp src/dsp/dsp_timer_capture.cpp src/dsp/dsp_interrupts.cpp
  src/dsp/dsp_run_clock.cpp src/dsp/dsp_run_reset.cpp)
target_include_directories(blofeld_dsp_peripherals PUBLIC src)
target_link_libraries(blofeld_dsp_peripherals PUBLIC blofeld_firmware blofeld_gpio_pads)
add_library(blofeld_dsp
  src/dsp/dsp_memory_form.cpp src/dsp/dsp_address_stages.cpp src/dsp/dsp_core.cpp
  src/dsp/dsp_move_writeback.cpp src/dsp/dsp_address_phase.cpp src/dsp/dsp_native_access.cpp
  src/dsp/dsp_reference_execution.cpp src/dsp/dsp_program_fetch.cpp src/dsp/dsp_reference_cycle.cpp
  src/dsp/dsp_fetch_run.cpp src/dsp/dsp_fetch_state.cpp src/dsp/dsp_fetch_checkpoint.cpp
  src/dsp/dsp_pipeline.cpp src/dsp/dsp_pipeline_decode.cpp src/dsp/dsp_pipeline_transfer.cpp
  src/dsp/dsp_pipeline_execute.cpp src/dsp/dsp_interlocks.cpp src/dsp/dsp_interlock_decode.cpp
  src/dsp/dsp_interrupt_timing.cpp src/dsp/dsp_machine.cpp src/dsp/dsp_scheduled_run.cpp
  src/dsp/dsp_scheduled_events.cpp src/dsp/dsp_agu_interlocks.cpp src/dsp/dsp_machine_reference_dma.cpp
  src/dsp/dsp_scheduled_reset.cpp)
target_include_directories(blofeld_dsp PUBLIC src)
target_link_libraries(blofeld_dsp PUBLIC blofeld_dsp_peripherals PRIVATE blofeld_dsp_native)
if(BUILD_TESTING OR BLOFELD_FUZZING)
  add_library(blofeld_dsp_instruction_fixture test/dsp_instruction_fixture.cpp)
  target_link_libraries(blofeld_dsp_instruction_fixture PUBLIC blofeld_dsp PRIVATE blofeld_dsp_native)
endif()
add_dependencies(blofeld_core blofeld_hc12_spec)
target_include_directories(blofeld_core PUBLIC src)
target_link_libraries(blofeld_core PUBLIC blofeld_mcu PRIVATE blofeld_sleigh blofeld_hc12_cycles)
target_compile_definitions(blofeld_core PUBLIC
  BLOFELD_DEFAULT_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
add_library(blofeld_mcu_dsp src/board/mcu_dsp_schedule.cpp)
target_include_directories(blofeld_mcu_dsp PUBLIC src)
target_link_libraries(blofeld_mcu_dsp PUBLIC blofeld_core blofeld_dsp)
add_library(blofeld_host_midi src/midi/host_midi.cpp)
target_include_directories(blofeld_host_midi PUBLIC src)
target_link_libraries(blofeld_host_midi PUBLIC blofeld_quality)
add_library(blofeld_reference_board src/board/reference_config.cpp
  src/board/reference_board.cpp src/board/reference_wiring.cpp src/board/reference_returns.cpp src/board/reference_checkpoint.cpp
  src/board/reference_dsp_inputs.cpp src/board/reference_dsp_clocks.cpp src/board/reference_pcm.cpp src/board/reference_audio.cpp
  src/board/reference_midi.cpp src/board/reference_dsp_midi.cpp)
target_include_directories(blofeld_reference_board PUBLIC src)
target_link_libraries(blofeld_reference_board PUBLIC blofeld_mcu_dsp blofeld_private_files
  blofeld_pcm_transport blofeld_mcu_midi blofeld_midi_timed blofeld_usbip_tcp)
add_executable(blofeld-headless src/app/main.cpp src/app/host_midi_runner.cpp)
target_link_libraries(blofeld-headless PRIVATE blofeld_reference_board blofeld_host_midi blofeld_sample_library blofeld_device_identity)

if(BUILD_TESTING)
  # One checked translation unit for the shared target-memory reference fixture.
  add_library(blofeld_hcs08_programmer_fixture OBJECT test/hcs08_programmer_fixture.cpp)
  target_link_libraries(blofeld_hcs08_programmer_fixture PUBLIC
    blofeld_hcs08 blofeld_hcs08_flash blofeld_hcs08_icg)
  add_executable(blofeld-hcs08-core-test test/hcs08_core_test.cpp)
  target_link_libraries(blofeld-hcs08-core-test PRIVATE blofeld_hcs08)
  target_compile_definitions(blofeld-hcs08-core-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME hcs08_core COMMAND blofeld-hcs08-core-test)
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
  add_executable(blofeld-original-keyboard-chain-test test/original_keyboard_chain_test.cpp
    test/keyboard_chain_fixture.cpp test/keyboard_chain_execution.cpp test/keyboard_chain_trace.cpp
    test/keyboard_chain_checkpoint.cpp test/keyboard_chain_overlap.cpp
    test/keyboard_reset_fixture.cpp)
  target_link_libraries(blofeld-original-keyboard-chain-test PRIVATE blofeld_hcs08_programmer_fixture)
  target_link_libraries(blofeld-original-keyboard-chain-test PRIVATE blofeld_core blofeld_mcu_dsp
    blofeld_dsp_native blofeld_lcd_fixture blofeld_lcd_export
    blofeld_hcs08_reset blofeld_hcs08_bdc_wire blofeld_hcs08_run_clock)
  target_compile_definitions(blofeld-original-keyboard-chain-test PRIVATE
    BLOFELD_HCS08_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla")
  add_test(NAME original_keyboard_chain COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_chain.py"
    $<TARGET_FILE:blofeld-original-keyboard-chain-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HCS08.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-chain.trace")
  set_tests_properties(original_keyboard_chain PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 520)
  add_executable(blofeld-audio-runtime-test test/audio_runtime_test.cpp
    test/audio_runtime_failure.cpp test/audio_runtime_board.cpp test/audio_runtime_underrun.cpp)
  target_link_libraries(blofeld-audio-runtime-test PRIVATE blofeld_audio_runtime blofeld_reference_board)
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
  target_link_libraries(blofeld-pcm-queue-test PRIVATE blofeld_pcm_queue Threads::Threads)
  add_test(NAME pcm_queue COMMAND blofeld-pcm-queue-test)
  set_tests_properties(pcm_queue PROPERTIES TIMEOUT 40)
  add_executable(blofeld-host-audio-test test/host_audio_test.cpp test/host_audio_native.cpp)
  target_link_libraries(blofeld-host-audio-test PRIVATE blofeld_host_audio blofeld_dsp blofeld_dsp_native)
  add_test(NAME host_audio COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/host_audio.py" $<TARGET_FILE:blofeld-host-audio-test>)
  set_tests_properties(host_audio PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
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
  set_tests_properties(reference_dsp_host_midi_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
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
  target_link_libraries(blofeld-pcm-transport-test PRIVATE blofeld_pcm_transport blofeld_private_files)
  add_test(NAME pcm_transport COMMAND blofeld-pcm-transport-test
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/pcm-transport")
  add_executable(blofeld-serial-pcm-test test/serial_pcm_test.cpp test/serial_pcm_native.cpp)
  target_link_libraries(blofeld-serial-pcm-test PRIVATE blofeld_serial_pcm blofeld_dsp blofeld_dsp_native)
  add_test(NAME serial_pcm COMMAND blofeld-serial-pcm-test)
  add_executable(blofeld-esai-test test/dsp_esai_test.cpp test/dsp_esai_native.cpp)
  target_link_libraries(blofeld-esai-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_esai COMMAND blofeld-esai-test)
  add_executable(blofeld-esai-interrupt-test
    test/dsp_esai_interrupt_test.cpp test/dsp_esai_interrupt_native.cpp)
  target_link_libraries(blofeld-esai-interrupt-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_esai_interrupts COMMAND blofeld-esai-interrupt-test)
  add_executable(blofeld-esai-clock-test test/dsp_esai_clock_test.cpp test/dsp_esai_clock_native.cpp)
  target_link_libraries(blofeld-esai-clock-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_esai_clock COMMAND blofeld-esai-clock-test)
  add_executable(blofeld-dma-test test/dsp_dma_test.cpp test/dsp_dma_state.cpp test/dsp_dma_native.cpp)
  target_link_libraries(blofeld-dma-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_dma COMMAND blofeld-dma-test)
  add_executable(blofeld-bank-test test/dsp_bank_test.cpp test/dsp_bank_phase.cpp test/dsp_bank_native.cpp)
  target_link_libraries(blofeld-bank-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_bank_arbitration COMMAND blofeld-bank-test)
  add_executable(blofeld-repeat-test test/dsp_repeat_test.cpp)
  target_link_libraries(blofeld-repeat-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_repeat_restrictions COMMAND blofeld-repeat-test)
  add_executable(blofeld-reference-access-test test/dsp_reference_test.cpp
    test/dsp_reference_core.cpp test/dsp_reference_dma.cpp test/dsp_reference_interrupts.cpp)
  target_link_libraries(blofeld-reference-access-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_reference_access COMMAND blofeld-reference-access-test)
  add_executable(blofeld-reference-dma-admission-test test/dsp_reference_dma_admission_test.cpp)
  target_link_libraries(blofeld-reference-dma-admission-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_reference_dma_admission COMMAND blofeld-reference-dma-admission-test)
  add_executable(blofeld-program-latch-test test/dsp_program_latch_test.cpp)
  target_link_libraries(blofeld-program-latch-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_program_latch COMMAND blofeld-program-latch-test)
  add_executable(blofeld-prefetch-queue-test test/dsp_prefetch_queue_test.cpp)
  target_link_libraries(blofeld-prefetch-queue-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_prefetch_queue COMMAND blofeld-prefetch-queue-test)
  add_executable(blofeld-prefetch-condition-test test/dsp_prefetch_condition_test.cpp)
  target_link_libraries(blofeld-prefetch-condition-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_prefetch_conditions COMMAND blofeld-prefetch-condition-test)
  add_executable(blofeld-reference-cycle-test test/dsp_reference_cycle_test.cpp)
  target_link_libraries(blofeld-reference-cycle-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_reference_bus_cycle COMMAND blofeld-reference-cycle-test)
  add_executable(blofeld-fetch-run-test test/dsp_fetch_run_test.cpp test/dsp_fetch_run_guards.cpp)
  target_link_libraries(blofeld-fetch-run-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_fetch_run COMMAND blofeld-fetch-run-test)
  add_executable(blofeld-retirement-test test/dsp_retirement_test.cpp test/dsp_retirement_fetch.cpp)
  target_link_libraries(blofeld-retirement-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_reference_retirement COMMAND blofeld-retirement-test)
  add_executable(blofeld-address-phase-test test/dsp_address_phase_test.cpp test/dsp_address_phase_clock.cpp)
  target_link_libraries(blofeld-address-phase-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_address_phase COMMAND blofeld-address-phase-test)
  add_executable(blofeld-pipeline-test test/dsp_pipeline_test.cpp test/dsp_pipeline_state.cpp test/dsp_pipeline_guards.cpp)
  target_link_libraries(blofeld-pipeline-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_automatic_alu_pipeline COMMAND blofeld-pipeline-test)
  add_executable(blofeld-memory-pipeline-test test/dsp_memory_pipeline_test.cpp test/dsp_memory_pipeline_stream.cpp test/dsp_memory_pipeline_guards.cpp)
  target_link_libraries(blofeld-memory-pipeline-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_automatic_memory_pipeline COMMAND blofeld-memory-pipeline-test)
  add_executable(blofeld-parallel-memory-test test/dsp_parallel_memory_test.cpp
    test/dsp_parallel_memory_execution.cpp test/dsp_parallel_memory_state.cpp)
  target_link_libraries(blofeld-parallel-memory-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_parallel_memory_addresses COMMAND blofeld-parallel-memory-test address)
  add_test(NAME dsp_parallel_memory_execution COMMAND blofeld-parallel-memory-test execution)
  add_test(NAME dsp_parallel_memory_state COMMAND blofeld-parallel-memory-test state)
  add_executable(blofeld-transfer-pipeline-test test/dsp_transfer_pipeline_fields.cpp
    test/dsp_transfer_pipeline_agu.cpp test/dsp_transfer_pipeline_state.cpp)
  target_link_libraries(blofeld-transfer-pipeline-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_pipeline_transfers COMMAND blofeld-transfer-pipeline-test fields)
  add_test(NAME dsp_pipeline_agu_dependencies COMMAND blofeld-transfer-pipeline-test agu)
  add_test(NAME dsp_pipeline_transfer_state COMMAND blofeld-transfer-pipeline-test state)
  add_executable(blofeld-peripheral-pipeline-test test/dsp_peripheral_pipeline_test.cpp
    test/dsp_peripheral_pipeline_writes.cpp test/dsp_peripheral_pipeline_state.cpp)
  target_link_libraries(blofeld-peripheral-pipeline-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_pipeline_peripheral_writes COMMAND blofeld-peripheral-pipeline-test writes)
  add_test(NAME dsp_pipeline_peripheral_state COMMAND blofeld-peripheral-pipeline-test state)
  add_test(NAME dsp_pipeline_bootstrap_prefix COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/dsp_bootstrap_pipeline.py"
    $<TARGET_FILE:blofeld-peripheral-pipeline-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/dsp-pipeline-bootstrap-prefix.trace")
  set_tests_properties(dsp_pipeline_bootstrap_prefix PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-overlap-test test/dsp_overlap_test.cpp test/dsp_overlap_clock.cpp)
  target_link_libraries(blofeld-overlap-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_native_overlap COMMAND blofeld-overlap-test)
  add_executable(blofeld-writeback-test test/dsp_writeback_test.cpp test/dsp_writeback_parallel.cpp test/dsp_writeback_clock.cpp)
  target_link_libraries(blofeld-writeback-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_move_writeback COMMAND blofeld-writeback-test)
  add_executable(blofeld-dax-test test/dsp_dax_test.cpp test/dsp_dax_native.cpp test/dsp_dax_dma.cpp)
  target_link_libraries(blofeld-dax-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_dax COMMAND blofeld-dax-test)
  add_executable(blofeld-reference-checkpoint-test test/reference_checkpoint.cpp)
  target_link_libraries(blofeld-reference-checkpoint-test PRIVATE blofeld_reference_board blofeld_dsp_native)
  add_test(NAME reference_checkpoint COMMAND blofeld-reference-checkpoint-test
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-checkpoint")
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
  add_executable(blofeld-dsp-completion-observer-test test/dsp_completion_observer.cpp)
  target_link_libraries(blofeld-dsp-completion-observer-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_completion_observer COMMAND blofeld-dsp-completion-observer-test)
  add_executable(blofeld-dsp-native-access-test test/dsp_native_access_test.cpp
    test/dsp_native_access_failures.cpp test/dsp_native_access_continuations.cpp)
  target_link_libraries(blofeld-dsp-native-access-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_native_access COMMAND blofeld-dsp-native-access-test)
  add_executable(blofeld-dsp-external-events-test test/dsp_external_events.cpp
    test/dsp_external_esai.cpp test/mcu_dsp_external_events.cpp)
  target_link_libraries(blofeld-dsp-external-events-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME dsp_external_events COMMAND blofeld-dsp-external-events-test)
  add_executable(blofeld-reference-original-midi-rom test/reference_original_midi_rom.cpp)
  target_link_libraries(blofeld-reference-original-midi-rom PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME reference_original_midi_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/reference_original_midi_cli.py"
    $<TARGET_FILE:blofeld-headless> $<TARGET_FILE:blofeld-reference-original-midi-rom>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/reference-original-midi")
  set_tests_properties(reference_original_midi_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-reference-rom-fixture test/reference_rom.cpp)
  target_link_libraries(blofeld-reference-rom-fixture PRIVATE blofeld_dsp blofeld_dsp_native)
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
  add_executable(blofeld-mcu-dsp-schedule-test test/mcu_dsp_schedule_test.cpp)
  target_link_libraries(blofeld-mcu-dsp-schedule-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME mcu_dsp_schedule COMMAND blofeld-mcu-dsp-schedule-test)
  add_executable(blofeld-mcu-dsp-wait-test test/mcu_dsp_wait_test.cpp)
  target_link_libraries(blofeld-mcu-dsp-wait-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME mcu_dsp_wait COMMAND blofeld-mcu-dsp-wait-test)
  add_executable(blofeld-hc12-cycle-test test/hc12_cycle_test.cpp)
  target_link_libraries(blofeld-hc12-cycle-test PRIVATE blofeld_core blofeld_hc12_cycles)
  add_test(NAME hc12_cycles COMMAND blofeld-hc12-cycle-test)
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
  add_executable(blofeld-mcu-clock-checkpoint-test test/mcu_clock_checkpoint_test.cpp)
  target_link_libraries(blofeld-mcu-clock-checkpoint-test PRIVATE blofeld_core)
  add_test(NAME mcu_clock_checkpoint COMMAND blofeld-mcu-clock-checkpoint-test)
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
  add_test(NAME dependency_instrumentation COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/dependency_instrumentation.py"
    --source "${CMAKE_CURRENT_SOURCE_DIR}"
    --database "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json"
    --binary $<TARGET_FILE:blofeld-headless>
    --audio-binary $<TARGET_FILE:blofeld-host-audio-test>
    --audio-binary $<TARGET_FILE:blofeld-audio-runtime-test>
    --audio-binary $<TARGET_FILE:blofeld-original-dsp-voice-host-test>
    --audio-binary $<TARGET_FILE:blofeld-original-dsp-voice-audio-test>
    --audio-binary $<TARGET_FILE:blofeld-original-dsp-ring-audio-test>
    --mode "${BLOFELD_SANITIZER}"
    --nm "${BLOFELD_LLVM_NM}" --readelf "${BLOFELD_LLVM_READELF}"
    --native zlibstatic=$<TARGET_FILE:zlibstatic>
    --native blofeld_sleigh=$<TARGET_FILE:blofeld_sleigh>
    --native blofeld_dsp_native=$<TARGET_FILE:blofeld_dsp_native>
    --native asmjit=$<TARGET_FILE:asmjit>
    --native sndfile=$<TARGET_FILE:sndfile>
    --native blofeld_pulse=$<TARGET_FILE:blofeld_pulse>
    --native blofeld_pulse_common=$<TARGET_FILE:blofeld_pulse_common>)
  add_executable(blofeld-midi-serial-test test/midi_serial_test.cpp)
  target_link_libraries(blofeld-midi-serial-test PRIVATE blofeld_midi_serial)
  add_test(NAME midi_serial COMMAND blofeld-midi-serial-test)
  add_executable(blofeld-mcu-midi-test test/mcu_midi_test.cpp)
  target_link_libraries(blofeld-mcu-midi-test PRIVATE blofeld_mcu_midi)
  add_test(NAME mcu_midi COMMAND blofeld-mcu-midi-test)
  add_executable(blofeld-nand-read-test test/nand_read_test.cpp)
  target_link_libraries(blofeld-nand-read-test PRIVATE blofeld_nand)
  add_test(NAME nand_read COMMAND blofeld-nand-read-test)
  add_executable(blofeld-nand-operation-clock-test test/nand_operation_clock_test.cpp)
  target_link_libraries(blofeld-nand-operation-clock-test PRIVATE blofeld_nand_operation_clock)
  add_test(NAME nand_operation_clock COMMAND blofeld-nand-operation-clock-test)
  add_executable(blofeld-nand-read-timing-test test/nand_read_timing_test.cpp)
  target_link_libraries(blofeld-nand-read-timing-test PRIVATE blofeld_nand_read_timing)
  add_test(NAME nand_read_timing COMMAND blofeld-nand-read-timing-test)
  add_executable(blofeld-original-nand-read-timing-test test/original_nand_read_timing_test.cpp)
  target_link_libraries(blofeld-original-nand-read-timing-test PRIVATE
    blofeld_nand_read_timing blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_nand_read_timing COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-nand-read-timing-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-read-timing.trace")
  set_tests_properties(original_nand_read_timing PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-nand-pack-test test/original_nand_pack_test.cpp)
  target_link_libraries(blofeld-original-nand-pack-test PRIVATE
    blofeld_nand_read_timing blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_nand_pack COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-nand-pack-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-pack.trace")
  set_tests_properties(original_nand_pack PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-nand-records-test test/original_nand_records_test.cpp)
  target_link_libraries(blofeld-original-nand-records-test PRIVATE
    blofeld_nand_read_timing blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_nand_records COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_nand_records.py"
    $<TARGET_FILE:blofeld-original-nand-records-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-records.trace")
  set_tests_properties(original_nand_records PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-nand-write-timing-test test/nand_write_timing_test.cpp)
  target_link_libraries(blofeld-nand-write-timing-test PRIVATE blofeld_nand_write_timing)
  add_test(NAME nand_write_timing COMMAND blofeld-nand-write-timing-test)
  add_executable(blofeld-original-nand-write-timing-test test/original_nand_write_timing_test.cpp)
  target_link_libraries(blofeld-original-nand-write-timing-test PRIVATE
    blofeld_nand_write_timing blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_nand_write_timing COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-nand-write-timing-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-write-timing.trace")
  set_tests_properties(original_nand_write_timing PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-nand-read-setup-test test/nand_read_setup_test.cpp)
  target_link_libraries(blofeld-nand-read-setup-test PRIVATE blofeld_nand_read_setup)
  add_test(NAME nand_read_setup COMMAND blofeld-nand-read-setup-test)
  add_executable(blofeld-original-nand-read-setup-test test/original_nand_read_setup_test.cpp)
  target_link_libraries(blofeld-original-nand-read-setup-test PRIVATE
    blofeld_nand_operation_clock blofeld_nand_read_setup blofeld_nand_read_timing blofeld_nand_write_timing
    blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_nand_read_setup COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-nand-read-setup-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-read-setup.trace")
  set_tests_properties(original_nand_read_setup PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-nand-image-test test/nand_image_test.cpp)
  target_link_libraries(blofeld-nand-image-test PRIVATE blofeld_nand)
  add_test(NAME nand_image COMMAND blofeld-nand-image-test)
  add_executable(blofeld-nand-store-test test/nand_store_test.cpp)
  target_link_libraries(blofeld-nand-store-test PRIVATE blofeld_nand_store)
  add_test(NAME nand_store COMMAND blofeld-nand-store-test)
  add_executable(blofeld-nand-write-test test/nand_write_test.cpp)
  target_link_libraries(blofeld-nand-write-test PRIVATE blofeld_nand)
  add_test(NAME nand_write COMMAND blofeld-nand-write-test)
  add_executable(blofeld-nand-power-test test/nand_power_test.cpp)
  target_link_libraries(blofeld-nand-power-test PRIVATE blofeld_nand)
  add_test(NAME nand_power COMMAND blofeld-nand-power-test)
  add_executable(blofeld-nand-gpio-test test/nand_gpio_test.cpp test/nand_gpio_fixture.cpp)
  target_link_libraries(blofeld-nand-gpio-test PRIVATE blofeld_nand_store blofeld_dsp blofeld_dsp_native)
  add_test(NAME nand_gpio COMMAND blofeld-nand-gpio-test)
  add_executable(blofeld-nand-gpio-write-test test/nand_gpio_write_test.cpp test/nand_gpio_fixture.cpp)
  target_link_libraries(blofeld-nand-gpio-write-test PRIVATE blofeld_nand_store blofeld_dsp blofeld_dsp_native)
  add_test(NAME nand_gpio_write COMMAND blofeld-nand-gpio-write-test
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/nand-gpio-write.trace")
  add_test(NAME original_nand_address COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-nand-gpio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-address.trace")
  set_tests_properties(original_nand_address PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-nand-write-test test/original_nand_write_test.cpp test/nand_gpio_fixture.cpp)
  target_link_libraries(blofeld-original-nand-write-test PRIVATE blofeld_nand_store blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_nand_write COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-nand-write-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-nand-write.trace")
  set_tests_properties(original_nand_write PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-update-validation-test test/original_update_validation_test.cpp)
  target_link_libraries(blofeld-original-update-validation-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_update_validation COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_update_validation.py"
    $<TARGET_FILE:blofeld-original-update-validation-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-update-validation.trace")
  set_tests_properties(original_update_validation PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-update-forward-test test/original_update_forward_test.cpp)
  target_link_libraries(blofeld-original-update-forward-test PRIVATE
    blofeld_mcu_dsp blofeld_dsp_native blofeld_lcd_fixture)
  add_executable(blofeld-original-update-coupled-test test/original_update_coupled_test.cpp)
  target_link_libraries(blofeld-original-update-coupled-test PRIVATE
    blofeld_mcu_dsp blofeld_dsp_native blofeld_lcd_fixture blofeld_lcd_export)
  add_test(NAME original_update_coupled COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_update_coupled.py"
    $<TARGET_FILE:blofeld-original-update-coupled-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-update-coupled.trace")
  set_tests_properties(original_update_coupled PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 540)
  add_test(NAME original_update_reset_inputs COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_update_reset_inputs.py"
    $<TARGET_FILE:blofeld-original-update-coupled-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-update-reset-inputs.trace")
  set_tests_properties(original_update_reset_inputs PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_executable(blofeld-original-update-diagnostic-test test/original_update_diagnostic_test.cpp)
  target_link_libraries(blofeld-original-update-diagnostic-test PRIVATE
    blofeld_mcu_dsp blofeld_dsp_native blofeld_lcd_fixture blofeld_lcd_export)
  add_test(NAME original_update_diagnostic COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_update_diagnostic.py"
    $<TARGET_FILE:blofeld-original-update-diagnostic-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-update-diagnostic.trace")
  set_tests_properties(original_update_diagnostic PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_test(NAME original_update_forwarding COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_update_forward.py"
    $<TARGET_FILE:blofeld-original-update-forward-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-update-forward.trace")
  set_tests_properties(original_update_forwarding PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
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
  add_executable(blofeld-dsp-capture-test test/dsp_capture_test.cpp)
  target_link_libraries(blofeld-dsp-capture-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_capture COMMAND blofeld-dsp-capture-test)
  add_executable(blofeld-dsp-timer-events-test test/dsp_timer_events_test.cpp)
  target_link_libraries(blofeld-dsp-timer-events-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_timer_events COMMAND blofeld-dsp-timer-events-test)
  add_executable(blofeld-dsp-interrupt-test test/dsp_interrupt_test.cpp)
  target_link_libraries(blofeld-dsp-interrupt-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_interrupts COMMAND blofeld-dsp-interrupt-test)
  add_executable(blofeld-dsp-irqa-test test/dsp_irqa_test.cpp)
  target_link_libraries(blofeld-dsp-irqa-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_irqa COMMAND blofeld-dsp-irqa-test)
  add_executable(blofeld-dsp-dma-interrupt-test test/dsp_dma_interrupt_test.cpp
    test/dsp_dma_interrupt_native.cpp)
  target_link_libraries(blofeld-dsp-dma-interrupt-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_dma_interrupts COMMAND blofeld-dsp-dma-interrupt-test)
  add_executable(blofeld-original-dsp-gpio-input-test test/original_dsp_gpio_input_test.cpp)
  target_link_libraries(blofeld-original-dsp-gpio-input-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_executable(blofeld-original-dsp-gpio-request-test test/original_dsp_gpio_request_test.cpp
    test/original_dsp_gpio_request_fixture.cpp)
  target_link_libraries(blofeld-original-dsp-gpio-request-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_gpio_request COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_gpio_request.py"
    "$<TARGET_FILE:blofeld-original-dsp-gpio-request-test>"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-gpio-request.trace")
  set_tests_properties(original_dsp_gpio_request PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_test(NAME original_dsp_gpio_input
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_gpio_input.py"
      "$<TARGET_FILE:blofeld-original-dsp-gpio-input-test>"
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-gpio-input.trace")
  set_tests_properties(original_dsp_gpio_input PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-gpio-midi-test
    test/original_gpio_midi_test.cpp test/original_gpio_midi_fixture.cpp
    test/original_gpio_midi_peer.cpp test/original_gpio_midi_endpoint.cpp
    test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-gpio-midi-test PRIVATE
    blofeld_core blofeld_firmware blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_midi
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_midi.py"
      "$<TARGET_FILE:blofeld-original-gpio-midi-test>"
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-midi.trace")
  set_tests_properties(original_gpio_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_test(NAME original_usb_gpio_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_usb_gpio_midi.py"
    $<TARGET_FILE:blofeld-original-gpio-midi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-usb-gpio-midi.trace")
  set_tests_properties(original_usb_gpio_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-dsp-gpio-test test/dsp_gpio_test.cpp)
  target_link_libraries(blofeld-dsp-gpio-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_gpio COMMAND blofeld-dsp-gpio-test)
  add_executable(blofeld-dsp-pll-test test/dsp_pll_test.cpp)
  target_link_libraries(blofeld-dsp-pll-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_pll COMMAND blofeld-dsp-pll-test)
  add_executable(blofeld-dsp-test test/dsp_core_test.cpp)
  target_link_libraries(blofeld-dsp-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_executable(blofeld-dsp-memory-map-test test/dsp_memory_map_test.cpp)
  target_link_libraries(blofeld-dsp-memory-map-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_memory_map COMMAND blofeld-dsp-memory-map-test)
  set_tests_properties(dsp_memory_map PROPERTIES TIMEOUT 60)
  add_executable(blofeld-dsp-instruction-test test/dsp_instruction_test.cpp
    test/dsp_logical_boundaries.cpp test/dsp_bitfield_boundaries.cpp test/dsp_operand_rejection.cpp)
  target_link_libraries(blofeld-dsp-instruction-test PRIVATE blofeld_dsp_instruction_fixture blofeld_dsp_native)
  add_test(NAME dsp_instruction_fixture COMMAND blofeld-dsp-instruction-test)
  set_tests_properties(dsp_instruction_fixture PROPERTIES TIMEOUT 60)
  foreach(control IN ITEMS trap debug)
    add_test(NAME dsp_${control}_conditions COMMAND blofeld-dsp-instruction-test --conditions ${control})
    set_tests_properties(dsp_${control}_conditions PROPERTIES TIMEOUT 60)
  endforeach()
  add_test(NAME dsp_shift_counts COMMAND blofeld-dsp-instruction-test --shift-counts)
  set_tests_properties(dsp_shift_counts PROPERTIES TIMEOUT 60)
  add_test(NAME dsp_operand_bounds COMMAND blofeld-dsp-instruction-test --operand-bounds)
  set_tests_properties(dsp_operand_bounds PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
  add_test(NAME native_dsp COMMAND blofeld-dsp-test)
  add_executable(blofeld-dsp-divide-test test/dsp_divide_test.cpp)
  target_link_libraries(blofeld-dsp-divide-test PRIVATE blofeld_dsp_instruction_fixture blofeld_dsp_native)
  add_test(NAME dsp_divide COMMAND blofeld-dsp-divide-test)
  set_tests_properties(dsp_divide PROPERTIES TIMEOUT 180)
  add_executable(blofeld-dsp-fast-normalization-test test/dsp_fast_normalization_test.cpp)
  target_link_libraries(blofeld-dsp-fast-normalization-test PRIVATE blofeld_dsp_instruction_fixture blofeld_dsp_native)
  add_test(NAME dsp_fast_normalization COMMAND blofeld-dsp-fast-normalization-test)
  set_tests_properties(dsp_fast_normalization PROPERTIES TIMEOUT 180)
  add_executable(blofeld-original-runtime-clocks-test test/original_runtime_clocks.cpp)
  target_link_libraries(blofeld-original-runtime-clocks-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_runtime_clocks COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_runtime_clocks.py"
    $<TARGET_FILE:blofeld-original-runtime-clocks-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-runtime-clocks.trace")
  set_tests_properties(original_runtime_clocks PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-mcu-request-release-test test/original_mcu_request_release.cpp)
  target_link_libraries(blofeld-original-mcu-request-release-test PRIVATE blofeld_core blofeld_firmware)
  add_test(NAME original_mcu_request_release COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_mcu_request_release.py"
    $<TARGET_FILE:blofeld-original-mcu-request-release-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-mcu-request-release.trace")
  set_tests_properties(original_mcu_request_release PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-gpio-pair-test test/original_gpio_pair_test.cpp)
  target_link_libraries(blofeld-original-gpio-pair-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_pair COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_pair.py"
    $<TARGET_FILE:blofeld-original-gpio-pair-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-pair.report")
  set_tests_properties(original_gpio_pair PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_executable(blofeld-original-gpio-pair-checkpoint-test test/original_gpio_pair_checkpoint_test.cpp)
  target_link_libraries(blofeld-original-gpio-pair-checkpoint-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_pair_checkpoints COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_pair_checkpoint.py"
    $<TARGET_FILE:blofeld-original-gpio-pair-checkpoint-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-pair-checkpoint.report")
  set_tests_properties(original_gpio_pair_checkpoints PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-gpio-sequence-test test/original_gpio_sequence_test.cpp)
  target_link_libraries(blofeld-original-gpio-sequence-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_sequence COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_sequence.py"
    $<TARGET_FILE:blofeld-original-gpio-sequence-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-sequence.report")
  set_tests_properties(original_gpio_sequence PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_executable(blofeld-original-gpio-parameter-test test/original_gpio_parameter_test.cpp)
  target_link_libraries(blofeld-original-gpio-parameter-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_parameters COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_parameter.py"
    $<TARGET_FILE:blofeld-original-gpio-parameter-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-parameter.report")
  set_tests_properties(original_gpio_parameters PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_executable(blofeld-original-gpio-sound-test test/original_gpio_sound_test.cpp)
  target_link_libraries(blofeld-original-gpio-sound-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_sound COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_sound.py"
    $<TARGET_FILE:blofeld-original-gpio-sound-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-sound.report")
  set_tests_properties(original_gpio_sound PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-gpio-notification-test test/original_gpio_notification_test.cpp)
  target_link_libraries(blofeld-original-gpio-notification-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_notification COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_notification.py"
    $<TARGET_FILE:blofeld-original-gpio-notification-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-notification.report")
  set_tests_properties(original_gpio_notification PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_executable(blofeld-original-gpio-usb-notification-test test/original_gpio_usb_notification_test.cpp
    test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-gpio-usb-notification-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_usb_notification COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_usb_notification.py"
    $<TARGET_FILE:blofeld-original-gpio-usb-notification-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-usb-notification.report")
  set_tests_properties(original_gpio_usb_notification PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_executable(blofeld-original-gpio-sysex-test test/original_gpio_sysex_test.cpp
    test/usb_endpoint_reset_fixture.cpp)
  target_link_libraries(blofeld-original-gpio-sysex-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_sysex COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_sysex.py"
    $<TARGET_FILE:blofeld-original-gpio-sysex-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-sysex.report")
  set_tests_properties(original_gpio_sysex PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_executable(blofeld-original-gpio-global-wait-test test/original_gpio_global_wait_test.cpp)
  target_link_libraries(blofeld-original-gpio-global-wait-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_gpio_global_wait COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_gpio_global_wait.py"
    $<TARGET_FILE:blofeld-original-gpio-global-wait-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-gpio-global-wait.report")
  set_tests_properties(original_gpio_global_wait PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_executable(blofeld-dsp-dispatch-cache-test test/dsp_dispatch_cache_test.cpp)
  target_link_libraries(blofeld-dsp-dispatch-cache-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_dispatch_cache COMMAND blofeld-dsp-dispatch-cache-test)
  add_executable(blofeld-dsp-cycle-cache-test test/dsp_cycle_cache_test.cpp
    test/dsp_cycle_cache_native.cpp)
  target_link_libraries(blofeld-dsp-cycle-cache-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_cycle_cache COMMAND blofeld-dsp-cycle-cache-test)
  add_executable(blofeld-dsp-memory-storage-test test/dsp_memory_storage_test.cpp)
  target_link_libraries(blofeld-dsp-memory-storage-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_memory_storage COMMAND blofeld-dsp-memory-storage-test)
  add_executable(blofeld-dsp-checkpoint-test test/dsp_checkpoint_test.cpp
    test/dsp_checkpoint_interrupts.cpp test/dsp_checkpoint_peripherals.cpp)
  target_link_libraries(blofeld-dsp-checkpoint-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_checkpoint COMMAND blofeld-dsp-checkpoint-test)
  add_executable(blofeld-dsp-norm-test test/dsp_norm_test.cpp)
  target_link_libraries(blofeld-dsp-norm-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_norm COMMAND blofeld-dsp-norm-test)
  add_executable(blofeld-dsp-transfer-test test/dsp_transfer_test.cpp)
  target_link_libraries(blofeld-dsp-transfer-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_transfers COMMAND blofeld-dsp-transfer-test)
  add_executable(blofeld-dsp-shifted-subtract-test test/dsp_shifted_subtract_test.cpp)
  target_link_libraries(blofeld-dsp-shifted-subtract-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_subl COMMAND blofeld-dsp-shifted-subtract-test --left)
  add_test(NAME dsp_subr COMMAND blofeld-dsp-shifted-subtract-test --right)
  add_test(NAME dsp_subr_host_overflow COMMAND blofeld-dsp-shifted-subtract-test --host-overflow)
  set_tests_properties(dsp_subl dsp_subr dsp_subr_host_overflow PROPERTIES TIMEOUT 120)
  add_executable(blofeld-dsp-saturation-test test/dsp_saturation_test.cpp
    test/dsp_saturation_oracle.cpp test/dsp_saturation_cases.cpp)
  target_link_libraries(blofeld-dsp-saturation-test PRIVATE blofeld_dsp blofeld_dsp_native)
  foreach(scope IN ITEMS regressions arithmetic increment decrement multiply accumulate rounding exceptions mac-host-overflow)
    add_test(NAME dsp_saturation_${scope} COMMAND blofeld-dsp-saturation-test ${scope})
    set_tests_properties(dsp_saturation_${scope} PROPERTIES TIMEOUT 120)
  endforeach()
  add_executable(blofeld-dsp-cmpm-test test/dsp_cmpm_test.cpp)
  add_executable(blofeld-dsp-power-test test/dsp_power_test.cpp test/dsp_saturation_oracle.cpp)
  target_link_libraries(blofeld-dsp-power-test PRIVATE blofeld_dsp blofeld_dsp_native)
  foreach(scope IN ITEMS mpy mpyr mac macr reject-a-mpy reject-b-mpy reject-a-mpyr reject-b-mpyr
      reject-a-mac reject-b-mac reject-a-macr reject-b-macr)
    add_test(NAME dsp_power_${scope} COMMAND blofeld-dsp-power-test ${scope})
    set_tests_properties(dsp_power_${scope} PROPERTIES TIMEOUT 120)
  endforeach()
  target_link_libraries(blofeld-dsp-cmpm-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_cmpm_accumulators COMMAND blofeld-dsp-cmpm-test --full)
  add_test(NAME dsp_cmpm_words COMMAND blofeld-dsp-cmpm-test --word)
  add_test(NAME dsp_cmpm_minimum COMMAND blofeld-dsp-cmpm-test --minimum)
  set_tests_properties(dsp_cmpm_accumulators dsp_cmpm_words dsp_cmpm_minimum PROPERTIES TIMEOUT 120)
  add_executable(blofeld-original-dsp-master-output-test test/original_dsp_master_output_test.cpp
    test/dsp_master_output_fixture.cpp)
  target_link_libraries(blofeld-original-dsp-master-output-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_master_output COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_master_output.py"
    $<TARGET_FILE:blofeld-original-dsp-master-output-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-master-output.trace")
  set_tests_properties(original_dsp_master_output PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_executable(blofeld-original-dsp-sample-test test/original_dsp_sample_test.cpp)
  target_link_libraries(blofeld-original-dsp-sample-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_sample COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_sample.py"
    $<TARGET_FILE:blofeld-original-dsp-sample-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-sample.trace")
  set_tests_properties(original_dsp_sample PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_executable(blofeld-original-dsp-sample-receiver-test test/original_dsp_sample_receiver_test.cpp)
  target_link_libraries(blofeld-original-dsp-sample-receiver-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_sample_receiver COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_sample_receiver.py"
    $<TARGET_FILE:blofeld-original-dsp-sample-receiver-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-sample-receiver.trace")
  set_tests_properties(original_dsp_sample_receiver PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_test(NAME sample_midi_pages COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_midi_pages_test.py")
  add_test(NAME sample_midi_reference COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/sample_midi_pages_test.py" --reference
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/sample-midi-reference.json")
  set_tests_properties(sample_midi_reference PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_test(NAME original_dsp_sample_receiver_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_sample_receiver_midi.py"
    $<TARGET_FILE:blofeld-original-dsp-sample-receiver-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-sample-receiver-midi.trace")
  set_tests_properties(original_dsp_sample_receiver_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_test(NAME original_dsp_sample_timer COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_sample_timer.py"
    $<TARGET_FILE:blofeld-original-dsp-sample-receiver-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-sample-timer.trace")
  set_tests_properties(original_dsp_sample_timer PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_test(NAME original_dsp_sample_refill COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_sample_refill.py"
    $<TARGET_FILE:blofeld-original-dsp-sample-receiver-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-sample-refill.trace")
  set_tests_properties(original_dsp_sample_refill PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 960)
  add_executable(blofeld-original-dsp-filters-test test/original_dsp_filters_test.cpp
    test/dsp_filter_model.cpp test/dsp_filter_fixture.cpp)
  target_link_libraries(blofeld-original-dsp-filters-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_filters COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_filters.py"
    $<TARGET_FILE:blofeld-original-dsp-filters-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-filters.trace")
  set_tests_properties(original_dsp_filters PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-waveforms-test test/original_dsp_waveforms_test.cpp)
  target_link_libraries(blofeld-original-dsp-waveforms-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_waveforms COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_waveforms.py"
    $<TARGET_FILE:blofeld-original-dsp-waveforms-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-waveforms.trace")
  set_tests_properties(original_dsp_waveforms PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-dsp-envelopes-test test/original_dsp_envelopes_test.cpp)
  target_link_libraries(blofeld-original-dsp-envelopes-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_envelopes COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_envelopes.py"
    $<TARGET_FILE:blofeld-original-dsp-envelopes-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-envelopes.trace")
  set_tests_properties(original_dsp_envelopes PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_library(blofeld_voice_fixture STATIC test/dsp_voice_fixture.cpp test/dsp_voice_master.cpp
    test/dsp_voice_pcm.cpp test/dsp_voice_repeat.cpp test/dsp_voice_dax.cpp)
  target_link_libraries(blofeld_voice_fixture PUBLIC blofeld_dsp blofeld_dsp_native blofeld_serial_pcm)
  add_executable(blofeld-original-dsp-voice-test test/original_dsp_voice_test.cpp)
  target_link_libraries(blofeld-original-dsp-voice-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_voice COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice.trace")
  set_tests_properties(original_dsp_voice PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-voice-master-test test/original_dsp_voice_master_test.cpp)
  target_link_libraries(blofeld-original-dsp-voice-master-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_voice_master COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice_master.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-master-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice-master.trace")
  set_tests_properties(original_dsp_voice_master PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-voice-pcm-test test/original_dsp_voice_pcm_test.cpp)
  target_link_libraries(blofeld-original-dsp-voice-pcm-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_voice_pcm COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice_pcm.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-pcm-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice-pcm.trace")
  set_tests_properties(original_dsp_voice_pcm PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-voice-host-test test/original_dsp_voice_host_test.cpp)
  target_link_libraries(blofeld-original-dsp-voice-host-test PRIVATE blofeld_voice_fixture blofeld_audio_runtime)
  add_test(NAME original_dsp_voice_host COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice_host.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-host-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice-host")
  set_tests_properties(original_dsp_voice_host PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_test(NAME original_dsp_voice_midi_host COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice_host.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-host-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice-midi-host" midi)
  set_tests_properties(original_dsp_voice_midi_host PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_test(NAME original_dsp_repeat_host COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice_host.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-host-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-repeat-host" stream)
  set_tests_properties(original_dsp_repeat_host PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 360
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_executable(blofeld-original-dsp-voice-midi-test test/original_dsp_voice_midi_test.cpp)
  target_link_libraries(blofeld-original-dsp-voice-midi-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_voice_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice_midi.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-midi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice-midi.trace")
  set_tests_properties(original_dsp_voice_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-foreground-test test/original_dsp_foreground_test.cpp)
  target_link_libraries(blofeld-original-dsp-foreground-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_foreground COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_foreground.py"
    $<TARGET_FILE:blofeld-original-dsp-foreground-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-foreground.trace")
  set_tests_properties(original_dsp_foreground PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-program-guard-test test/original_dsp_program_guard_test.cpp)
  target_link_libraries(blofeld-original-dsp-program-guard-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_program_guard COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_program_guard.py"
    $<TARGET_FILE:blofeld-original-dsp-program-guard-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-program-guard")
  set_tests_properties(original_dsp_program_guard PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-dsp-repeat-test test/original_dsp_repeat_test.cpp)
  target_link_libraries(blofeld-original-dsp-repeat-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_repeat COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_repeat.py"
    $<TARGET_FILE:blofeld-original-dsp-repeat-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-repeat.trace")
  set_tests_properties(original_dsp_repeat PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_executable(blofeld-original-dsp-dual-output-test test/original_dsp_dual_output_test.cpp)
  target_link_libraries(blofeld-original-dsp-dual-output-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_dual_output COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_dual_output.py"
    $<TARGET_FILE:blofeld-original-dsp-dual-output-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-dual-output.trace")
  set_tests_properties(original_dsp_dual_output PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_executable(blofeld-original-dsp-idle-midi-test test/original_dsp_idle_midi_test.cpp)
  target_link_libraries(blofeld-original-dsp-idle-midi-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_idle_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_idle_midi.py"
    $<TARGET_FILE:blofeld-original-dsp-idle-midi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-idle-midi.trace")
  set_tests_properties(original_dsp_idle_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_executable(blofeld-original-dsp-release-ring-test test/original_dsp_release_ring_test.cpp)
  target_link_libraries(blofeld-original-dsp-release-ring-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_release_ring COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_release_ring.py"
    $<TARGET_FILE:blofeld-original-dsp-release-ring-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-release-ring.trace")
  set_tests_properties(original_dsp_release_ring PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_executable(blofeld-original-dsp-sustain-ring-test test/original_dsp_sustain_ring_test.cpp)
  target_link_libraries(blofeld-original-dsp-sustain-ring-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_sustain_ring COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_sustain_ring.py"
    $<TARGET_FILE:blofeld-original-dsp-sustain-ring-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-sustain-ring.trace")
  set_tests_properties(original_dsp_sustain_ring PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_executable(blofeld-original-dsp-ring-audio-test test/original_dsp_ring_audio_test.cpp)
  target_link_libraries(blofeld-original-dsp-ring-audio-test PRIVATE
    blofeld_voice_fixture blofeld_audio_runtime)
  add_test(NAME original_dsp_ring_audio COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_ring_audio.py"
    $<TARGET_FILE:blofeld-original-dsp-ring-audio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-ring-audio")
  set_tests_properties(original_dsp_ring_audio PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_test(NAME original_dsp_ring_midi_audio COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_ring_audio.py"
    $<TARGET_FILE:blofeld-original-dsp-ring-audio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-ring-midi-audio" midi)
  set_tests_properties(original_dsp_ring_midi_audio PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_test(NAME original_dsp_ring_release_audio COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_ring_audio.py"
    $<TARGET_FILE:blofeld-original-dsp-ring-audio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-ring-release-audio" release)
  set_tests_properties(original_dsp_ring_release_audio PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_test(NAME original_dsp_ring_sustain_audio COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_ring_audio.py"
    $<TARGET_FILE:blofeld-original-dsp-ring-audio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-ring-sustain-audio" sustain)
  set_tests_properties(original_dsp_ring_sustain_audio PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_executable(blofeld-original-dsp-voice-audio-test test/original_dsp_voice_audio_test.cpp)
  target_link_libraries(blofeld-original-dsp-voice-audio-test PRIVATE
    blofeld_voice_fixture blofeld_audio_runtime)
  add_test(NAME original_dsp_voice_audio COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice_audio.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-audio-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice-audio")
  set_tests_properties(original_dsp_voice_audio PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600
    ENVIRONMENT "BLOFELD_TEST_SANITIZER=${BLOFELD_SANITIZER}")
  add_executable(blofeld-original-dsp-drive-voice-test test/original_dsp_drive_voice_test.cpp)
  target_link_libraries(blofeld-original-dsp-drive-voice-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_drive_voice COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_drive_voice.py"
    $<TARGET_FILE:blofeld-original-dsp-drive-voice-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-drive-voice.trace")
  set_tests_properties(original_dsp_drive_voice PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-peripheral-init-test test/original_dsp_peripheral_init.cpp)
  target_link_libraries(blofeld-original-dsp-peripheral-init-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_peripheral_init COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_peripheral_init.py"
    $<TARGET_FILE:blofeld-original-dsp-peripheral-init-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-peripheral-init.trace")
  set_tests_properties(original_dsp_peripheral_init PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-dsp-audio-dma-test
    test/original_dsp_audio_dma_test.cpp test/dsp_audio_dma_fixture.cpp)
  target_link_libraries(blofeld-original-dsp-audio-dma-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware blofeld_serial_pcm)
  add_test(NAME original_dsp_audio_dma COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_audio_dma.py"
    $<TARGET_FILE:blofeld-original-dsp-audio-dma-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-audio-dma.trace")
  set_tests_properties(original_dsp_audio_dma PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-dsp-scheduler-test test/original_dsp_scheduler_test.cpp)
  target_link_libraries(blofeld-original-dsp-scheduler-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_scheduler COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_scheduler.py"
    $<TARGET_FILE:blofeld-original-dsp-scheduler-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-scheduler.trace")
  set_tests_properties(original_dsp_scheduler PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 250 RUN_SERIAL TRUE)
  add_executable(blofeld-original-update-transport-test test/original_update_transport_test.cpp)
  target_link_libraries(blofeld-original-update-transport-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_update_transport COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_update_transport.py"
    $<TARGET_FILE:blofeld-original-update-transport-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-update-transport.trace")
  set_tests_properties(original_update_transport PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-keyboard-transport-test test/original_keyboard_transport_test.cpp)
  target_link_libraries(blofeld-original-keyboard-transport-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_keyboard_transport COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_transport.py"
    $<TARGET_FILE:blofeld-original-keyboard-transport-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-transport.trace")
  set_tests_properties(original_keyboard_transport PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-keyboard-short-test
    test/original_keyboard_short_test.cpp test/keyboard_short_fixture.cpp)
  target_link_libraries(blofeld-original-keyboard-short-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_keyboard_short COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_short.py"
    $<TARGET_FILE:blofeld-original-keyboard-short-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-short.trace")
  set_tests_properties(original_keyboard_short PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-keyboard-initial-test
    test/original_keyboard_initial_test.cpp test/keyboard_initial_fixture.cpp
    test/keyboard_initial_display.cpp test/keyboard_initial_replay.cpp)
  target_link_libraries(blofeld-original-keyboard-initial-test PRIVATE
    blofeld_mcu_dsp blofeld_dsp_native blofeld_lcd_fixture blofeld_lcd_export)
  add_test(NAME original_keyboard_initial COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_initial.py"
    $<TARGET_FILE:blofeld-original-keyboard-initial-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-initial.trace")
  set_tests_properties(original_keyboard_initial PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-keyboard-receiver-test test/original_keyboard_receiver_test.cpp)
  target_link_libraries(blofeld-original-keyboard-receiver-test PRIVATE blofeld_core)
  add_test(NAME original_keyboard_receiver COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_receiver.py"
    $<TARGET_FILE:blofeld-original-keyboard-receiver-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-receiver.trace")
  set_tests_properties(original_keyboard_receiver PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-original-keyboard-handoff-test test/original_keyboard_handoff_test.cpp)
  target_link_libraries(blofeld-original-keyboard-handoff-test PRIVATE
    blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME original_keyboard_handoff COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_keyboard_handoff.py"
    $<TARGET_FILE:blofeld-original-keyboard-handoff-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/BlofeldKBC14.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-keyboard-handoff.trace")
  set_tests_properties(original_keyboard_handoff PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-original-dsp-download-test test/original_dsp_download_test.cpp)
  target_link_libraries(blofeld-original-dsp-download-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_download COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_download.py"
    $<TARGET_FILE:blofeld-original-dsp-download-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-download.trace")
  set_tests_properties(original_dsp_download PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 90)
  add_executable(blofeld-original-dsp-request-queue-test test/original_dsp_request_queue_test.cpp)
  target_link_libraries(blofeld-original-dsp-request-queue-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_request_queue COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_request_queue.py"
    $<TARGET_FILE:blofeld-original-dsp-request-queue-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-request-queue.trace")
  set_tests_properties(original_dsp_request_queue PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 250)
  add_executable(blofeld-original-dsp-workspace-test
    test/original_dsp_workspace_test.cpp test/original_dsp_workspace_state.cpp)
  target_link_libraries(blofeld-original-dsp-workspace-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_workspace COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_workspace.py"
    $<TARGET_FILE:blofeld-original-dsp-workspace-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-workspace.trace")
  set_tests_properties(original_dsp_workspace PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_test(NAME original_dsp_request_readiness COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_request_readiness.py"
    $<TARGET_FILE:blofeld-original-dsp-request-queue-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-request-readiness.trace")
  set_tests_properties(original_dsp_request_readiness PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 130)
  add_executable(blofeld-dsp-vsl-test test/dsp_vsl_test.cpp)
  target_link_libraries(blofeld-dsp-vsl-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_vsl COMMAND blofeld-dsp-vsl-test)
  add_executable(blofeld-dsp-zero-loop-test test/dsp_zero_loop_test.cpp)
  target_link_libraries(blofeld-dsp-zero-loop-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_zero_loops COMMAND blofeld-dsp-zero-loop-test)
  add_executable(blofeld-dsp-corpus-test test/dsp_corpus_test.cpp)
  target_link_libraries(blofeld-dsp-corpus-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_hardware_corpus COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/dsp_hardware_corpus.py"
    $<TARGET_FILE:blofeld-dsp-corpus-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/dsp56300-tools/tools/difftest/goldens"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/dsp-corpus.json"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/dsp-hardware-corpus")
  set_tests_properties(dsp_hardware_corpus PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME dsp_operand_regressions COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/dsp_operand_regressions.py"
    $<TARGET_FILE:blofeld-dsp-corpus-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/dsp56300-tools/tools/difftest/goldens"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/dsp-operand-regressions.json"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/dsp-operand-regressions")
  set_tests_properties(dsp_operand_regressions PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 250)
  add_test(NAME dsp_corpus_bounds COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/dsp_corpus_bounds.py"
    $<TARGET_FILE:blofeld-dsp-corpus-test>)
  add_executable(blofeld-dsp-boot-test test/original_dsp_boot_test.cpp
    test/original_dsp_reference.cpp)
  target_link_libraries(blofeld-dsp-boot-test PRIVATE blofeld_dsp)
  add_test(NAME original_dsp_boot COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_boot.py"
    $<TARGET_FILE:blofeld-dsp-boot-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-boot.trace")
  set_tests_properties(original_dsp_boot PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-original-dsp-midi-test test/original_dsp_midi_test.cpp)
  target_link_libraries(blofeld-original-dsp-midi-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-dsp-midi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-midi.trace")
  set_tests_properties(original_dsp_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-dsp-shi-receive-test test/original_dsp_shi_receive_test.cpp)
  target_link_libraries(blofeld-original-dsp-shi-receive-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_shi_receive COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_shi_receive.py"
    $<TARGET_FILE:blofeld-original-dsp-shi-receive-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-shi-receive.trace")
  set_tests_properties(original_dsp_shi_receive PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-original-dsp-note-test test/original_dsp_note_test.cpp)
  target_link_libraries(blofeld-original-dsp-note-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_note COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_note.py"
    $<TARGET_FILE:blofeld-original-dsp-note-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-note.trace")
  set_tests_properties(original_dsp_note PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_test(NAME original_dsp_note_release COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_note_release.py"
    $<TARGET_FILE:blofeld-original-dsp-note-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-note-release.trace")
  set_tests_properties(original_dsp_note_release PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_test(NAME original_dsp_sustain COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_sustain.py"
    $<TARGET_FILE:blofeld-original-dsp-note-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-sustain.trace")
  set_tests_properties(original_dsp_sustain PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
  add_executable(blofeld-original-dsp-wire-test test/original_dsp_wire_test.cpp)
  target_link_libraries(blofeld-original-dsp-wire-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  foreach(scope IN ITEMS bytes pitch checkpoint download)
    add_test(NAME original_dsp_wire_${scope} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_wire.py"
      $<TARGET_FILE:blofeld-original-dsp-wire-test>
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-wire-${scope}.trace" ${scope})
    set_tests_properties(original_dsp_wire_${scope} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  endforeach()
  add_executable(blofeld-original-dsp-clocked-wire-test test/original_dsp_clocked_wire_test.cpp)
  target_link_libraries(blofeld-original-dsp-clocked-wire-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  foreach(scope IN ITEMS bytes pitch checkpoint)
    add_test(NAME original_dsp_clocked_wire_${scope} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_clocked_wire.py"
      $<TARGET_FILE:blofeld-original-dsp-clocked-wire-test>
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-clocked-wire-${scope}.trace" ${scope})
    set_tests_properties(original_dsp_clocked_wire_${scope} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  endforeach()
  set_tests_properties(original_dsp_clocked_wire_bytes PROPERTIES TIMEOUT 300)
  add_executable(blofeld-original-sample-preparation-test test/original_sample_preparation_test.cpp)
  target_link_libraries(blofeld-original-sample-preparation-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_sample_preparation COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_sample_preparation.py"
    $<TARGET_FILE:blofeld-original-sample-preparation-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/library/Standard_Factory_Sample_Content_V1.0_for_License_SL_as_MID_File.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-sample-preparation.trace")
  set_tests_properties(original_sample_preparation PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
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
  add_executable(blofeld-coupled-boot-test test/original_coupled_boot_test.cpp
    test/coupled_boot_fixture.cpp)
  target_link_libraries(blofeld-coupled-boot-test PRIVATE blofeld_core blofeld_dsp)
  add_test(NAME original_coupled_boot COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_boot.py"
    $<TARGET_FILE:blofeld-coupled-boot-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-coupled-boot.trace")
  set_tests_properties(original_coupled_boot PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-scheduled-coupling-test test/original_scheduled_coupling_test.cpp
    test/scheduled_coupling_fixture.cpp test/scheduled_frontier_fixture.cpp
    test/original_owned_clock_fixture.cpp)
  target_link_libraries(blofeld-scheduled-coupling-test PRIVATE blofeld_core blofeld_dsp blofeld_mcu_dsp
    blofeld_dsp_native blofeld_lcd_fixture)
  add_executable(blofeld-original-reset-start-test test/original_reset_start_test.cpp
    test/scheduled_coupling_fixture.cpp test/original_owned_clock_fixture.cpp)
  target_link_libraries(blofeld-original-reset-start-test PRIVATE blofeld_core blofeld_dsp blofeld_mcu_dsp
    blofeld_dsp_native blofeld_lcd_fixture)
  add_test(NAME original_reset_start COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_reset_start.py"
    $<TARGET_FILE:blofeld-original-reset-start-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-reset-start.trace")
  set_tests_properties(original_reset_start PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME original_shi_routes COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_shi_routes.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-shi-routes")
  set_tests_properties(original_shi_routes PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
  add_test(NAME original_checkpoint_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_checkpoint_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-checkpoint-cli.trace")
  set_tests_properties(original_checkpoint_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_test(NAME original_reference_cli COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_reference_cli.py"
    $<TARGET_FILE:blofeld-headless>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-reference-cli.trace")
  set_tests_properties(original_reference_cli PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 240)
  add_test(NAME original_scheduled_coupling COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_scheduled_coupling.py"
    $<TARGET_FILE:blofeld-scheduled-coupling-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-scheduled-coupling.trace")
  set_tests_properties(original_scheduled_coupling PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-shi-test test/dsp_shi_test.cpp)
  target_link_libraries(blofeld-shi-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_shi COMMAND blofeld-shi-test)
  add_executable(blofeld-mcu-shi-test test/mcu_shi_test.cpp)
  target_link_libraries(blofeld-mcu-shi-test PRIVATE blofeld_core blofeld_dsp_peripherals)
  add_test(NAME mcu_shi COMMAND blofeld-mcu-shi-test)
  add_executable(blofeld-core-test test/core_test.cpp)
  target_link_libraries(blofeld-core-test PRIVATE blofeld_core)
  add_test(NAME native_core COMMAND blofeld-core-test)
  add_executable(blofeld-mcu-checkpoint-test test/mcu_checkpoint_test.cpp
    test/mcu_checkpoint_peripherals.cpp test/mcu_checkpoint_flash.cpp)
  target_link_libraries(blofeld-mcu-checkpoint-test PRIVATE blofeld_core)
  add_test(NAME mcu_checkpoint COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/mcu_checkpoint.py"
    $<TARGET_FILE:blofeld-mcu-checkpoint-test>)
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
  target_link_libraries(blofeld-gpio-pad-source-test PRIVATE blofeld_mcu blofeld_dsp_peripherals)
  add_test(NAME gpio_pad_sources COMMAND blofeld-gpio-pad-source-test)
  add_executable(blofeld-gpio-pad-checkpoint-test test/gpio_pad_checkpoint_test.cpp)
  target_link_libraries(blofeld-gpio-pad-checkpoint-test PRIVATE
    blofeld_core blofeld_dsp blofeld_dsp_native)
  add_test(NAME gpio_pad_checkpoints COMMAND blofeld-gpio-pad-checkpoint-test)
  add_executable(blofeld-mcu-dsp-reset-test test/mcu_dsp_reset_test.cpp)
  target_link_libraries(blofeld-mcu-dsp-reset-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME mcu_dsp_reset COMMAND blofeld-mcu-dsp-reset-test)
  add_executable(blofeld-mcu-reset-test test/mcu_reset_test.cpp)
  target_link_libraries(blofeld-mcu-reset-test PRIVATE blofeld_core blofeld_mcu_dsp blofeld_dsp blofeld_dsp_native)
  add_test(NAME mcu_reset COMMAND blofeld-mcu-reset-test)
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
  add_executable(blofeld-dsp-run-clock-test test/dsp_run_clock_test.cpp)
  target_link_libraries(blofeld-dsp-run-clock-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_run_clock COMMAND blofeld-dsp-run-clock-test)
  add_executable(blofeld-dsp-agu-interlocks-test test/dsp_agu_interlocks_test.cpp)
  target_link_libraries(blofeld-dsp-agu-interlocks-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_agu_interlocks COMMAND blofeld-dsp-agu-interlocks-test)
  add_executable(blofeld-dsp-interlock-test test/dsp_interlock_clock_test.cpp test/dsp_interlock_state.cpp)
  target_include_directories(blofeld-dsp-interlock-test PRIVATE src)
  target_link_libraries(blofeld-dsp-interlock-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_interlock_clock COMMAND blofeld-dsp-interlock-test)
  add_test(NAME dsp_interlock_reference COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/dsp_interlock_clock.py"
    $<TARGET_FILE:blofeld-dsp-interlock-test>
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/dsp-interlock-reference")
  add_test(NAME dsp_status_pipeline COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/dsp_status_pipeline.py"
    $<TARGET_FILE:blofeld-dsp-interlock-test>
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/dsp-status-pipeline")
  add_executable(blofeld-dsp-reset-epoch-test test/dsp_reset_epoch_test.cpp)
  target_link_libraries(blofeld-dsp-reset-epoch-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_reset_epoch COMMAND blofeld-dsp-reset-epoch-test)
  add_executable(blofeld-dsp-reset-interlock-test test/dsp_reset_interlock_test.cpp)
  target_link_libraries(blofeld-dsp-reset-interlock-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_reset_interlocks COMMAND blofeld-dsp-reset-interlock-test)
  add_executable(blofeld-dsp-hardware-reset-test test/dsp_hardware_reset_test.cpp)
  target_link_libraries(blofeld-dsp-hardware-reset-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_hardware_reset COMMAND blofeld-dsp-hardware-reset-test)
  add_executable(blofeld-dsp-irq-clock-test test/dsp_irq_clock_test.cpp)
  target_link_libraries(blofeld-dsp-irq-clock-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_irq_clock COMMAND blofeld-dsp-irq-clock-test)
  set_tests_properties(dsp_irq_clock PROPERTIES TIMEOUT 120)
  add_executable(blofeld-dsp-irq-admission-test test/dsp_irq_admission_test.cpp)
  target_link_libraries(blofeld-dsp-irq-admission-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_irq_admission COMMAND blofeld-dsp-irq-admission-test)
  set_tests_properties(dsp_irq_admission PROPERTIES TIMEOUT 120)
  add_executable(blofeld-dsp-scheduled-run-test test/dsp_scheduled_run_test.cpp)
  target_link_libraries(blofeld-dsp-scheduled-run-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_scheduled_run COMMAND blofeld-dsp-scheduled-run-test)
  add_executable(blofeld-joint-dsp-hold-test test/joint_dsp_hold_test.cpp)
  target_link_libraries(blofeld-joint-dsp-hold-test PRIVATE blofeld_mcu_dsp blofeld_dsp_native)
  add_test(NAME joint_dsp_hold COMMAND blofeld-joint-dsp-hold-test)
  add_executable(blofeld-original-dsp-clock-test test/original_dsp_clock_test.cpp)
  target_link_libraries(blofeld-original-dsp-clock-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_clock.py"
    $<TARGET_FILE:blofeld-original-dsp-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-clock.trace")
  set_tests_properties(original_dsp_clock PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME original_dsp_agu_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_agu_clock.py"
    $<TARGET_FILE:blofeld-original-dsp-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-agu-clock.trace")
  set_tests_properties(original_dsp_agu_clock PROPERTIES SKIP_RETURN_CODE 77)

  add_test(NAME original_dsp_reset_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_reset_clock.py"
    $<TARGET_FILE:blofeld-original-dsp-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-reset-clock.trace")
  set_tests_properties(original_dsp_reset_clock PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
  add_executable(blofeld-clocked-midi-test test/mcu_clocked_midi_test.cpp)
  target_link_libraries(blofeld-clocked-midi-test PRIVATE blofeld_mcu_midi)
  add_test(NAME mcu_clocked_midi COMMAND blofeld-clocked-midi-test)
  add_executable(blofeld-timer-test test/mcu_timer_test.cpp)
  target_link_libraries(blofeld-timer-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_timer COMMAND blofeld-timer-test)
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
  get_property(blofeld_build_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
  set(blofeld_hardened_executables "")
  foreach(target IN LISTS blofeld_build_targets)
    get_target_property(kind ${target} TYPE)
    if(kind STREQUAL "EXECUTABLE")
      string(APPEND blofeld_hardened_executables "$<TARGET_FILE:${target}>\n")
    endif()
  endforeach()
  file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/hardening-executables.txt"
    CONTENT "${blofeld_hardened_executables}")
  set(blofeld_hardening_options)
  if(BLOFELD_USE_LIBSTDCXX)
    list(APPEND blofeld_hardening_options --libstdcxx)
  endif()
  if(BLOFELD_CF_PROTECTION)
    list(APPEND blofeld_hardening_options --cf-protection)
  endif()
  add_test(NAME build_hardening COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/build_hardening.py"
    --source "${CMAKE_CURRENT_SOURCE_DIR}"
    --database "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json"
    --binary-list "${CMAKE_CURRENT_BINARY_DIR}/generated/hardening-executables.txt"
    --readelf "${BLOFELD_LLVM_READELF}" --compiler "${CMAKE_C_COMPILER}"
    --native zlibstatic=$<TARGET_FILE:zlibstatic>
    --native blofeld_sleigh=$<TARGET_FILE:blofeld_sleigh>
    --native blofeld_dsp_native=$<TARGET_FILE:blofeld_dsp_native>
    --native asmjit=$<TARGET_FILE:asmjit>
    --native sndfile=$<TARGET_FILE:sndfile>
    --native blofeld_pulse=$<TARGET_FILE:blofeld_pulse>
    --native blofeld_pulse_common=$<TARGET_FILE:blofeld_pulse_common>
    ${blofeld_hardening_options})
endif()

install(TARGETS blofeld-headless RUNTIME DESTINATION bin)
install(PROGRAMS tools/blofeld-display DESTINATION bin COMPONENT Display)
install(FILES gui/__init__.py gui/frame.py gui/watch.py gui/display.py
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
  DEPENDS blofeld-hcs08-prefetch-store-test
    blofeld-original-hcs08-spi-word-test blofeld-original-hcs08-spi-move-test
    blofeld-original-startup-identity-test blofeld-original-identity-main-test blofeld-original-device-identity-test blofeld-virtual-device-state-test
    blofeld-device-identity-test blofeld-hcs08-instruction-reservation-test blofeld-hcs08-irq-bus-test blofeld-hcs08-irq-test
    blofeld-original-hcs08-spi-irq-test blofeld-hcs08-spi-test blofeld-original-hcs08-spi-test blofeld-hcs08-register-knowledge-test
    blofeld-original-hcs08-clock-startup-test blofeld-hcs08-gpio-test blofeld-original-hcs08-gpio-test format-check
    blofeld-hcs08-peripheral-reset-test blofeld-hcs08-icg-test blofeld-hcs08-icg-native-test blofeld-hcs08-flash-clock-test
    blofeld-original-hcs08-reset-entry-test blofeld-hcs08-reset-security-test blofeld-hcs08-background-reset-test blofeld-original-keyboard-chain-test
    blofeld-original-keyboard-reset-test blofeld-hcs08-reset-test blofeld-hcs08-run-clock-test blofeld-original-keyboard-target-test
    blofeld-hcs08-bdc-wire-test blofeld-original-keyboard-bdc-wire-test blofeld-hcs08-bdc-test blofeld-original-hcs08-bdc-test
    blofeld-hcs08-flash-test blofeld-hcs08-flash-recovery-test blofeld-original-hcs08-flash-test blofeld-original-hcs08-test
    blofeld-hcs08-core-test blofeld-hcs08-arithmetic-test blofeld-gpio-output-test blofeld-gpio-unknown-driver-test
    blofeld-gpio-pad-source-test blofeld-original-runtime-clocks-test blofeld-original-mcu-request-release-test blofeld-original-gpio-pair-test
    blofeld-original-gpio-pair-checkpoint-test blofeld-original-gpio-sequence-test blofeld-original-gpio-parameter-test blofeld-original-gpio-sound-test
    blofeld-original-gpio-notification-test blofeld-original-gpio-usb-notification-test blofeld-original-gpio-sysex-test blofeld-original-gpio-global-wait-test
    blofeld-headless blofeld-dsp-dispatch-cache-test blofeld-host-audio-test blofeld-pcm-queue-test
    blofeld-audio-runtime-test blofeld-esai-test blofeld-esai-clock-test blofeld-dma-test
    blofeld-gpio-pad-checkpoint-test blofeld-hcs08-prefetch-scanner-test blofeld-hcs08-prefetch-irq-test blofeld-hcs08-prefetch-jump-test
    blofeld-hcs08-prefetch-return-test blofeld-hcs08-spi-run-clock-test blofeld-original-hcs08-spi-run-clock-test blofeld-usbip-runtime-test
    blofeld-usbip-suspend-test blofeld-hcs08-prefetch-peripheral-test blofeld-hcs08-prefetch-branch-test blofeld-hcs08-prefetch-data-test
    blofeld-hcs08-prefetch-test blofeld-original-hcs08-prefetch-test blofeld-hcs08-spi-clock-test blofeld-original-hcs08-calibration-test
    blofeld-original-hcs08-pitch-event-test blofeld-original-hcs08-adc-clock-test blofeld-hcs08-irq-enable-delay-test blofeld-hcs08-adc-flags-test
    blofeld-hcs08-adc-test blofeld-original-hcs08-adc-test blofeld-hcs08-tpm-test blofeld-hcs08-cop-test
    blofeld-original-hcs08-timer-wait-test blofeld-original-hcs08-icg-test blofeld-nand-operation-clock-test blofeld-original-keyboard-reset-sync-test
    blofeld-hcs08-instruction-timing-test blofeld-hcs08-divide-test blofeld-original-reset-start-test blofeld-joint-dsp-hold-test
    blofeld-dsp-hardware-reset-test blofeld-dsp-reset-interlock-test blofeld-dsp-timer-events-test blofeld-dsp-memory-storage-test
    blofeld-dsp-irqa-test blofeld-dsp-dma-interrupt-test blofeld-original-dsp-gpio-input-test
    blofeld-original-dsp-gpio-request-test
    blofeld-original-gpio-midi-test
    blofeld-original-dsp-download-test
    blofeld-original-update-transport-test
    blofeld-original-keyboard-transport-test
    blofeld-original-keyboard-short-test
    blofeld-original-keyboard-initial-test
    blofeld-original-keyboard-receiver-test
    blofeld-original-keyboard-handoff-test
    blofeld-esai-interrupt-test
    blofeld-pin-routing-test
    blofeld-sample-library-test blofeld-hc12-ccr-test blofeld-original-usb-midi-test
    blofeld-original-usb-queue-test
    blofeld-original-midi-forward-test
    blofeld-original-midi-dispatch-test
    blofeld-original-usb-peer-test
    blofeld-sample-wave-test
    blofeld-usbip-test blofeld-usb-host-test blofeld-usb-host-native-test
    blofeld-usb-device-test blofeld-usbip-management-test blofeld-usb-device-native-test
    blofeld-usbip-session-test blofeld-usbip-tcp-test blofeld-usbip-tcp-native-test
    blofeld-usbip-frame-timeout-test
    blofeld-usbip-output-timeout-test
    blofeld-usbip-session-boundaries-test
    blofeld-serial-pcm-test
    blofeld-host-midi-test
    blofeld-midi-timed-test
    blofeld-original-host-midi-test
    blofeld-pcm-transport-test
    blofeld-bank-test
    blofeld-repeat-test
    blofeld-reference-access-test
    blofeld-program-latch-test blofeld-prefetch-queue-test blofeld-prefetch-condition-test blofeld-reference-cycle-test
    blofeld-fetch-run-test blofeld-retirement-test blofeld-address-phase-test blofeld-writeback-test
    blofeld-overlap-test blofeld-pipeline-test blofeld-memory-pipeline-test blofeld-parallel-memory-test
    blofeld-transfer-pipeline-test blofeld-reference-dma-admission-test
    blofeld-dax-test
    blofeld-reference-checkpoint-test blofeld-reference-rom-fixture
    blofeld-reference-original-midi-rom
    blofeld-dsp-completion-observer-test blofeld-dsp-native-access-test blofeld-dsp-external-events-test
    blofeld-core-test blofeld-code-cache-test blofeld-mcu-test
    blofeld-mcu-dsp-schedule-test
    blofeld-mcu-dsp-wait-test
    blofeld-hc12-cycle-test
    blofeld-hc12-cycle-extended-test
    blofeld-hc12-compare-stack-test
    blofeld-hc12-indexed-test
    blofeld-hc12-minmax-test
    blofeld-hc12-flash-cycles-test
    blofeld-mcu-cycle-test
    blofeld-mcu-run-clock-test
    blofeld-dsp-run-clock-test blofeld-dsp-reset-epoch-test blofeld-dsp-interlock-test blofeld-dsp-divide-test
    blofeld-dsp-fast-normalization-test blofeld-dsp-agu-interlocks-test
    blofeld-dsp-irq-clock-test
    blofeld-dsp-irq-admission-test
    blofeld-dsp-scheduled-run-test
    blofeld-original-dsp-clock-test
    blofeld-original-dsp-midi-test
    blofeld-original-dsp-note-test
    blofeld-original-dsp-scheduler-test
    blofeld-original-dsp-request-queue-test
    blofeld-original-dsp-workspace-test
    blofeld-original-dsp-peripheral-init-test
    blofeld-original-dsp-audio-dma-test
    blofeld-original-dsp-shi-receive-test
    blofeld-original-dsp-wire-test
    blofeld-original-dsp-clocked-wire-test
    blofeld-mcu-clock-checkpoint-test
    blofeld-mcu-wait-clock-test
    blofeld-original-cycle-boot-test
    blofeld-original-cycle-application-test
    blofeld-original-run-clock-test
    blofeld-original-owned-clock-test
    blofeld-midi-serial-test blofeld-mcu-midi-test blofeld-clocked-midi-test
    blofeld-nand-read-test blofeld-nand-write-test blofeld-nand-power-test blofeld-nand-gpio-test
    blofeld-nand-read-timing-test blofeld-original-nand-read-timing-test
    blofeld-original-nand-pack-test
    blofeld-original-nand-records-test
    blofeld-nand-write-timing-test blofeld-original-nand-write-timing-test
    blofeld-nand-read-setup-test blofeld-original-nand-read-setup-test
    blofeld-nand-image-test
    blofeld-nand-store-test
    blofeld-nand-gpio-write-test
    blofeld-original-nand-write-test
    blofeld-original-update-validation-test
    blofeld-original-update-forward-test
    blofeld-original-update-diagnostic-test
    blofeld-original-update-coupled-test
    blofeld-mcu-checkpoint-test
    blofeld-shi-test blofeld-mcu-shi-test blofeld-dsp-test blofeld-dsp-boot-test blofeld-dsp-pll-test
    blofeld-dsp-instruction-test blofeld-dsp-memory-map-test blofeld-dsp-checkpoint-test blofeld-dsp-cycle-cache-test
    blofeld-dsp-gpio-test
    blofeld-dsp-timers-test
    blofeld-dsp-timer-updates-test
    blofeld-dsp-capture-test
    blofeld-dsp-interrupt-test
    blofeld-dsp-norm-test
    blofeld-dsp-transfer-test blofeld-dsp-shifted-subtract-test blofeld-dsp-vsl-test
    blofeld-dsp-saturation-test
    blofeld-dsp-power-test
    blofeld-dsp-cmpm-test
    blofeld-original-dsp-filters-test
    blofeld-original-dsp-master-output-test
    blofeld-original-dsp-sample-test
    blofeld-original-dsp-sample-receiver-test
    blofeld-original-dsp-waveforms-test
    blofeld-original-dsp-envelopes-test
    blofeld-original-dsp-voice-test
    blofeld-original-dsp-voice-master-test
    blofeld-original-dsp-voice-pcm-test
    blofeld-original-dsp-voice-host-test
    blofeld-original-dsp-voice-audio-test
    blofeld-original-dsp-voice-midi-test
    blofeld-original-dsp-foreground-test
    blofeld-original-dsp-program-guard-test
    blofeld-original-dsp-repeat-test
    blofeld-original-dsp-dual-output-test
    blofeld-original-dsp-idle-midi-test
    blofeld-original-dsp-release-ring-test blofeld-original-dsp-sustain-ring-test
    blofeld-original-dsp-ring-audio-test
    blofeld-original-dsp-drive-voice-test
    blofeld-dsp-corpus-test blofeld-coupled-boot-test blofeld-dsp-zero-loop-test
    blofeld-original-sample-preparation-test
    blofeld-scheduled-coupling-test
    blofeld-mcu-cop-test blofeld-mcu-reset-test blofeld-mcu-dsp-reset-test
    blofeld-clock-test blofeld-sci-test blofeld-sci-reference-test
    blofeld-timer-test blofeld-interrupt-test blofeld-rti-test
    blofeld-memory-usb-test blofeld-usb-event-test blofeld-usb-isr-test blofeld-boot-test
    blofeld-mcu-sender-test
    blofeld-flash-config-test blofeld-original-flash-config-test
    blofeld-flash-test blofeld-flash-stop-test blofeld-original-flash-stop-test
    blofeld-flash-backdoor-test
    blofeld-original-flash-test
    blofeld-original-flash-restart-test
    blofeld-original-parallel-update-test
    blofeld-original-lcd-helper-test blofeld-original-lcd-image-test
    blofeld-original-mcu-scan-test
    blofeld-usb-ique-control-test
    blofeld-usb-endpoint-reset-test
    blofeld-original-usb-reset-test
    blofeld-original-application-test
    blofeld-wai-test
    blofeld-st7565-test blofeld-lcd-image-test blofeld-lcd-export-test
    blofeld-st7565-parallel-test
    blofeld-flash-image-test blofeld-flash-file-worker
  USES_TERMINAL)
endif()

if(BLOFELD_FUZZING)
  add_executable(dsp-instruction-fuzz test/dsp_instruction_fuzz.cpp)
  target_link_libraries(dsp-instruction-fuzz PRIVATE blofeld_dsp_instruction_fixture)
  target_link_options(dsp-instruction-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(nand-image-fuzz test/nand_image_fuzz.cpp)
  target_link_libraries(nand-image-fuzz PRIVATE blofeld_nand ZLIB::ZLIB)
  target_link_options(nand-image-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(nand-read-fuzz test/nand_read_fuzz.cpp)
  target_link_libraries(nand-read-fuzz PRIVATE blofeld_nand)
  target_link_options(nand-read-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(st7565-parallel-fuzz test/st7565_parallel_fuzz.cpp)
  target_link_libraries(st7565-parallel-fuzz PRIVATE blofeld_display)
  target_link_options(st7565-parallel-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(st7565-fuzz test/st7565_fuzz.cpp)
  target_link_libraries(st7565-fuzz PRIVATE blofeld_display)
  target_link_options(st7565-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(flash-image-fuzz test/flash_image_fuzz.cpp)
  target_link_libraries(flash-image-fuzz PRIVATE blofeld_storage ZLIB::ZLIB)
  target_link_options(flash-image-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(firmware-fuzz test/firmware_fuzz.cpp)
  target_link_libraries(firmware-fuzz PRIVATE blofeld_firmware)
  target_link_options(firmware-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(mcu-model-fuzz test/mcu_model_fuzz.cpp)
  target_link_libraries(mcu-model-fuzz PRIVATE blofeld_mcu)
  target_link_options(mcu-model-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(dsp-shi-fuzz test/dsp_shi_fuzz.cpp)
  target_link_libraries(dsp-shi-fuzz PRIVATE blofeld_dsp_peripherals)
  target_link_options(dsp-shi-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(dsp-model-fuzz test/dsp_model_fuzz.cpp)
  target_link_libraries(dsp-model-fuzz PRIVATE blofeld_dsp_peripherals)
  target_link_options(dsp-model-fuzz PRIVATE -fsanitize=fuzzer)
endif()
