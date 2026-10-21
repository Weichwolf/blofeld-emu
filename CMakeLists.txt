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
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

include(cmake/Hc12Spec.cmake)
add_library(blofeld_firmware src/firmware.cpp src/archive.cpp src/midi_file.cpp src/diagnostic.cpp src/bus.cpp)
target_include_directories(blofeld_firmware PUBLIC include)
target_link_libraries(blofeld_firmware PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_private_files src/private_file.cpp)
target_include_directories(blofeld_private_files PUBLIC include)
target_link_libraries(blofeld_private_files PUBLIC blofeld_firmware)
add_library(blofeld_sample_library src/sample_library.cpp src/sample_xml_reader.cpp
  src/sample_xml_values.cpp src/sample_xml_codec.cpp src/sample_xml_file.cpp src/sample_client.cpp
  src/sample_wave.cpp src/sample_wave_format.cpp src/sample_wave_metadata.cpp src/sample_pitch.cpp
  src/sample_wave_client.cpp)
target_include_directories(blofeld_sample_library PUBLIC include)
target_link_libraries(blofeld_sample_library PUBLIC blofeld_private_files)
add_library(blofeld_storage src/flash_image.cpp src/flash_file.cpp)
target_include_directories(blofeld_storage PUBLIC include)
target_link_libraries(blofeld_storage PUBLIC blofeld_firmware PRIVATE blofeld_private_files ZLIB::ZLIB)
add_library(blofeld_nand src/nand.cpp src/nand_write.cpp src/nand_array.cpp src/nand_state.cpp
  src/nand_image.cpp src/nand_image_codec.cpp)
target_include_directories(blofeld_nand PUBLIC include)
target_link_libraries(blofeld_nand PUBLIC blofeld_firmware PRIVATE ZLIB::ZLIB)
add_library(blofeld_nand_store src/nand_file.cpp src/nand_store.cpp)
target_include_directories(blofeld_nand_store PUBLIC include)
target_link_libraries(blofeld_nand_store PUBLIC blofeld_nand PRIVATE blofeld_private_files)
add_library(blofeld_mcu src/mcu_bus.cpp src/mcu_bus_clone.cpp src/mcu_bus_usb.cpp src/mcu_code_cache.cpp src/mcu_memory.cpp src/mcu_gpio.cpp
  src/mcu_flash_config.cpp src/mcu_flash.cpp src/mcu_flash_backdoor.cpp src/mcu_flash_memory.cpp src/mcu_bus_flash.cpp
  src/mcu_clock.cpp src/mcu_run_clock.cpp src/mcu_sci.cpp src/mcu_timer.cpp src/mcu_interrupts.cpp
  src/mcu_usb.cpp src/mcu_usb_io.cpp src/mcu_usb_status.cpp src/mcu_usb_packets.cpp
  src/mcu_usb_control.cpp
  src/mcu_usb_ique_control.cpp src/mcu_usb_reset.cpp)
target_include_directories(blofeld_mcu PUBLIC include)
target_link_libraries(blofeld_mcu PUBLIC blofeld_firmware blofeld_storage)
add_library(blofeld_display src/st7565.cpp src/st7565_parallel.cpp)
target_include_directories(blofeld_display PUBLIC include)
target_link_libraries(blofeld_display PUBLIC blofeld_firmware)
add_library(blofeld_core src/hc12.cpp src/hc12_decimal.cpp src/mcu_machine.cpp)
add_library(blofeld_hc12_cycles src/hc12_cycle_plan.cpp src/hc12_cycle_page2.cpp src/hc12_cycle_bus.cpp)
target_include_directories(blofeld_hc12_cycles PUBLIC include)
target_link_libraries(blofeld_hc12_cycles PUBLIC blofeld_firmware)
add_library(blofeld_midi_serial src/midi_serial.cpp)
target_include_directories(blofeld_midi_serial PUBLIC include)
target_link_libraries(blofeld_midi_serial PUBLIC blofeld_firmware)
add_library(blofeld_midi_timed src/midi_timed.cpp)
target_include_directories(blofeld_midi_timed PUBLIC include)
target_link_libraries(blofeld_midi_timed PUBLIC blofeld_midi_serial)
add_library(blofeld_serial_pcm src/serial_pcm.cpp)
target_include_directories(blofeld_serial_pcm PUBLIC include)
target_link_libraries(blofeld_serial_pcm PUBLIC blofeld_firmware)
add_library(blofeld_pcm_transport src/pcm_recording.cpp src/pcm_wave.cpp src/pcm_wave_file.cpp)
target_include_directories(blofeld_pcm_transport PUBLIC include)
target_link_libraries(blofeld_pcm_transport PUBLIC blofeld_serial_pcm PRIVATE blofeld_private_files)
add_library(blofeld_mcu_midi src/mcu_midi.cpp src/mcu_clocked_midi.cpp)
target_include_directories(blofeld_mcu_midi PUBLIC include)
target_link_libraries(blofeld_mcu_midi PUBLIC blofeld_mcu blofeld_midi_serial)
add_library(blofeld_dsp_peripherals src/dsp_shi.cpp src/dsp_shi_spi.cpp src/dsp_bus.cpp src/dsp_pll.cpp
  src/dsp_esai.cpp src/dsp_esai_transmit.cpp src/dsp_esai_clock.cpp src/dsp_bus_esai.cpp
  src/dsp_dma.cpp src/dsp_dma_address.cpp src/dsp_dma_cycle.cpp src/dsp_bus_dma.cpp
  src/dsp_dax.cpp src/dsp_dax_clock.cpp src/dsp_bus_dax.cpp
  src/dsp_gpio.cpp src/dsp_timers.cpp src/dsp_timer_clock.cpp src/dsp_timer_capture.cpp src/dsp_interrupts.cpp
  src/dsp_run_clock.cpp)
target_include_directories(blofeld_dsp_peripherals PUBLIC include)
target_link_libraries(blofeld_dsp_peripherals PUBLIC blofeld_firmware)
add_library(blofeld_dsp src/dsp_core.cpp src/dsp_native_access.cpp src/dsp_reference_execution.cpp
  src/dsp_interrupt_timing.cpp
  src/dsp_machine.cpp src/dsp_scheduled_run.cpp
  src/dsp_scheduled_events.cpp)
target_include_directories(blofeld_dsp PUBLIC include)
target_link_libraries(blofeld_dsp PUBLIC blofeld_dsp_peripherals PRIVATE blofeld_dsp_native)
if(BUILD_TESTING OR BLOFELD_FUZZING)
  add_library(blofeld_dsp_instruction_fixture test/dsp_instruction_fixture.cpp)
  target_link_libraries(blofeld_dsp_instruction_fixture PUBLIC blofeld_dsp PRIVATE blofeld_dsp_native)
endif()
add_dependencies(blofeld_core blofeld_hc12_spec)
target_include_directories(blofeld_core PUBLIC include)
target_link_libraries(blofeld_core PUBLIC blofeld_mcu PRIVATE blofeld_sleigh blofeld_hc12_cycles)
target_compile_definitions(blofeld_core PUBLIC
  BLOFELD_DEFAULT_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
add_library(blofeld_mcu_dsp src/mcu_dsp_schedule.cpp)
target_include_directories(blofeld_mcu_dsp PUBLIC include)
target_link_libraries(blofeld_mcu_dsp PUBLIC blofeld_core blofeld_dsp)
add_library(blofeld_host_midi src/host_midi.cpp)
target_include_directories(blofeld_host_midi PUBLIC include)
target_link_libraries(blofeld_host_midi PUBLIC blofeld_quality)
add_executable(blofeld-headless src/main.cpp src/reference_config.cpp
  src/reference_board.cpp src/reference_wiring.cpp src/reference_returns.cpp src/reference_checkpoint.cpp
  src/reference_dsp_inputs.cpp src/reference_dsp_clocks.cpp src/reference_pcm.cpp
  src/reference_midi.cpp src/reference_dsp_midi.cpp src/host_midi_runner.cpp)
target_link_libraries(blofeld-headless PRIVATE blofeld_mcu_dsp blofeld_private_files blofeld_pcm_transport
  blofeld_mcu_midi blofeld_host_midi blofeld_midi_timed blofeld_sample_library)

if(BUILD_TESTING)
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
  add_executable(blofeld-dax-test test/dsp_dax_test.cpp test/dsp_dax_native.cpp test/dsp_dax_dma.cpp)
  target_link_libraries(blofeld-dax-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_dax COMMAND blofeld-dax-test)
  add_executable(blofeld-reference-checkpoint-test test/reference_checkpoint.cpp
    src/reference_config.cpp src/reference_board.cpp src/reference_wiring.cpp
    src/reference_returns.cpp src/reference_checkpoint.cpp src/reference_dsp_inputs.cpp src/reference_dsp_clocks.cpp src/reference_pcm.cpp
    src/reference_midi.cpp src/reference_dsp_midi.cpp)
  target_include_directories(blofeld-reference-checkpoint-test PRIVATE src)
  target_link_libraries(blofeld-reference-checkpoint-test PRIVATE
    blofeld_mcu_dsp blofeld_private_files blofeld_dsp_native blofeld_pcm_transport blofeld_mcu_midi
    blofeld_midi_timed)
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
    --mode "${BLOFELD_SANITIZER}"
    --nm "${BLOFELD_LLVM_NM}" --readelf "${BLOFELD_LLVM_READELF}"
    --native zlibstatic=$<TARGET_FILE:zlibstatic>
    --native blofeld_sleigh=$<TARGET_FILE:blofeld_sleigh>
    --native blofeld_dsp_native=$<TARGET_FILE:blofeld_dsp_native>
    --native asmjit=$<TARGET_FILE:asmjit>)
  add_executable(blofeld-midi-serial-test test/midi_serial_test.cpp)
  target_link_libraries(blofeld-midi-serial-test PRIVATE blofeld_midi_serial)
  add_test(NAME midi_serial COMMAND blofeld-midi-serial-test)
  add_executable(blofeld-mcu-midi-test test/mcu_midi_test.cpp)
  target_link_libraries(blofeld-mcu-midi-test PRIVATE blofeld_mcu_midi)
  add_test(NAME mcu_midi COMMAND blofeld-mcu-midi-test)
  add_executable(blofeld-nand-read-test test/nand_read_test.cpp)
  target_link_libraries(blofeld-nand-read-test PRIVATE blofeld_nand)
  add_test(NAME nand_read COMMAND blofeld-nand-read-test)
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
  add_executable(blofeld-flash-backdoor-test test/flash_backdoor_test.cpp test/flash_backdoor_native.cpp)
  target_link_libraries(blofeld-flash-backdoor-test PRIVATE blofeld_core)
  add_test(NAME flash_backdoor COMMAND blofeld-flash-backdoor-test)
  add_executable(blofeld-flash-test test/mcu_flash_test.cpp)
  target_link_libraries(blofeld-flash-test PRIVATE blofeld_core)
  add_test(NAME flash_commands COMMAND blofeld-flash-test)
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
  target_link_libraries(blofeld-original-usb-reset-test PRIVATE blofeld_application_fixture)
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
  add_executable(blofeld-dsp-interrupt-test test/dsp_interrupt_test.cpp)
  target_link_libraries(blofeld-dsp-interrupt-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME dsp_interrupts COMMAND blofeld-dsp-interrupt-test)
  add_executable(blofeld-dsp-gpio-test test/dsp_gpio_test.cpp)
  target_link_libraries(blofeld-dsp-gpio-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_gpio COMMAND blofeld-dsp-gpio-test)
  add_executable(blofeld-dsp-pll-test test/dsp_pll_test.cpp)
  target_link_libraries(blofeld-dsp-pll-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_pll COMMAND blofeld-dsp-pll-test)
  add_executable(blofeld-dsp-test test/dsp_core_test.cpp)
  target_link_libraries(blofeld-dsp-test PRIVATE blofeld_dsp blofeld_dsp_native)
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
  foreach(scope IN ITEMS regressions arithmetic multiply accumulate rounding exceptions mac-host-overflow)
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
  add_library(blofeld_voice_fixture STATIC test/dsp_voice_fixture.cpp)
  target_link_libraries(blofeld_voice_fixture PUBLIC blofeld_dsp blofeld_dsp_native)
  add_executable(blofeld-original-dsp-voice-test test/original_dsp_voice_test.cpp)
  target_link_libraries(blofeld-original-dsp-voice-test PRIVATE blofeld_voice_fixture)
  add_test(NAME original_dsp_voice COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_voice.py"
    $<TARGET_FILE:blofeld-original-dsp-voice-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-voice.trace")
  set_tests_properties(original_dsp_voice PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
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
  add_executable(blofeld-original-dsp-scheduler-test test/original_dsp_scheduler_test.cpp)
  target_link_libraries(blofeld-original-dsp-scheduler-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_scheduler COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_scheduler.py"
    $<TARGET_FILE:blofeld-original-dsp-scheduler-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-scheduler.trace")
  set_tests_properties(original_dsp_scheduler PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 250)
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
  add_executable(blofeld-original-dsp-note-test test/original_dsp_note_test.cpp)
  target_link_libraries(blofeld-original-dsp-note-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  add_test(NAME original_dsp_note COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_note.py"
    $<TARGET_FILE:blofeld-original-dsp-note-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-note.trace")
  set_tests_properties(original_dsp_note PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-dsp-wire-test test/original_dsp_wire_test.cpp)
  target_link_libraries(blofeld-original-dsp-wire-test PRIVATE
    blofeld_dsp blofeld_dsp_native blofeld_firmware)
  foreach(scope IN ITEMS bytes pitch checkpoint)
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
  add_executable(blofeld-original-dsp-clock-test test/original_dsp_clock_test.cpp)
  target_link_libraries(blofeld-original-dsp-clock-test PRIVATE blofeld_dsp blofeld_dsp_native)
  add_test(NAME original_dsp_clock COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_clock.py"
    $<TARGET_FILE:blofeld-original-dsp-clock-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-clock.trace")
  set_tests_properties(original_dsp_clock PROPERTIES SKIP_RETURN_CODE 77)
  add_executable(blofeld-clocked-midi-test test/mcu_clocked_midi_test.cpp)
  target_link_libraries(blofeld-clocked-midi-test PRIVATE blofeld_mcu_midi)
  add_test(NAME mcu_clocked_midi COMMAND blofeld-clocked-midi-test)
  add_executable(blofeld-timer-test test/mcu_timer_test.cpp)
  target_link_libraries(blofeld-timer-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_timer COMMAND blofeld-timer-test)
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
    ${blofeld_hardening_options})
endif()

install(TARGETS blofeld-headless RUNTIME DESTINATION bin)
install(FILES LICENSE THIRD_PARTY.txt DESTINATION share/doc/blofeld-emu)
install(DIRECTORY LICENSES/ DESTINATION share/doc/blofeld-emu/licenses)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla" DESTINATION share/blofeld-emu)

file(GLOB_RECURSE BLOFELD_FORMAT_SOURCES CONFIGURE_DEPENDS
  "include/*.hpp" "src/*.hpp" "src/*.cpp" "test/*.cpp" "test/*.hpp")
add_custom_target(format-check
  COMMAND "${BLOFELD_CLANG_FORMAT}" --dry-run --Werror ${BLOFELD_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
add_custom_target(quality
  COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
  DEPENDS format-check blofeld-headless blofeld-esai-test blofeld-esai-clock-test blofeld-dma-test
    blofeld-sample-library-test
    blofeld-sample-wave-test
    blofeld-serial-pcm-test
    blofeld-host-midi-test
    blofeld-midi-timed-test
    blofeld-original-host-midi-test
    blofeld-pcm-transport-test
    blofeld-bank-test
    blofeld-repeat-test
    blofeld-reference-access-test
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
    blofeld-hc12-flash-cycles-test
    blofeld-mcu-cycle-test
    blofeld-mcu-run-clock-test
    blofeld-dsp-run-clock-test
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
    blofeld-nand-image-test
    blofeld-nand-store-test
    blofeld-nand-gpio-write-test
    blofeld-original-nand-write-test
    blofeld-mcu-checkpoint-test
    blofeld-shi-test blofeld-mcu-shi-test blofeld-dsp-test blofeld-dsp-boot-test blofeld-dsp-pll-test
    blofeld-dsp-instruction-test
    blofeld-dsp-checkpoint-test
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
    blofeld-original-dsp-waveforms-test
    blofeld-original-dsp-envelopes-test
    blofeld-original-dsp-voice-test
    blofeld-original-dsp-drive-voice-test
    blofeld-dsp-corpus-test blofeld-coupled-boot-test blofeld-dsp-zero-loop-test
    blofeld-original-sample-preparation-test
    blofeld-scheduled-coupling-test
    blofeld-clock-test blofeld-sci-test blofeld-sci-reference-test
    blofeld-timer-test blofeld-interrupt-test blofeld-rti-test
    blofeld-memory-usb-test blofeld-usb-event-test blofeld-usb-isr-test blofeld-boot-test
    blofeld-mcu-sender-test
    blofeld-flash-config-test blofeld-original-flash-config-test
    blofeld-flash-test
    blofeld-flash-backdoor-test
    blofeld-original-flash-test
    blofeld-original-flash-restart-test
    blofeld-original-parallel-update-test
    blofeld-original-lcd-helper-test
    blofeld-original-mcu-scan-test
    blofeld-usb-ique-control-test
    blofeld-usb-endpoint-reset-test
    blofeld-original-usb-reset-test
    blofeld-original-application-test
    blofeld-wai-test
    blofeld-st7565-test
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
