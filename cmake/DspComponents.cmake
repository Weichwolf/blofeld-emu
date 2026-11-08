# SPDX-License-Identifier: GPL-3.0-only
# Board memory, peripherals and native DSP execution.
add_library(blofeld_firmware src/firmware/firmware.cpp src/firmware/archive.cpp src/firmware/midi_file.cpp src/common/diagnostic.cpp src/common/bus.cpp)
target_include_directories(blofeld_firmware PUBLIC src)
target_link_libraries(blofeld_firmware PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_gpio_pads src/runtime/gpio_pads.cpp)
target_include_directories(blofeld_gpio_pads PUBLIC src)
target_link_libraries(blofeld_gpio_pads PUBLIC blofeld_firmware)
add_library(blofeld_dsp_peripherals src/dsp/dsp_shi.cpp src/dsp/dsp_shi_spi.cpp src/dsp/dsp_bus.cpp src/dsp/dsp_io_map.cpp src/dsp/dsp_pll.cpp
  src/dsp/dsp_esai.cpp src/dsp/dsp_esai_transmit.cpp src/dsp/dsp_esai_clock.cpp src/dsp/dsp_esai_events.cpp src/dsp/dsp_bus_esai.cpp
  src/dsp/dsp_dma.cpp src/dsp/dsp_dma_address.cpp src/dsp/dsp_dma_cycle.cpp src/dsp/dsp_bus_dma.cpp
  src/dsp/dsp_dax.cpp src/dsp/dsp_dax_clock.cpp src/dsp/dsp_bus_dax.cpp
  src/dsp/dsp_gpio.cpp src/dsp/dsp_timers.cpp src/dsp/dsp_timer_clock.cpp src/dsp/dsp_timer_capture.cpp
  src/dsp/dsp_timer_boundary.cpp src/dsp/dsp_dma_quiescent.cpp src/dsp/dsp_interrupts.cpp
  src/dsp/dsp_native_region_bus.cpp src/dsp/dsp_native_region_clock.cpp
  src/dsp/dsp_run_clock.cpp src/dsp/dsp_run_reset.cpp)
target_include_directories(blofeld_dsp_peripherals PUBLIC src)
target_link_libraries(blofeld_dsp_peripherals PUBLIC blofeld_firmware blofeld_gpio_pads)
if(BUILD_TESTING)
  add_executable(blofeld-dsp-clock-cache-test test/dsp_clock_cache_test.cpp)
  target_link_libraries(blofeld-dsp-clock-cache-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_clock_cache COMMAND blofeld-dsp-clock-cache-test)
  add_executable(blofeld-dsp-esai-copy-test test/dsp_esai_copy_test.cpp)
  target_link_libraries(blofeld-dsp-esai-copy-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_esai_copy COMMAND blofeld-dsp-esai-copy-test)
  add_executable(blofeld-dsp-timer-drive-test test/dsp_timer_drive_test.cpp)
  target_link_libraries(blofeld-dsp-timer-drive-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_timer_drive COMMAND blofeld-dsp-timer-drive-test)
endif()
add_library(blofeld_dsp_peripheral_calendar src/dsp/dsp_peripheral_calendar.cpp)
target_include_directories(blofeld_dsp_peripheral_calendar PUBLIC src)
target_link_libraries(blofeld_dsp_peripheral_calendar PUBLIC blofeld_firmware)
add_library(blofeld_dsp_machine STATIC EXCLUDE_FROM_ALL src/dsp/dsp_io.cpp
  src/dsp/dsp_interrupt_admission.cpp src/dsp/dsp_machine.cpp src/dsp/dsp_machine_run.cpp)
target_include_directories(blofeld_dsp_machine PUBLIC src)
target_link_libraries(blofeld_dsp_machine PUBLIC blofeld_dsp_peripherals dsp56300::core)
add_library(blofeld_dsp_execution STATIC EXCLUDE_FROM_ALL
  src/dsp/dsp_execution.cpp src/dsp/dsp_execution_run.cpp)
target_include_directories(blofeld_dsp_execution PUBLIC src)
target_link_libraries(blofeld_dsp_execution PUBLIC blofeld_dsp_machine)
add_library(blofeld_mcu_dsp_runtime STATIC EXCLUDE_FROM_ALL src/runtime/mcu_dsp_runtime.cpp
  src/runtime/mcu_dsp_runtime_peripheral.cpp src/runtime/mcu_dsp_runtime_regions.cpp)
target_link_libraries(blofeld_mcu_dsp_runtime PUBLIC blofeld_dsp_execution blofeld_dsp_peripheral_calendar)
target_link_libraries(blofeld_mcu_dsp_runtime PUBLIC blofeld_core)
if(BUILD_TESTING)
  add_executable(blofeld-native-region-clock-test test/native_region_clock_test.cpp)
  target_link_libraries(blofeld-native-region-clock-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME native_region_clock COMMAND blofeld-native-region-clock-test)

  add_executable(blofeld-dsp-timer-boundary-test test/dsp_timer_boundary_test.cpp)
  target_link_libraries(blofeld-dsp-timer-boundary-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_timer_boundary COMMAND blofeld-dsp-timer-boundary-test)
  add_executable(blofeld-dsp-dma-quiescent-test test/dsp_dma_quiescent_test.cpp)
  target_link_libraries(blofeld-dsp-dma-quiescent-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_dma_quiescent COMMAND blofeld-dsp-dma-quiescent-test)
  add_executable(blofeld-dsp-program-view-test test/dsp_program_view_test.cpp)
  target_link_libraries(blofeld-dsp-program-view-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_program_view COMMAND blofeld-dsp-program-view-test)
  add_executable(blofeld-dsp-data-windows-test test/dsp_data_windows_test.cpp)
  target_link_libraries(blofeld-dsp-data-windows-test PRIVATE blofeld_dsp_machine)
  target_include_directories(blofeld-dsp-data-windows-test PRIVATE "${BLOFELD_DSP56300_ROOT}/test/support")
  add_test(NAME dsp_data_windows COMMAND blofeld-dsp-data-windows-test)
  add_executable(blofeld-dsp-native-bus-test test/dsp_native_bus_test.cpp)
  target_link_libraries(blofeld-dsp-native-bus-test PRIVATE blofeld_dsp_machine)
  target_include_directories(blofeld-dsp-native-bus-test PRIVATE "${BLOFELD_DSP56300_ROOT}/test/support")
  add_test(NAME dsp_native_bus COMMAND blofeld-dsp-native-bus-test)

  add_executable(blofeld-dsp-shifted-subtract-test test/dsp_shifted_subtract_test.cpp)
  target_link_libraries(blofeld-dsp-shifted-subtract-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_subl COMMAND blofeld-dsp-shifted-subtract-test --left)
  add_test(NAME dsp_subr COMMAND blofeld-dsp-shifted-subtract-test --right)
  add_test(NAME dsp_subr_host_overflow COMMAND blofeld-dsp-shifted-subtract-test --host-overflow)
  set_tests_properties(dsp_subl dsp_subr dsp_subr_host_overflow PROPERTIES TIMEOUT 120)
  add_executable(blofeld-dsp-vsl-test test/dsp_vsl_test.cpp)
  target_link_libraries(blofeld-dsp-vsl-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_vsl COMMAND blofeld-dsp-vsl-test)
  set_tests_properties(dsp_vsl PROPERTIES TIMEOUT 120)
  add_executable(blofeld-dsp-cmpm-test test/dsp_cmpm_test.cpp)
  target_link_libraries(blofeld-dsp-cmpm-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_cmpm_accumulators COMMAND blofeld-dsp-cmpm-test --full)
  add_test(NAME dsp_cmpm_words COMMAND blofeld-dsp-cmpm-test --word)
  add_test(NAME dsp_cmpm_minimum COMMAND blofeld-dsp-cmpm-test --minimum)
  set_tests_properties(dsp_cmpm_accumulators dsp_cmpm_words dsp_cmpm_minimum PROPERTIES TIMEOUT 120)
  add_executable(blofeld-dsp-saturation-test test/dsp_saturation_test.cpp
    test/dsp_saturation_oracle.cpp test/dsp_saturation_cases.cpp)
  target_link_libraries(blofeld-dsp-saturation-test PRIVATE blofeld_dma_fixture)
  foreach(scope IN ITEMS regressions arithmetic increment decrement multiply accumulate rounding exceptions mac-host-overflow overflow-flags)
    add_test(NAME dsp_saturation_${scope} COMMAND blofeld-dsp-saturation-test ${scope})
    set_tests_properties(dsp_saturation_${scope} PROPERTIES TIMEOUT 120)
  endforeach()

  add_executable(blofeld-dsp-transfer-test test/dsp_transfer_test.cpp)
  target_link_libraries(blofeld-dsp-transfer-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_transfers COMMAND blofeld-dsp-transfer-test)
  add_executable(blofeld-dsp-norm-test test/dsp_norm_test.cpp)
  target_link_libraries(blofeld-dsp-norm-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_norm COMMAND blofeld-dsp-norm-test)

  add_executable(blofeld-repeat-test test/dsp_repeat_test.cpp)
  target_link_libraries(blofeld-repeat-test PRIVATE blofeld_dsp_machine)
  add_test(NAME dsp_repeat_restrictions COMMAND blofeld-repeat-test)
  add_executable(blofeld-esai-test test/dsp_esai_test.cpp test/dsp_esai_native.cpp)
  target_link_libraries(blofeld-esai-test PRIVATE blofeld_dsp_machine)
  add_test(NAME dsp_esai COMMAND blofeld-esai-test)
  add_library(blofeld_dma_fixture STATIC test/dma_native_fixture.cpp)
  target_link_libraries(blofeld_dma_fixture PUBLIC blofeld_dsp_machine)
  add_executable(blofeld-modifier-agu-test test/dsp_modifier_agu_test.cpp)
  target_link_libraries(blofeld-modifier-agu-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_modifier_agu COMMAND blofeld-modifier-agu-test)
  set_tests_properties(dsp_modifier_agu PROPERTIES TIMEOUT 120)

  add_executable(blofeld-original-dsp-midi-test test/original_dsp_midi_test.cpp)
  target_link_libraries(blofeld-original-dsp-midi-test PRIVATE
    blofeld_dma_fixture)
  add_test(NAME original_dsp_midi COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_firmware.py"
    $<TARGET_FILE:blofeld-original-dsp-midi-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-midi.trace")
  set_tests_properties(original_dsp_midi PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  add_executable(blofeld-original-dsp-shi-receive-test test/original_dsp_shi_receive_test.cpp)
  target_link_libraries(blofeld-original-dsp-shi-receive-test PRIVATE
    blofeld_dma_fixture)
  add_test(NAME original_dsp_shi_receive COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_shi_receive.py"
    $<TARGET_FILE:blofeld-original-dsp-shi-receive-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-shi-receive.trace")
  set_tests_properties(original_dsp_shi_receive PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
  add_executable(blofeld-original-dsp-note-test test/original_dsp_note_test.cpp)
  target_link_libraries(blofeld-original-dsp-note-test PRIVATE
    blofeld_dma_fixture)
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
    blofeld_dma_fixture)
  foreach(scope IN ITEMS bytes pitch checkpoint download)
    add_test(NAME original_dsp_wire_${scope} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_wire.py"
      $<TARGET_FILE:blofeld-original-dsp-wire-test>
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-wire-${scope}.trace" ${scope})
    set_tests_properties(original_dsp_wire_${scope} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  endforeach()

  add_executable(blofeld-dsp-interrupt-test test/dsp_interrupt_test.cpp
    test/dsp_interrupt_fixture.cpp test/dsp_interrupt_restore.cpp)
  target_link_libraries(blofeld-dsp-interrupt-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_interrupts COMMAND blofeld-dsp-interrupt-test)

  add_executable(blofeld-dsp-irqa-test test/dsp_irqa_test.cpp)
  target_link_libraries(blofeld-dsp-irqa-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_irqa COMMAND blofeld-dsp-irqa-test)
  add_library(blofeld_timer_fixture STATIC test/timer_native_fixture.cpp)
  target_link_libraries(blofeld_timer_fixture PUBLIC blofeld_dma_fixture)
  add_executable(blofeld-dsp-capture-test test/dsp_capture_test.cpp)
  target_link_libraries(blofeld-dsp-capture-test PRIVATE blofeld_timer_fixture)
  add_test(NAME dsp_capture COMMAND blofeld-dsp-capture-test)
  add_executable(blofeld-dsp-timer-events-test test/dsp_timer_events_test.cpp)
  target_link_libraries(blofeld-dsp-timer-events-test PRIVATE blofeld_timer_fixture)
  add_test(NAME dsp_timer_events COMMAND blofeld-dsp-timer-events-test)
  add_executable(blofeld-dsp-dma-interrupt-test test/dsp_dma_interrupt_test.cpp
    test/dsp_dma_interrupt_native.cpp)
  target_link_libraries(blofeld-dsp-dma-interrupt-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_dma_interrupts COMMAND blofeld-dsp-dma-interrupt-test)

  add_executable(blofeld-dax-test test/dsp_dax_test.cpp)
  target_link_libraries(blofeld-dax-test PRIVATE blofeld_dsp_peripherals)
  add_test(NAME dsp_dax COMMAND blofeld-dax-test)

  add_executable(blofeld-dsp-adapter-clock-test test/dsp_adapter_clock_test.cpp)
  target_link_libraries(blofeld-dsp-adapter-clock-test PRIVATE blofeld_dsp_machine)
  add_test(NAME dsp_adapter_clock COMMAND blofeld-dsp-adapter-clock-test)

  add_executable(blofeld-dsp-adapter-ram-bit-test test/dsp_adapter_ram_bit_test.cpp)
  target_link_libraries(blofeld-dsp-adapter-ram-bit-test PRIVATE blofeld_dsp_machine)
  add_test(NAME dsp_adapter_ram_bit COMMAND blofeld-dsp-adapter-ram-bit-test)
  set_tests_properties(dsp_adapter_ram_bit PROPERTIES TIMEOUT 1200)
endif()

if(BUILD_TESTING)
  add_executable(blofeld-peripheral-calendar-test test/runtime/peripheral_calendar_test.cpp)
  target_link_libraries(blofeld-peripheral-calendar-test PRIVATE blofeld_dsp_peripheral_calendar)
  add_test(NAME peripheral_calendar COMMAND blofeld-peripheral-calendar-test)

  add_executable(blofeld-dsp-execution-test test/runtime/dsp_execution_test.cpp)
  target_include_directories(blofeld-dsp-execution-test PRIVATE "${BLOFELD_DSP56300_ROOT}/test/support")
  target_link_libraries(blofeld-dsp-execution-test PRIVATE blofeld_dsp_execution)
  add_test(NAME dsp_execution COMMAND blofeld-dsp-execution-test)
endif()
