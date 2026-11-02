# SPDX-License-Identifier: GPL-3.0-only
# Shared by the regular build and the independent-DSP integration build.
add_library(blofeld_firmware src/firmware/firmware.cpp src/firmware/archive.cpp src/firmware/midi_file.cpp src/common/diagnostic.cpp src/common/bus.cpp)
target_include_directories(blofeld_firmware PUBLIC src)
target_link_libraries(blofeld_firmware PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_gpio_pads src/board/gpio_pads.cpp)
target_include_directories(blofeld_gpio_pads PUBLIC src)
target_link_libraries(blofeld_gpio_pads PUBLIC blofeld_firmware)
add_library(blofeld_dsp_peripherals src/dsp/dsp_shi.cpp src/dsp/dsp_shi_spi.cpp src/dsp/dsp_bus.cpp src/dsp/dsp_io_map.cpp src/dsp/dsp_pll.cpp
  src/dsp/dsp_esai.cpp src/dsp/dsp_esai_transmit.cpp src/dsp/dsp_esai_clock.cpp src/dsp/dsp_bus_esai.cpp
  src/dsp/dsp_dma.cpp src/dsp/dsp_dma_address.cpp src/dsp/dsp_dma_cycle.cpp src/dsp/dsp_bus_dma.cpp
  src/dsp/dsp_dax.cpp src/dsp/dsp_dax_clock.cpp src/dsp/dsp_bus_dax.cpp
  src/dsp/dsp_gpio.cpp src/dsp/dsp_timers.cpp src/dsp/dsp_timer_clock.cpp src/dsp/dsp_timer_capture.cpp src/dsp/dsp_interrupts.cpp
  src/dsp/dsp_run_clock.cpp src/dsp/dsp_run_reset.cpp)
target_include_directories(blofeld_dsp_peripherals PUBLIC src)
target_link_libraries(blofeld_dsp_peripherals PUBLIC blofeld_firmware blofeld_gpio_pads)
add_library(blofeld_dsp_peripheral_calendar src/dsp/dsp_peripheral_calendar.cpp)
target_include_directories(blofeld_dsp_peripheral_calendar PUBLIC src)
target_link_libraries(blofeld_dsp_peripheral_calendar PUBLIC blofeld_firmware)
add_library(blofeld_own_dsp_io STATIC EXCLUDE_FROM_ALL src/dsp/own_dsp_io.cpp
  src/dsp/own_dsp_interrupts.cpp src/dsp/own_dsp_machine.cpp src/dsp/dsp_unclocked.cpp)
target_include_directories(blofeld_own_dsp_io PUBLIC src)
target_link_libraries(blofeld_own_dsp_io PUBLIC blofeld_dsp_peripherals dsp56300::core
  PRIVATE blofeld_dsp_peripheral_decode)
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
add_library(blofeld_own_dsp_fetch STATIC EXCLUDE_FROM_ALL src/dsp/own_dsp_fetch.cpp
  src/dsp/own_dsp_fetch_checkpoint.cpp src/dsp/own_dsp_timeline.cpp src/dsp/own_dsp_timeline_checkpoint.cpp
  src/dsp/dsp_program_fetch.cpp src/dsp/dsp_fetch_context.cpp src/dsp/dsp_loop_fetch.cpp
  src/dsp/dsp_loop_source.cpp)
target_include_directories(blofeld_own_dsp_fetch PUBLIC src)
target_link_libraries(blofeld_own_dsp_fetch PUBLIC blofeld_own_dsp_io blofeld_dsp_branch_decode
  blofeld_dsp_control_decode)
add_library(blofeld_mcu_dsp_runtime STATIC EXCLUDE_FROM_ALL src/board/mcu_dsp_runtime.cpp
  src/board/mcu_dsp_runtime_peripheral.cpp)
target_link_libraries(blofeld_mcu_dsp_runtime PUBLIC blofeld_own_dsp_fetch blofeld_dsp_peripheral_calendar)
if(NOT BLOFELD_OWN_DSP_ONLY)
  target_link_libraries(blofeld_mcu_dsp_runtime PUBLIC blofeld_core)
  if(BUILD_TESTING)
    add_executable(blofeld-mcu-dsp-runtime-test test/mcu_dsp_runtime_test.cpp)
    target_link_libraries(blofeld-mcu-dsp-runtime-test PRIVATE blofeld_mcu_dsp_runtime)
    add_test(NAME mcu_dsp_runtime COMMAND blofeld-mcu-dsp-runtime-test)
  endif()
endif()
if(BUILD_TESTING)
  add_library(blofeld_fetch_reset_fixture STATIC test/dsp_fetch_reset_fixture.cpp)
  target_link_libraries(blofeld_fetch_reset_fixture PUBLIC blofeld_own_dsp_fetch)
  add_executable(blofeld-timeline-ownership-test test/dsp_timeline_ownership_test.cpp)
  target_link_libraries(blofeld-timeline-ownership-test PRIVATE blofeld_fetch_reset_fixture)
  add_test(NAME dsp_timeline_ownership COMMAND blofeld-timeline-ownership-test)
  add_executable(blofeld-fetch-reset-test test/dsp_fetch_reset_test.cpp test/dsp_fetch_reset_stores.cpp)
  target_link_libraries(blofeld-fetch-reset-test PRIVATE blofeld_fetch_reset_fixture)
  add_test(NAME dsp_fetch_reset COMMAND blofeld-fetch-reset-test)
  add_executable(blofeld-fetch-run-test test/dsp_fetch_run_test.cpp test/dsp_fetch_run_guards.cpp)
  target_link_libraries(blofeld-fetch-run-test PRIVATE blofeld_own_dsp_fetch blofeld_dma_fixture)
  add_test(NAME dsp_fetch_run COMMAND blofeld-fetch-run-test)
  add_executable(blofeld-reference-cycle-test test/dsp_reference_cycle_test.cpp)
  target_link_libraries(blofeld-reference-cycle-test PRIVATE blofeld_own_dsp_fetch blofeld_dma_fixture)
  add_test(NAME dsp_reference_bus_cycle COMMAND blofeld-reference-cycle-test)
  add_executable(blofeld-prefetch-queue-test test/dsp_prefetch_queue_test.cpp)
  target_link_libraries(blofeld-prefetch-queue-test PRIVATE blofeld_own_dsp_fetch blofeld_dma_fixture)
  add_test(NAME dsp_prefetch_queue COMMAND blofeld-prefetch-queue-test)
  add_executable(blofeld-prefetch-condition-test test/dsp_prefetch_condition_test.cpp)
  target_link_libraries(blofeld-prefetch-condition-test PRIVATE blofeld_own_dsp_fetch blofeld_dma_fixture)
  add_test(NAME dsp_prefetch_conditions COMMAND blofeld-prefetch-condition-test)
  add_executable(blofeld-repeat-test test/dsp_repeat_test.cpp)
  target_link_libraries(blofeld-repeat-test PRIVATE blofeld_own_dsp_io)
  add_test(NAME dsp_repeat_restrictions COMMAND blofeld-repeat-test)
  add_executable(blofeld-esai-test test/dsp_esai_test.cpp test/dsp_esai_native.cpp)
  target_link_libraries(blofeld-esai-test PRIVATE blofeld_own_dsp_io)
  add_test(NAME dsp_esai COMMAND blofeld-esai-test)
  add_library(blofeld_dma_fixture STATIC test/dma_native_fixture.cpp)
  target_link_libraries(blofeld_dma_fixture PUBLIC blofeld_own_dsp_io)
  add_executable(blofeld-dsp-unclocked-test test/dsp_unclocked_test.cpp)
  target_link_libraries(blofeld-dsp-unclocked-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_unclocked COMMAND blofeld-dsp-unclocked-test)
  add_executable(blofeld-dsp-write-decoder-test test/dsp_write_decoder_test.cpp)
  target_link_libraries(blofeld-dsp-write-decoder-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_write_decoder COMMAND blofeld-dsp-write-decoder-test)
  set_tests_properties(dsp_write_decoder PROPERTIES TIMEOUT 60)
  add_executable(blofeld-dsp-reserved-write-decoder-test test/dsp_reserved_write_decoder_test.cpp)
  target_link_libraries(blofeld-dsp-reserved-write-decoder-test PRIVATE
    blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_reserved_write_decoder COMMAND blofeld-dsp-reserved-write-decoder-test)
  set_tests_properties(dsp_reserved_write_decoder PROPERTIES TIMEOUT 60)
  add_executable(blofeld-reserved-read-decoder-test test/dsp_reserved_read_decoder_test.cpp)
  target_link_libraries(blofeld-reserved-read-decoder-test PRIVATE
    blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_reserved_read_decoder COMMAND blofeld-reserved-read-decoder-test)
  set_tests_properties(dsp_reserved_read_decoder PROPERTIES TIMEOUT 60)
  add_executable(blofeld-dsp-gpio-write-decoder-test test/dsp_gpio_write_decoder_test.cpp)
  target_link_libraries(blofeld-dsp-gpio-write-decoder-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_gpio_write_decoder COMMAND blofeld-dsp-gpio-write-decoder-test)
  set_tests_properties(dsp_gpio_write_decoder PROPERTIES TIMEOUT 60)
  add_executable(blofeld-dsp-gpio-latch-test test/dsp_gpio_latch_test.cpp)
  target_link_libraries(blofeld-dsp-gpio-latch-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_gpio_latch COMMAND blofeld-dsp-gpio-latch-test)
  set_tests_properties(dsp_gpio_latch PROPERTIES TIMEOUT 60)
  add_executable(blofeld-dsp-timer-sample-test test/dsp_timer_sample_test.cpp)
  target_link_libraries(blofeld-dsp-timer-sample-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_timer_sample COMMAND blofeld-dsp-timer-sample-test)
  add_executable(blofeld-dsp-independent-io-fetch-test test/dsp_independent_io_fetch_test.cpp)
  target_link_libraries(blofeld-dsp-independent-io-fetch-test PRIVATE
    blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_independent_io_fetch COMMAND blofeld-dsp-independent-io-fetch-test)
  set_tests_properties(dsp_independent_io_fetch PROPERTIES TIMEOUT 60)
  add_executable(blofeld-dsp-ram-rmw-test test/dsp_ram_rmw_test.cpp)
  target_link_libraries(blofeld-dsp-ram-rmw-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_ram_rmw COMMAND blofeld-dsp-ram-rmw-test)
  set_tests_properties(dsp_ram_rmw PROPERTIES TIMEOUT 120)
  add_executable(blofeld-dsp-observed-test test/dsp_observed_test.cpp)
  target_link_libraries(blofeld-dsp-observed-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME dsp_observed COMMAND blofeld-dsp-observed-test)
  add_executable(blofeld-dsp-boot-test test/original_dsp_boot_test.cpp
    test/original_dsp_reference.cpp)
  target_link_libraries(blofeld-dsp-boot-test PRIVATE blofeld_dma_fixture)
  add_test(NAME original_dsp_boot COMMAND ${Python3_EXECUTABLE}
    "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_boot.py"
    $<TARGET_FILE:blofeld-dsp-boot-test>
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/nxp-spi-bootstrap.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-boot.trace")
  set_tests_properties(original_dsp_boot PROPERTIES SKIP_RETURN_CODE 77)
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
  add_executable(blofeld-original-dsp-clocked-wire-test test/original_dsp_clocked_wire_test.cpp)
  target_link_libraries(blofeld-original-dsp-clocked-wire-test PRIVATE
    blofeld_dma_fixture blofeld_own_dsp_fetch)
  foreach(scope IN ITEMS bytes pitch checkpoint)
    add_test(NAME original_dsp_clocked_wire_${scope} COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/test/original_dsp_clocked_wire.py"
      $<TARGET_FILE:blofeld-original-dsp-clocked-wire-test>
      "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
      "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-dsp-clocked-wire-${scope}.trace" ${scope})
    set_tests_properties(original_dsp_clocked_wire_${scope} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 120)
  endforeach()
  set_tests_properties(original_dsp_clocked_wire_bytes PROPERTIES TIMEOUT 300)
  add_executable(blofeld-dsp-test test/dsp_core_test.cpp test/dsp_core_fixture.cpp
    test/dsp_core_pinit.cpp)
  target_link_libraries(blofeld-dsp-test PRIVATE blofeld_dma_fixture blofeld_own_dsp_fetch)
  add_test(NAME native_dsp COMMAND blofeld-dsp-test)
  add_executable(blofeld-dsp-memory-map-test test/dsp_memory_map_test.cpp)
  target_link_libraries(blofeld-dsp-memory-map-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME dsp_memory_map COMMAND blofeld-dsp-memory-map-test)
  set_tests_properties(dsp_memory_map PROPERTIES TIMEOUT 60)
  add_executable(blofeld-dsp-interrupt-test test/dsp_interrupt_test.cpp
    test/dsp_interrupt_fixture.cpp test/dsp_interrupt_restore.cpp)
  target_link_libraries(blofeld-dsp-interrupt-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_interrupts COMMAND blofeld-dsp-interrupt-test)
  add_executable(blofeld-own-dsp-vector-pair-test test/own_dsp_vector_pair_test.cpp)
  target_link_libraries(blofeld-own-dsp-vector-pair-test PRIVATE blofeld_own_dsp_fetch blofeld_dma_fixture)
  add_test(NAME own_dsp_vector_pair COMMAND blofeld-own-dsp-vector-pair-test)
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
  add_executable(blofeld-dma-test test/dsp_dma_test.cpp test/dsp_dma_state.cpp test/dsp_dma_native.cpp)
  target_link_libraries(blofeld-dma-test PRIVATE blofeld_own_dsp_fetch blofeld_dma_fixture)
  add_test(NAME dsp_dma COMMAND blofeld-dma-test)
  if(NOT BLOFELD_SANITIZER STREQUAL "none")
    add_executable(blofeld-dma-callback-allocation-test test/dsp_dma_callback_allocation_test.cpp)
    target_link_libraries(blofeld-dma-callback-allocation-test PRIVATE blofeld_dsp_peripherals)
    add_test(NAME dsp_dma_callback_allocation COMMAND blofeld-dma-callback-allocation-test)
    set_tests_properties(dsp_dma_callback_allocation PROPERTIES TIMEOUT 120)
  endif()
  add_executable(blofeld-bank-test test/dsp_bank_test.cpp test/dsp_bank_phase.cpp test/dsp_bank_native.cpp)
  target_link_libraries(blofeld-bank-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_bank_arbitration COMMAND blofeld-bank-test)
  add_executable(blofeld-program-latch-test test/dsp_program_latch_test.cpp)
  target_link_libraries(blofeld-program-latch-test PRIVATE blofeld_own_dsp_fetch blofeld_dma_fixture)
  add_test(NAME dsp_program_latch COMMAND blofeld-program-latch-test)
  add_executable(blofeld-dax-test test/dsp_dax_test.cpp test/dsp_dax_native.cpp test/dsp_dax_dma.cpp)
  target_link_libraries(blofeld-dax-test PRIVATE blofeld_dma_fixture)
  add_test(NAME dsp_dax COMMAND blofeld-dax-test)
  add_executable(blofeld-esai-clock-test test/dsp_esai_clock_test.cpp test/dsp_esai_clock_native.cpp)
  target_link_libraries(blofeld-esai-clock-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME dsp_esai_clock COMMAND blofeld-esai-clock-test)
  add_executable(blofeld-esai-status-fetch-test test/dsp_esai_status_fetch_test.cpp)
  target_link_libraries(blofeld-esai-status-fetch-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME dsp_esai_status_fetch COMMAND blofeld-esai-status-fetch-test)
  add_executable(blofeld-esai-interrupt-test
    test/dsp_esai_interrupt_test.cpp test/dsp_esai_interrupt_native.cpp)
  target_link_libraries(blofeld-esai-interrupt-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME dsp_esai_interrupts COMMAND blofeld-esai-interrupt-test)
  add_executable(blofeld-own-dsp-fetch-test test/own_dsp_fetch_test.cpp)
  target_link_libraries(blofeld-own-dsp-fetch-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME own_dsp_fetch COMMAND blofeld-own-dsp-fetch-test)
  add_executable(blofeld-own-dsp-clock-test test/own_dsp_clock_test.cpp)
  target_link_libraries(blofeld-own-dsp-clock-test PRIVATE blofeld_own_dsp_io)
  add_test(NAME own_dsp_clock COMMAND blofeld-own-dsp-clock-test)
  add_executable(blofeld-own-dsp-ram-bit-dma-test test/own_dsp_ram_bit_dma_test.cpp)
  target_link_libraries(blofeld-own-dsp-ram-bit-dma-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME own_dsp_ram_bit_dma COMMAND blofeld-own-dsp-ram-bit-dma-test)
  add_executable(blofeld-own-dsp-ram-bit-test test/own_dsp_ram_bit_test.cpp)
  target_link_libraries(blofeld-own-dsp-ram-bit-test PRIVATE blofeld_own_dsp_io)
  add_test(NAME own_dsp_ram_bit COMMAND blofeld-own-dsp-ram-bit-test)
  set_tests_properties(own_dsp_ram_bit PROPERTIES TIMEOUT 1200)
  add_executable(blofeld-own-dsp-timeline-checkpoint-test test/own_dsp_timeline_checkpoint_test.cpp)
  target_link_libraries(blofeld-own-dsp-timeline-checkpoint-test PRIVATE blofeld_own_dsp_fetch)
  add_test(NAME own_dsp_timeline_checkpoint COMMAND blofeld-own-dsp-timeline-checkpoint-test)
endif()
