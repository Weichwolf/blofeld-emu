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
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Git REQUIRED)
find_package(Threads REQUIRED)

# Core correctness suites run in their own projects, independently of the app.
set(CPU12_BUILD_TESTING OFF)
set(DSP56300_BUILD_TESTING OFF)
include(cmake/Cpu12.cmake)
include(cmake/Dsp56300.cmake)
include(cmake/Zlib.cmake)
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

add_library(blofeld_vi_display src/vi/display/st7565.cpp src/vi/display/lcd_image.cpp)
target_include_directories(blofeld_vi_display PUBLIC src/vi)
target_link_libraries(blofeld_vi_display PUBLIC blofeld_quality)
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
add_library(blofeld_vi_storage src/vi/storage/flash_image.cpp)
target_include_directories(blofeld_vi_storage PUBLIC src/vi)
target_link_libraries(blofeld_vi_storage PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_vi_mcu_memory src/vi/mcu/memory.cpp)
target_include_directories(blofeld_vi_mcu_memory PUBLIC src/vi)
target_link_libraries(blofeld_vi_mcu_memory PUBLIC blofeld_vi_storage cpu12::core)
add_library(blofeld_vi_execution
  src/vi/io/shi.cpp src/vi/io/mcu_flash.cpp
  src/vi/mcu/boot_port.cpp src/vi/mcu/ports.cpp src/vi/mcu/timer.cpp src/vi/mcu/clock.cpp
  src/vi/mcu/execute.cpp src/vi/dsp/execute.cpp src/vi/vm_context.cpp)
target_include_directories(blofeld_vi_execution PUBLIC src/vi)
target_link_libraries(blofeld_vi_execution PUBLIC
  blofeld_vi_mcu_memory dsp56300::core Threads::Threads)
add_library(blofeld_app_profile src/app/profile.cpp)
add_library(blofeld_host_process src/host/process/configuration.cpp)
target_include_directories(blofeld_host_process PUBLIC src/host)
target_link_libraries(blofeld_host_process PUBLIC blofeld_quality)
target_include_directories(blofeld_app_profile PUBLIC src/app)
target_link_libraries(blofeld_app_profile PUBLIC blofeld_host_files blofeld_host_process)
add_executable(blofeld-headless src/app/main.cpp)
target_include_directories(blofeld-headless PRIVATE src/app)
target_link_libraries(blofeld-headless PRIVATE blofeld_vi_execution blofeld_app_profile)

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
  blofeld_unit(vi-display-test test/vi/display/st7565_test.cpp blofeld_vi_display)
  blofeld_unit(vi-lcd-image-test test/vi/display/lcd_image_test.cpp blofeld_vi_display)
  blofeld_unit(host-directory-test test/host/files/directory_test.cpp blofeld_host_files)
  blofeld_unit(app-display-test test/app/display/publication_test.cpp blofeld_app_display)
  blofeld_unit(host-read-test test/host/files/read_test.cpp blofeld_host_files)
  set(blofeld_unit_inputs "${CMAKE_CURRENT_BINARY_DIR}/generated/unit-inputs")
  add_custom_command(OUTPUT "${blofeld_unit_inputs}/zip.done" "${blofeld_unit_inputs}/firmware.done"
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
  blofeld_unit(vi-mcu-memory-test test/vi/mcu/memory_test.cpp blofeld_vi_mcu_memory)
  target_include_directories(vi-mcu-memory-test PRIVATE "${BLOFELD_CPU12_ROOT}/test/support")
  blofeld_unit(vi-vm-context-test test/vi/vm_context_test.cpp blofeld_vi_execution)
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
  get_property(blofeld_unit_targets GLOBAL PROPERTY BLOFELD_UNIT_TARGETS)
  add_custom_target(quality
    COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
    DEPENDS format-check blofeld-headless ${blofeld_unit_targets}
    USES_TERMINAL)
endif()

install(TARGETS blofeld-headless RUNTIME DESTINATION bin)
install(PROGRAMS tools/blofeld-display DESTINATION bin COMPONENT Display)
install(FILES src/app/ui/__init__.py src/app/ui/frame.py src/app/ui/watch.py src/app/ui/display.py
  DESTINATION share/blofeld-emu/ui COMPONENT Display)
install(FILES packaging/org.blofeld_emu.Display.desktop
  DESTINATION share/applications COMPONENT Display)
install(FILES LICENSE THIRD_PARTY.txt DESTINATION share/doc/blofeld-emu)
install(DIRECTORY LICENSES/ DESTINATION share/doc/blofeld-emu/licenses)
