# SPDX-License-Identifier: GPL-3.0-only
cmake_minimum_required(VERSION 3.25)
project(blofeld_emu VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR
   NOT CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19 OR
   NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 20)
  message(FATAL_ERROR "Use LLVM 19: cmake --preset strict")
endif()
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
find_package(ZLIB REQUIRED)
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

if(DEFINED blofeld_dependency_sanitizers)
  target_compile_options(blofeld_sleigh PRIVATE
    "-fsanitize=${blofeld_dependency_sanitizers}" -fno-sanitize-recover=all -fno-omit-frame-pointer -g)
endif()
# Apply clang-tidy to our targets, after the third-party target was created.
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

include(cmake/Hc12Spec.cmake)
add_library(blofeld_firmware src/firmware.cpp src/archive.cpp src/midi_file.cpp src/diagnostic.cpp)
target_include_directories(blofeld_firmware PUBLIC include)
target_link_libraries(blofeld_firmware PUBLIC blofeld_quality PRIVATE ZLIB::ZLIB)
add_library(blofeld_mcu src/mcu_bus.cpp src/mcu_memory.cpp src/mcu_gpio.cpp
  src/mcu_clock.cpp src/mcu_sci.cpp src/mcu_timer.cpp)
target_include_directories(blofeld_mcu PUBLIC include)
target_link_libraries(blofeld_mcu PUBLIC blofeld_firmware)
add_library(blofeld_core src/hc12.cpp)
add_dependencies(blofeld_core blofeld_hc12_spec)
target_include_directories(blofeld_core PUBLIC include)
target_link_libraries(blofeld_core PUBLIC blofeld_mcu PRIVATE blofeld_sleigh)
target_compile_definitions(blofeld_core PUBLIC
  BLOFELD_DEFAULT_SLA="${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla")
add_executable(blofeld-headless src/main.cpp)
target_link_libraries(blofeld-headless PRIVATE blofeld_core)

if(BUILD_TESTING)
  add_executable(blofeld-core-test test/core_test.cpp)
  target_link_libraries(blofeld-core-test PRIVATE blofeld_core)
  add_test(NAME native_core COMMAND blofeld-core-test)
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
  add_executable(blofeld-timer-test test/mcu_timer_test.cpp)
  target_link_libraries(blofeld-timer-test PRIVATE blofeld_mcu)
  add_test(NAME mcu_timer COMMAND blofeld-timer-test)
  add_executable(blofeld-boot-test test/original_boot_test.cpp)
  target_link_libraries(blofeld-boot-test PRIVATE blofeld_core)
  add_test(NAME original_mcu_boot COMMAND blofeld-boot-test
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/Blofeld_Firmware_V1.25.zip"
    "${CMAKE_CURRENT_BINARY_DIR}/Testing/original-boot.trace")
  set_tests_properties(original_mcu_boot PROPERTIES SKIP_RETURN_CODE 77)
  add_test(NAME firmware_python
    COMMAND ${Python3_EXECUTABLE} -m unittest discover -s test -v)
  set_tests_properties(firmware_python PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    ENVIRONMENT "BLOFELD_TEST_BINARY=$<TARGET_FILE:blofeld-headless>")
endif()

install(TARGETS blofeld-headless RUNTIME DESTINATION bin)
install(FILES LICENSE THIRD_PARTY.txt DESTINATION share/doc/blofeld-emu)
install(DIRECTORY LICENSES/ DESTINATION share/doc/blofeld-emu/licenses)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/share/HC12.sla" DESTINATION share/blofeld-emu)

file(GLOB_RECURSE BLOFELD_FORMAT_SOURCES CONFIGURE_DEPENDS
  "include/*.hpp" "src/*.hpp" "src/*.cpp" "test/*.cpp")
add_custom_target(format-check
  COMMAND "${BLOFELD_CLANG_FORMAT}" --dry-run --Werror ${BLOFELD_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
add_custom_target(quality
  COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
  DEPENDS format-check blofeld-headless blofeld-core-test blofeld-mcu-test
    blofeld-clock-test blofeld-sci-test blofeld-timer-test blofeld-boot-test
  USES_TERMINAL)
endif()

if(BLOFELD_FUZZING)
  add_executable(firmware-fuzz test/firmware_fuzz.cpp)
  target_link_libraries(firmware-fuzz PRIVATE blofeld_firmware)
  target_link_options(firmware-fuzz PRIVATE -fsanitize=fuzzer)
  add_executable(mcu-model-fuzz test/mcu_model_fuzz.cpp)
  target_link_libraries(mcu-model-fuzz PRIVATE blofeld_mcu)
  target_link_options(mcu-model-fuzz PRIVATE -fsanitize=fuzzer)
endif()
