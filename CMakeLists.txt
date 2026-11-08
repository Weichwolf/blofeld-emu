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
set(CMAKE_CXX_CLANG_TIDY "${BLOFELD_CLANG_TIDY};--warnings-as-errors=*")

add_library(blofeld_vi_display src/vi/display/st7565.cpp src/vi/display/lcd_image.cpp)
target_include_directories(blofeld_vi_display PUBLIC src/vi)
target_link_libraries(blofeld_vi_display PUBLIC blofeld_quality)
add_library(blofeld_host_files src/host/files/directory.cpp)
target_include_directories(blofeld_host_files PUBLIC src/host)
target_link_libraries(blofeld_host_files PUBLIC blofeld_quality)
add_library(blofeld_app_display src/app/display/publication.cpp)
target_include_directories(blofeld_app_display PUBLIC src/app)
target_link_libraries(blofeld_app_display PUBLIC blofeld_vi_display blofeld_host_files)

file(GLOB_RECURSE BLOFELD_FORMAT_SOURCES CONFIGURE_DEPENDS
  "src/*.hpp" "src/*.cpp" "test/*.hpp" "test/*.cpp")
add_custom_target(format-check
  COMMAND "${BLOFELD_CLANG_FORMAT}" --dry-run --Werror ${BLOFELD_FORMAT_SOURCES}
  VERBATIM)
if(BUILD_TESTING)
  function(blofeld_unit name source library)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE ${library})
    add_test(NAME ${name} COMMAND ${name} "${CMAKE_CURRENT_BINARY_DIR}/Testing/${name}")
    set_tests_properties(${name} PROPERTIES TIMEOUT 10)
    set_property(GLOBAL APPEND PROPERTY BLOFELD_UNIT_TARGETS ${name})
  endfunction()
  blofeld_unit(vi-display-test test/vi/display/st7565_test.cpp blofeld_vi_display)
  blofeld_unit(vi-lcd-image-test test/vi/display/lcd_image_test.cpp blofeld_vi_display)
  blofeld_unit(host-directory-test test/host/files/directory_test.cpp blofeld_host_files)
  blofeld_unit(app-display-test test/app/display/publication_test.cpp blofeld_app_display)
  add_test(NAME app-ui-frame-test COMMAND "${Python3_EXECUTABLE}" -B
    "${CMAKE_CURRENT_SOURCE_DIR}/test/app/ui/frame_test.py" "${CMAKE_CURRENT_BINARY_DIR}/Testing")
  set_tests_properties(app-ui-frame-test PROPERTIES TIMEOUT 10)
  get_property(blofeld_unit_targets GLOBAL PROPERTY BLOFELD_UNIT_TARGETS)
  add_custom_target(quality
    COMMAND "${CMAKE_CTEST_COMMAND}" --output-on-failure
    DEPENDS format-check ${blofeld_unit_targets}
    USES_TERMINAL)
endif()

install(PROGRAMS tools/blofeld-display DESTINATION bin COMPONENT Display)
install(FILES src/app/ui/__init__.py src/app/ui/frame.py src/app/ui/watch.py src/app/ui/display.py
  DESTINATION share/blofeld-emu/ui COMPONENT Display)
install(FILES packaging/org.blofeld_emu.Display.desktop
  DESTINATION share/applications COMPONENT Display)
install(FILES LICENSE THIRD_PARTY.txt DESTINATION share/doc/blofeld-emu)
install(DIRECTORY LICENSES/ DESTINATION share/doc/blofeld-emu/licenses)
