# SPDX-License-Identifier: GPL-3.0-only
# Keep client code in the instrumented native build; never load system libpulse.
function(blofeld_stage_audio_archive component output)
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/doc/dependencies.json" manifest)
  string(JSON name GET "${manifest}" ${component} file)
  string(JSON size GET "${manifest}" ${component} bytes)
  string(JSON hash GET "${manifest}" ${component} sha256)
  set(archive "${CMAKE_CURRENT_SOURCE_DIR}/doc/local/${name}")
  if(NOT EXISTS "${archive}")
    message(FATAL_ERROR "Missing ${component}: run python3 tools/dependencies.py --fetch --component ${component}")
  endif()
  file(SIZE "${archive}" actual_size)
  file(SHA256 "${archive}" actual_hash)
  if(NOT actual_size EQUAL size OR NOT actual_hash STREQUAL hash)
    message(FATAL_ERROR "${component} source size/SHA-256 differs from doc/dependencies.json")
  endif()
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${archive}")
  set(stage "${CMAKE_CURRENT_BINARY_DIR}/generated/audio/${component}")
  set(pristine "${stage}-pristine")
  file(REMOVE_RECURSE "${pristine}")
  file(ARCHIVE_EXTRACT INPUT "${archive}" DESTINATION "${pristine}")
  file(GLOB_RECURSE files RELATIVE "${pristine}" "${pristine}/*")
  foreach(relative IN LISTS files)
    get_filename_component(parent "${relative}" DIRECTORY)
    file(MAKE_DIRECTORY "${stage}/${parent}")
    file(COPY_FILE "${pristine}/${relative}" "${stage}/${relative}" ONLY_IF_DIFFERENT)
  endforeach()
  file(REMOVE_RECURSE "${pristine}")
  set(${output} "${stage}" PARENT_SCOPE)
endfunction()

function(blofeld_add_audio_dependencies)
  blofeld_stage_audio_archive(pulseaudio pulse_stage)
  blofeld_stage_audio_archive(libsndfile sndfile_stage)
  blofeld_stage_audio_archive(meson meson_stage)
  set(pulse_source "${pulse_stage}/pulseaudio-17.0")
  set(sndfile_source "${sndfile_stage}/libsndfile-1.2.2")
  set(callback_patch "${CMAKE_CURRENT_SOURCE_DIR}/patches/pulseaudio-callback-types.patch")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${callback_patch}")
  execute_process(COMMAND "${BLOFELD_PATCH}" --batch --silent --fuzz=0 -p1
    --directory "${pulse_source}" --input "${callback_patch}"
    COMMAND_ERROR_IS_FATAL ANY)
  foreach(notice "${pulse_source}/LICENSE|PulseAudio.txt"
                 "${pulse_source}/LGPL|LGPL-2.1.txt"
                 "${sndfile_source}/COPYING|libsndfile.txt")
    string(REPLACE "|" ";" paths "${notice}")
    list(GET paths 0 original)
    list(GET paths 1 packaged)
    file(SHA256 "${original}" original_hash)
    file(SHA256 "${CMAKE_CURRENT_SOURCE_DIR}/LICENSES/${packaged}" packaged_hash)
    if(NOT original_hash STREQUAL packaged_hash)
      message(FATAL_ERROR "Audio dependency notice differs: ${packaged}")
    endif()
  endforeach()
  # Honor scoped libsndfile options on the first configure.
  set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
  set(BUILD_SHARED_LIBS OFF)
  set(BUILD_TESTING OFF)
  set(BUILD_PROGRAMS OFF)
  set(BUILD_EXAMPLES OFF)
  set(ENABLE_CPACK OFF)
  set(ENABLE_EXTERNAL_LIBS OFF)
  set(ENABLE_MPEG OFF)
  add_subdirectory("${sndfile_source}" "sndfile" EXCLUDE_FROM_ALL SYSTEM)
  blofeld_harden_native(sndfile)
  if(DEFINED blofeld_dependency_sanitizers)
    target_compile_options(sndfile PRIVATE "-fsanitize=${blofeld_dependency_sanitizers}"
      -fno-sanitize-recover=all -fno-omit-frame-pointer -g)
  endif()
  set(pulse_build "${CMAKE_CURRENT_BINARY_DIR}/generated/audio/pulse-config")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/configure_pulse.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/doc/dependencies.json")
  execute_process(COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/configure_pulse.py"
    --source "${pulse_source}" --build "${pulse_build}" --meson "${meson_stage}"
    --compiler "${CMAKE_C_COMPILER}" --sndfile-source "${sndfile_source}"
    --sndfile-build "${CMAKE_CURRENT_BINARY_DIR}/sndfile"
    COMMAND_ERROR_IS_FATAL ANY)
  include("${pulse_build}/native-targets.cmake")
  foreach(target blofeld_pulse_common blofeld_pulse)
    blofeld_harden_native(${target})
    if(DEFINED blofeld_dependency_sanitizers)
      target_compile_options(${target} PRIVATE "-fsanitize=${blofeld_dependency_sanitizers}"
        -fno-sanitize-recover=all -fno-omit-frame-pointer -g)
    endif()
  endforeach()
  target_link_libraries(blofeld_pulse_common PRIVATE sndfile m ${CMAKE_DL_LIBS})
  target_link_libraries(blofeld_pulse PUBLIC blofeld_pulse_common)
endfunction()

blofeld_add_audio_dependencies()
