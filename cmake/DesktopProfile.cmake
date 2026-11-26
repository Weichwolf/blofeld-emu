# SPDX-License-Identifier: GPL-3.0-only
set(BLOFELD_DOWNLOADS "${CMAKE_CURRENT_SOURCE_DIR}/build/local/downloads" CACHE PATH
  "Directory containing the local firmware.zip, sounds.zip and samples.zip files")
find_program(BLOFELD_PDFTOTEXT pdftotext)
set(blofeld_desktop "${CMAKE_CURRENT_BINARY_DIR}/desktop")
add_custom_target(blofeld-profile
  COMMAND "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/tools/desktop_profile/prepare.py"
    --downloads "${BLOFELD_DOWNLOADS}" --output "${blofeld_desktop}"
    --cache "${CMAKE_CURRENT_SOURCE_DIR}/build/local/profile-downloads"
    --pdftotext "$<IF:$<BOOL:${BLOFELD_PDFTOTEXT}>,${BLOFELD_PDFTOTEXT},pdftotext>"
  USES_TERMINAL VERBATIM)
add_custom_target(blofeld-prepare
  COMMAND "$<TARGET_FILE:blofeld-headless>" --profile "${blofeld_desktop}/desktop.profile" --prepare
  DEPENDS blofeld-headless blofeld-profile
  USES_TERMINAL VERBATIM)
