# SPDX-License-Identifier: GPL-3.0-only
option(BLOFELD_USE_CCACHE "Use ccache when available" ON)

if(BLOFELD_USE_CCACHE)
  find_program(BLOFELD_CCACHE_EXECUTABLE NAMES ccache)
  if(BLOFELD_CCACHE_EXECUTABLE)
    foreach(language C CXX)
      if(NOT DEFINED CMAKE_${language}_COMPILER_LAUNCHER)
        set(CMAKE_${language}_COMPILER_LAUNCHER "${BLOFELD_CCACHE_EXECUTABLE}")
      endif()
    endforeach()
  endif()
endif()
