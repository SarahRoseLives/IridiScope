# - Find SDRplay API
# Find the native SDRplay API v3 includes and library
# This module defines
#  SDRPLAY_INCLUDE_DIR, where to find sdrplay_api.h, etc.
#  SDRPLAY_LIBRARIES, the libraries needed to use SDRplay API.
#  SDRPLAY_FOUND, If false, do not try to use SDRplay API.
# also defined, but not for general use are
#  SDRPLAY_LIBRARY, where to find the SDRplay API library.

find_path(SDRPLAY_INCLUDE_DIR
    NAMES sdrplay_api.h
    HINTS
        ${SDRPLAY_DIR}/include
        /opt/homebrew/include
        /opt/local/include
        /usr/include
        /usr/local/include
)

find_library(SDRPLAY_LIBRARY
    NAMES sdrplay_api
    HINTS
        $ENV{SDRPLAY_DIR}/lib
        /opt/homebrew/lib
        /opt/local/lib
        /usr/lib
        /usr/local/lib
        /usr/lib/x86_64-linux-gnu
        /usr/lib/aarch64-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SDRplay DEFAULT_MSG SDRPLAY_LIBRARY SDRPLAY_INCLUDE_DIR)

if (SDRPLAY_LIBRARY AND SDRPLAY_INCLUDE_DIR)
    set(SDRPLAY_LIBRARIES ${SDRPLAY_LIBRARY})
    set(SDRPLAY_FOUND "YES")
else (SDRPLAY_LIBRARY AND SDRPLAY_INCLUDE_DIR)
    set(SDRPLAY_FOUND "NO")
endif (SDRPLAY_LIBRARY AND SDRPLAY_INCLUDE_DIR)

if (SDRPLAY_FOUND)
    if (NOT SDRplay_FIND_QUIETLY)
        message(STATUS "Found SDRplay API: ${SDRPLAY_LIBRARIES}")
    endif (NOT SDRplay_FIND_QUIETLY)
else (SDRPLAY_FOUND)
    if (SDRplay_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find SDRplay API library")
    endif (SDRplay_FIND_REQUIRED)
endif (SDRPLAY_FOUND)

mark_as_advanced(
        SDRPLAY_LIBRARY
        SDRPLAY_INCLUDE_DIR
)

set(SDRPLAY_INCLUDE_DIRS ${SDRPLAY_INCLUDE_DIR})
set(SDRPLAY_LIBRARIES ${SDRPLAY_LIBRARY})
