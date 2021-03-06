# - Find the CONCERT LP solver.
# This code defines the following variables:
#
#  CONCERT_FOUND                 - TRUE if CONCERT was found.
#  CONCERT_INCLUDE_DIRS          - Full paths to all include dirs.
#  CONCERT_LIBRARIES             - Full paths to all libraries.
#  CONCERT_RUNTIME_LIBRARY       - Full path to the dll file on windows
#
# Usage:
#  find_package(concert)
#
# The location of CONCERT can be specified using the environment variable
# or cmake parameter DOWNWARD_CONCERT_ROOT. If different installations
# for 32-/64-bit versions and release/debug versions of CONCERT are available,
# they can be specified with
#   DOWNWARD_CONCERT_ROOT32
#   DOWNWARD_CONCERT_ROOT64
#   DOWNWARD_CONCERT_ROOT_RELEASE32
#   DOWNWARD_CONCERT_ROOT_RELEASE64
#   DOWNWARD_CONCERT_ROOT_DEBUG32
#   DOWNWARD_CONCERT_ROOT_DEBUG64
# More specific paths are preferred over less specific ones when searching
# for libraries.
#
# Note that the standard FIND_PACKAGE features are supported
# (QUIET, REQUIRED, etc.).

foreach(BITWIDTH 32 64)
    foreach(BUILDMODE "RELEASE" "DEBUG")
        set(CONCERT_HINT_PATHS_${BUILDMODE}${BITWIDTH}
            ${DOWNWARD_CONCERT_ROOT_${BUILDMODE}${BITWIDTH}}
            $ENV{DOWNWARD_CONCERT_ROOT_${BUILDMODE}${BITWIDTH}}
            ${DOWNWARD_CONCERT_ROOT${BITWIDTH}}
            $ENV{DOWNWARD_CONCERT_ROOT${BITWIDTH}}
            ${DOWNWARD_CONCERT_ROOT}
            $ENV{DOWNWARD_CONCERT_ROOT}
        )
    endforeach()
endforeach()

if(${CMAKE_SIZEOF_VOID_P} EQUAL 4)
    set(CONCERT_HINT_PATHS_RELEASE ${CONCERT_HINT_PATHS_RELEASE32})
    set(CONCERT_HINT_PATHS_DEBUG ${CONCERT_HINT_PATHS_DEBUG32})
elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set(CONCERT_HINT_PATHS_RELEASE ${CONCERT_HINT_PATHS_RELEASE64})
    set(CONCERT_HINT_PATHS_DEBUG ${CONCERT_HINT_PATHS_DEBUG64})
else()
    message(WARNING "Bitwidth could not be detected, preferring 32-bit version of CONCERT")
    set(CONCERT_HINT_PATHS_RELEASE
        ${CONCERT_HINT_PATHS_RELEASE32}
        ${CONCERT_HINT_PATHS_RELEASE64}
    )
    set(CONCERT_HINT_PATHS_DEBUG
        ${CONCERT_HINT_PATHS_DEBUG32}
        ${CONCERT_HINT_PATHS_DEBUG64}
    )
endif()

find_path(CONCERT_INCLUDE_DIRS
    NAMES
    ilconcert/iloenv.h
    HINTS
    ${CONCERT_HINT_PATHS_RELEASE}
    ${CONCERT_HINT_PATHS_DEBUG}
    PATH_SUFFIXES
    include
)

if(APPLE)
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32
        "lib/x86_osx/static_pic")
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_32 ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32})
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64
        "lib/x86-64_osx/static_pic")
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_64 ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64})
elseif(UNIX)
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32
        "lib/x86_sles10_4.1/static_pic"
        "lib/x86_linux/static_pic")
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_32 ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32})
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64
        "lib/x86-64_sles10_4.1/static_pic"
        "lib/x86-64_linux/static_pic")
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_64 ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64})
elseif(MSVC)
    # Note that the numbers are correct: Visual Studio 2011 is version 10.
    if (MSVC10)
        set(CONCERT_COMPILER_HINT "vs2011")
    elseif(MSVC11)
        set(CONCERT_COMPILER_HINT "vs2012")
    elseif(MSVC12)
        set(CONCERT_COMPILER_HINT "vs2013")
    endif()

    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32 "lib/x86_windows_${CONCERT_COMPILER_HINT}/stat_mda")
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_32 "lib/x86_windows_${CONCERT_COMPILER_HINT}/stat_mdd")
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64 "lib/x86-64_windows_${CONCERT_COMPILER_HINT}/stat_mda")
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_64 "lib/x86-64_windows_${CONCERT_COMPILER_HINT}/stat_mdd")
    if(${CMAKE_SIZEOF_VOID_P} EQUAL 4)
        set(CONCERT_RUNTIME_LIBRARY_HINT "bin/x86_win32")
    elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
        set(CONCERT_RUNTIME_LIBRARY_HINT "bin/x86_win64")
    endif()
endif()

if(${CMAKE_SIZEOF_VOID_P} EQUAL 4)
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32})
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG ${CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_32})
elseif(${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64})
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG ${CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_64})
else()
    message(WARNING "Bitwidth could not be detected, preferring 32bit version of CONCERT")
    set(CONCERT_LIBRARY_PATH_SUFFIX_RELEASE
        ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32}
        ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64}
    )
    set(CONCERT_LIBRARY_PATH_SUFFIX_DEBUG
        ${CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_32}
        ${CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_64}
    )
endif()

find_library(CONCERT_LIBRARY_RELEASE
    NAMES
    concert
    concert1262
    HINTS
    ${CONCERT_HINT_PATHS_RELEASE}
    PATH_SUFFIXES
    ${CONCERT_LIBRARY_PATH_SUFFIX_RELEASE}
)

find_library(CONCERT_LIBRARY_DEBUG
    NAMES
    concert
    concert1262
    HINTS
    ${CONCERT_HINT_PATHS_DEBUG}
    PATH_SUFFIXES
    ${CONCERT_LIBRARY_PATH_SUFFIX_DEBUG}
)

if(CONCERT_LIBRARY_RELEASE OR CONCERT_LIBRARY_DEBUG)
    find_package(Threads REQUIRED)
    set(CONCERT_LIBRARIES
        optimized ${CONCERT_LIBRARY_RELEASE} ${CMAKE_THREAD_LIBS_INIT}
        debug ${CONCERT_LIBRARY_DEBUG} ${CMAKE_THREAD_LIBS_INIT}
    )
endif()

# HACK: there must be a better way to find the dll file.
find_path(CONCERT_RUNTIME_LIBRARY_PATH
    NAMES
    concert1262.dll
    HINTS
    ${CONCERT_HINT_PATHS_RELEASE}
    ${CONCERT_HINT_PATHS_DEBUG}
    PATH_SUFFIXES
    ${CONCERT_RUNTIME_LIBRARY_HINT}
)
if(CONCERT_RUNTIME_LIBRARY_PATH)
    set(CONCERT_RUNTIME_LIBRARY "${CONCERT_RUNTIME_LIBRARY_PATH}/CONCERT1262.dll")
endif()

# Check if everything was found and set CONCERT_FOUND.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Concert
    REQUIRED_VARS CONCERT_INCLUDE_DIRS CONCERT_LIBRARIES
)

mark_as_advanced(
    CONCERT_INCLUDE_DIRS CONCERT_LIBRARIES CONCERT_LIBRARY_PATH_SUFFIX
    CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_32 CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_32
    CONCERT_LIBRARY_PATH_SUFFIX_RELEASE_64 CONCERT_LIBRARY_PATH_SUFFIX_DEBUG_64
    CONCERT_LIBRARY_PATH_SUFFIX_RELEASE CONCERT_LIBRARY_PATH_SUFFIX_DEBUG
    CONCERT_LIBRARY_RELEASE CONCERT_LIBRARY_DEBUG CONCERT_RUNTIME_LIBRARY_PATH
)
