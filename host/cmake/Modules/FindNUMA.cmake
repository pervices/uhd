#
# Copyright 2026 Per Vices Corporation
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

#find libnuma on various systems
#sets NUMA_FOUND, NUMA_LIBRARIES, NUMA_INCLUDE_DIRS
#override NUMA_LIBRARIES NUMA_INCLUDE_DIRS to manually set

find_package(PkgConfig QUIET)
pkg_check_modules(PC_NUMA QUIET numa)

find_path(NUMA_INCLUDE_DIRS
    NAMES numa.h
    HINTS $ENV{NUMA_DIR}/include
    ${PC_NUMA_INCLUDEDIR}
    PATHS /usr/local/include /usr/include
)

find_library(NUMA_LIBRARIES
    NAMES numa
    HINTS $ENV{NUMA_DIR}/lib ${PC_NUMA_LIBDIR}
    PATHS /usr/local/lib /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NUMA DEFAULT_MSG NUMA_LIBRARIES NUMA_INCLUDE_DIRS)
mark_as_advanced(NUMA_INCLUDE_DIRS NUMA_LIBRARIES)
