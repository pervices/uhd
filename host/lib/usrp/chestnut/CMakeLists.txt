#
# Copyright 2024 Per Vices Corporation
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# This file included, use CMake directory variables
########################################################################

IF(ENABLE_CHESTNUT)
    LIBUHD_APPEND_SOURCES(
        ${CMAKE_CURRENT_SOURCE_DIR}/chestnut_impl.cpp
    )

    if(ENABLE_DPDK)
        include_directories(${DPDK_INCLUDE_DIRS})
        add_definitions(-DHAVE_DPDK)
    endif(ENABLE_DPDK)
ENDIF(ENABLE_CHESTNUT)
