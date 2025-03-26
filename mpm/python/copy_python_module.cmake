#
# Copyright 2017 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
set(BINARY_DIR "" CACHE STRING "")
set(SOURCE_DIR "" CACHE STRING "")
file(COPY "${SOURCE_DIR}/usrp_mpm/" DESTINATION ${BINARY_DIR}/usrp_mpm
    FILES_MATCHING PATTERN *.py)
