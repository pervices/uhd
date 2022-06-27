
# Copyright 2010-2015 Ettus Research LLC
# Copyright 2018 Ettus Research, a National Instruments Company
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# unit test support
########################################################################
include(UHDUnitTest)

########################################################################
# build test common
########################################################################
set(test_common_SOURCEDIR ${CMAKE_CURRENT_SOURCE_DIR}/common)
add_subdirectory(common)
link_directories(${test_common_SOURCEDIR})

########################################################################
# unit test suite
########################################################################
set(test_sources
    addr_test.cpp
    buffer_test.cpp
    byteswap_test.cpp
    cast_test.cpp
    cal_database_test.cpp
    cal_data_iq_test.cpp
    cal_data_gain_pwr_test.cpp
    chdr_parse_test.cpp
    cal_data_dsa_test.cpp
    chdr_test.cpp
    constrained_device_args_test.cpp
    convert_test.cpp
    dict_test.cpp
    eeprom_utils_test.cpp
    error_test.cpp
    fp_compare_delta_test.cpp
    fp_compare_epsilon_test.cpp
    gain_group_test.cpp
    interpolation_test.cpp
    isatty_test.cpp
    log_test.cpp
    math_test.cpp
    narrow_cast_test.cpp
    property_test.cpp
    ranges_test.cpp
    rfnoc_node_test.cpp
    scope_exit_test.cpp
    sensors_test.cpp
    soft_reg_test.cpp
    sph_recv_test.cpp
    sph_send_test.cpp
    subdev_spec_test.cpp
    time_spec_test.cpp
    tasks_test.cpp
    vrt_test.cpp
    expert_test.cpp
    fe_conn_test.cpp
    link_test.cpp
    rx_streamer_test.cpp
    tx_streamer_test.cpp
    block_id_test.cpp
    rfnoc_property_test.cpp
    multichan_register_iface_test.cpp
)

# Note: Python-based tests cannot have the same name as a C++-based test (i.e.,
# only differ in the cpp/py file extension). If in doubt, prepend 'py'
set(pytest_sources
    pyranges_test.py
    verify_fbs_test.py
    pychdr_parse_test.py
    uhd_image_downloader_test.py
)

#turn each test cpp file into an executable with an int main() function
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

if(ENABLE_C_API)
    list(APPEND test_sources
        eeprom_c_test.c
        error_c_test.cpp
        ranges_c_test.c
        sensors_c_test.c
        string_vector_c_test.c
        subdev_spec_c_test.c
    )
endif(ENABLE_C_API)

include_directories("${UHD_SOURCE_DIR}/lib/include")
include_directories("${UHD_BINARY_DIR}/lib/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/common")

if(ENABLE_CRIMSON_TNG)
# Cannot call into crimson_tng_impl.o that is in libuhd.so due to default visibility=hidden
# so it is necessary to link it into the executable before the shared library is linked in
add_executable(crimson_tng_test
    crimson_tng_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/crimson_tng/crimson_tng_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/crimson_tng/crimson_tng_io_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/crimson_tng/crimson_tng_iface.cpp
)
target_link_libraries(crimson_tng_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(crimson_tng_test crimson_tng_test)
UHD_INSTALL(TARGETS crimson_tng_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)
endif(ENABLE_CRIMSON_TNG)

if(ENABLE_CYAN_16T)
# Cannot call into cyan_16t_impl.o that is in libuhd.so due to default visibility=hidden
# so it is necessary to link it into the executable before the shared library is linked in
add_executable(cyan_16t_test
    cyan_16t_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_16t/cyan_16t_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_16t/io_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_16t/cyan_16t_iface.cpp
)
target_link_libraries(cyan_16t_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(cyan_16t_test cyan_16t_test)
UHD_INSTALL(TARGETS cyan_16t_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)
endif(ENABLE_CYAN_16T)

if(ENABLE_CYAN_P1HDR16T)
# Cannot call into cyan_p1hdr16t_impl.o that is in libuhd.so due to default visibility=hidden
# so it is necessary to link it into the executable before the shared library is linked in
add_executable(cyan_p1hdr16t_test
    cyan_p1hdr16t_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_p1hdr16t/cyan_p1hdr16t_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_p1hdr16t/io_p1hdrimpl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_p1hdr16t/cyan_p1hdr16t_iface.cpp
)
target_link_libraries(cyan_p1hdr16t_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(cyan_p1hdr16t_test cyan_p1hdr16t_test)
UHD_INSTALL(TARGETS cyan_p1hdr16t_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)
endif(ENABLE_CYAN_P1HDR16T)

if(ENABLE_CYAN_P1HDR32T)
# Cannot call into cyan_p1hdr16t_impl.o that is in libuhd.so due to default visibility=hidden
# so it is necessary to link it into the executable before the shared library is linked in
add_executable(cyan_p1hdr32t_test
    cyan_p1hdr32t_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_p1hdr32t/cyan_p1hdr32t_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_p1hdr32t/io_impl.cpp//this was io_p1hdrimpl.cpp for hdr16t, but this is the closest one to that name
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_p1hdr32t/cyan_p1hdr32t_iface.cpp
)
target_link_libraries(cyan_p1hdr32t_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(cyan_p1hdr32t_test cyan_p1hdr16t_test)
UHD_INSTALL(TARGETS cyan_p1hdr32t_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)
endif(ENABLE_CYAN_P1HDR32T)

if(ENABLE_CYAN_64T)
# Cannot call into cyan_64t_impl.o that is in libuhd.so due to default visibility=hidden
# so it is necessary to link it into the executable before the shared library is linked in
add_executable(cyan_64t_test
    cyan_64t_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_64t/cyan_64t_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_64t/io_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_64t/cyan_64t_iface.cpp
)
target_link_libraries(cyan_64t_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(cyan_64t_test cyan_64t_test)
UHD_INSTALL(TARGETS cyan_64t_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)
endif(ENABLE_CYAN_64T)

if(ENABLE_CYAN_8T)
# Cannot call into cyan_8t_impl.o that is in libuhd.so due to default visibility=hidden
# so it is necessary to link it into the executable before the shared library is linked in
add_executable(cyan_8t_test
    cyan_8t_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_8t/cyan_8t_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_8t/io_impl.cpp
    ${CMAKE_SOURCE_DIR}/lib/usrp/cyan_8t/cyan_8t_iface.cpp
)
target_link_libraries(cyan_8t_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(cyan_8t_test cyan_8t_test)
UHD_INSTALL(TARGETS cyan_8t_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)
endif(ENABLE_CYAN_8T)

# Other tests that don't directly link with libuhd: (TODO find a nicer way to do this)
include_directories(${CMAKE_BINARY_DIR}/lib/rfnoc/nocscript/)
include_directories(${CMAKE_SOURCE_DIR}/lib/rfnoc/nocscript/)
add_executable(nocscript_expr_test
    nocscript_expr_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/rfnoc/nocscript/expression.cpp
)
target_link_libraries(nocscript_expr_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(nocscript_expr_test nocscript_expr_test)
UHD_INSTALL(TARGETS nocscript_expr_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)

add_executable(nocscript_ftable_test
    nocscript_ftable_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/rfnoc/nocscript/function_table.cpp
    ${CMAKE_SOURCE_DIR}/lib/rfnoc/nocscript/expression.cpp
)
target_link_libraries(nocscript_ftable_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(nocscript_ftable_test nocscript_ftable_test)
UHD_INSTALL(TARGETS nocscript_ftable_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)

add_executable(nocscript_parser_test
    nocscript_parser_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/rfnoc/nocscript/parser.cpp
    ${CMAKE_SOURCE_DIR}/lib/rfnoc/nocscript/function_table.cpp
    ${CMAKE_SOURCE_DIR}/lib/rfnoc/nocscript/expression.cpp
)
target_link_libraries(nocscript_parser_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(nocscript_parser_test nocscript_parser_test)
UHD_INSTALL(TARGETS nocscript_parser_test RUNTIME DESTINATION ${PKG_LIB_DIR}/tests COMPONENT tests)

add_executable(config_parser_test
    config_parser_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/utils/config_parser.cpp
)
target_link_libraries(config_parser_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(config_parser_test config_parser_test)
UHD_INSTALL(TARGETS
    config_parser_test
    RUNTIME
    DESTINATION ${PKG_LIB_DIR}/tests
    COMPONENT tests
)

add_executable(paths_test
    paths_test.cpp
    ${CMAKE_SOURCE_DIR}/lib/utils/pathslib.cpp
)
# Careful: This is to satisfy the out-of-library build of paths.cpp. This is
# duplicate code from lib/utils/CMakeLists.txt, and it's been simplified.
set(UHD_LIB_DIR "lib")
file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" UHD_PKG_PATH)
string(REPLACE "\\" "\\\\" UHD_PKG_PATH "${UHD_PKG_PATH}")
set_source_files_properties(
    ${CMAKE_SOURCE_DIR}/lib/utils/paths.cpp
    PROPERTIES COMPILE_DEFINITIONS
    "UHD_PKG_PATH=\"${UHD_PKG_PATH}\";UHD_LIB_DIR=\"${UHD_LIB_DIR}\""
)
target_link_libraries(paths_test uhd ${Boost_LIBRARIES})
UHD_ADD_TEST(paths_test paths_test)
UHD_INSTALL(TARGETS
    paths_test
    RUNTIME
    DESTINATION ${PKG_LIB_DIR}/tests
    COMPONENT tests
)

########################################################################
# demo of a loadable module
########################################################################
if(MSVC OR APPLE OR LINUX)
    add_library(module_test MODULE module_test.cpp)
    target_link_libraries(module_test uhd)
endif()

add_subdirectory(devtest)
