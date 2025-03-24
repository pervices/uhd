#!/bin/sh

# Coerce CMake to use gcc and g++ from ccache, if available
# https://stackoverflow.com/a/17275650
export PATH=/usr/lib/ccache:"${PATH}"
export CC="$(which gcc)"
export CXX="$(which g++)"

# we're doing a non-install build (for now) so use /usr/local
PREFIX=/usr/local

#echo "CC: ${CC}"
#echo "CXX: ${CXX}"
#echo "ccache:"
#ccache -p

mkdir -p host/build
cd host/build
cmake \
        -Wno-dev \
	-DPYTHON_EXECUTABLE=$(which python2) \
        -DCMAKE_BUILD_TYPE:STRING=Debug \
        -DENABLE_C_API=OFF \
        -DENABLE_MANUAL=OFF \
        -DENABLE_MAN_PAGES=OFF \
        -DENABLE_DOXYGEN=OFF \
        -DCMAKE_INSTALL_PREFIX=${PREFIX} \
        -DENABLE_EXAMPLES=ON \
        -DENABLE_UTILS=ON \
        -DENABLE_TESTS=ON \
        -DENABLE_N230=OFF \
        -DENABLE_N300=OFF \
        -DENABLE_E320=OFF \
        -DENABLE_USRP1=OFF \
        -DENABLE_USRP2=OFF \
        -DENABLE_CRIMSON_TNG=ON \
        -DENABLE_B100=OFF \
        -DENABLE_B200=OFF \
        -DENABLE_X300=OFF \
        -DENABLE_USB=OFF \
        -DENABLE_OCTOCLOCK=OFF \
        -DENABLE_MPMD=OFF \
        -DENABLE_DPDK=OFF \
        ..

CMAKE_RET=$?
if [ $CMAKE_RET -ne 0 ]; then
        echo "cmake failed: $CMAKE_RET"
        exit $CMAKE_RET
fi

make -j$(nproc --all)

MAKE_RET=$?
if [ $MAKE_RET -ne 0 ]; then
        echo "make failed: $MAKE_RET"
        exit $MAKE_RET
fi

make -j$(nproc --all) test

MAKE_TEST_RET=$?
if [ $MAKE_TEST_RET -ne 0 ]; then
        echo "make test failed: $MAKE_TEST_RET"
        exit $MAKE_TEST_RET
fi

exit 0
