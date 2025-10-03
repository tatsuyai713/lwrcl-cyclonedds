#!/bin/bash

rm -rf ./build-qnx
mkdir -p build-qnx

cmake  . -B build-qnx -DCMAKE_TOOLCHAIN_FILE=./cmake/qnx_toolchain.cmake -DCMAKE_INSTALL_PREFIX=/opt/qnx/cyclonedds-libs -DCMAKE_PREFIX_PATH=/opt/qnx/cyclonedds/lib/cmake
cmake --build build-qnx
sudo cmake --install build-qnx