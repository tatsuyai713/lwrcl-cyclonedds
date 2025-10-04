#!/bin/bash

export PATH=/opt/cyclonedds/bin/:$PATH
export LD_LIBRARY_PATH=/opt/cyclonedds/lib/:$LD_LIBRARY_PATH
cd ./src/ros-data-types-cyclonedds/src
rm -rf ./build-qnx
cmake  . -B build-qnx -DCMAKE_TOOLCHAIN_FILE=../../cmake/qnx_toolchain.cmake -DCMAKE_INSTALL_PREFIX=/opt/qnx/cyclonedds-libs -DCMAKE_PREFIX_PATH=/opt/qnx/cyclonedds/lib/cmake
cmake --build build-qnx
sudo cmake --install build-qnx
