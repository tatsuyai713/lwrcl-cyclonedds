#!/bin/bash

export PATH=/opt/qnx/cyclonedds/bin/:$PATH
export LD_LIBRARY_PATH=/opt/qnx/cyclonedds/lib/:$LD_LIBRARY_PATH
rm -rf ./build-qnx
cmake  . -B build-qnx -DCMAKE_TOOLCHAIN_FILE=./cmake/qnx_toolchain.cmake
cmake --build build-qnx
