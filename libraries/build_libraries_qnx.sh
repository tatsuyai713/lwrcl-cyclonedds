#!/bin/bash

export PATH=/opt/qnx/cyclonedds/bin/:$PATH
export LD_LIBRARY_PATH=/opt/qnx/cyclonedds/lib/:$LD_LIBRARY_PATH
rm -rf ./build-qnx
cmake  . -B build-qnx -DCMAKE_TOOLCHAIN_FILE=./cmake/qnx_toolchain.cmake \
  -DCMAKE_INSTALL_PREFIX=/opt/qnx/cyclonedds-libs \
  -DCMAKE_PREFIX_PATH=/opt/qnx/cyclonedds/lib/cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DYAML_BUILD_SHARED_LIBS=ON \
  -DYAML_CPP_INSTALL=ON
cmake --build build-qnx
