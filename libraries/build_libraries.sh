#!/bin/bash

export PATH=/opt/cyclonedds/bin/:$PATH
export LD_LIBRARY_PATH=/opt/cyclonedds/lib/:$LD_LIBRARY_PATH
rm -rf ./build
cmake  . -B build \
  -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds-libs \
  -DCMAKE_PREFIX_PATH=/opt/cyclonedds/lib/cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DYAML_BUILD_SHARED_LIBS=ON \
  -DYAML_CPP_INSTALL=ON
cmake --build build
sudo cmake --install build
