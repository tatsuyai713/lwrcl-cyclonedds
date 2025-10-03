#!/bin/bash

rm -rf ./build
mkdir -p build

cmake  . -B build -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds-libs -DCMAKE_PREFIX_PATH=/opt/cyclonedds/lib/cmake
cmake --build build
sudo cmake --install build