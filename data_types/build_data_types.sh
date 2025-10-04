#!/bin/bash

export PATH=/opt/cyclonedds/bin/:$PATH
export LD_LIBRARY_PATH=/opt/cyclonedds/lib/:$LD_LIBRARY_PATH
cd ./src/ros-data-types-cyclonedds/src
rm -rf ./build
cmake  . -B build -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds-libs -DCMAKE_PREFIX_PATH=/opt/cyclonedds/lib/cmake
cmake --build build
sudo cmake --install build
