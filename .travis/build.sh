#!/bin/sh

mkdir build
cd build
cmake --version
cmake $CMAKE_VARS ..
make && make test
