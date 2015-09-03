#!/bin/sh

mkdir build
cd build
cmake $CMAKE_VARS ..
make && make test
