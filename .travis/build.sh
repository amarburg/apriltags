#!/bin/sh

HOME=$PWD

# Need to build googletest first
cd /usr/src/gtest
sudo cmake .
sudo make
sudo cp *.a /usr/lib


cd $HOME
mkdir build
cd build
cmake --version
cmake $CMAKE_VARS ..
make && make test
