#!/bin/sh

HOME=$PWD

# Need to build googletest first
cd /usr/src/gtest
sudo cmake .
sudo make
sudo cp *.a /usr/lib

if [ ! -f /usr/lib/libtbb.so ]; then
	sudo ln -s /usr/lib/libtbb.so.2 /usr/lib/libtbb.so
fi

cd $HOME
mkdir build
cd build
cmake --version
cmake $CMAKE_VARS ..
make && make test
