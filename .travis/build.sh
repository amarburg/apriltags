#!/bin/sh

HOME=$PWD

# Need to build googletest first
cd /usr/src/gtest
sudo cmake .
sudo make
sudo cp *.a /usr/lib

## Strangely, the libtbb2 package for precise doesn't create this symlink.
if [ ! -f /usr/lib/libtbb.so ]; then
	sudo ln -s /usr/lib/libtbb.so.2 /usr/lib/libtbb.so
fi

# Now build out package
cd $HOME
mkdir build
cd build
cmake --version
cmake $CMAKE_VARS -DOpenCV_DIR=/usr/share/OpenCV/ ..
make && make test
