#!/bin/sh

add-apt-repository ppa:amarburg/opencv3 -y
apt-get update -q
apt-get install -y libopencv3-highgui-dev libopencv3-core-dev libopencv3-imgproc-dev libopencv3-imgcodecs.dev
