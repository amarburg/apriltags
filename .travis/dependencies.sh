#!/bin/sh

add-apt-repository ppa:amarburg/opencv3 -y
apt-get update -q
apt-get install -y libopencv-highgui-dev libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs.dev
