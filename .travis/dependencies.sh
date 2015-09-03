#!/bin/sh

sudo add-apt-repository ppa:amarburg/opencv3 -y
sudo apt-get update -q
sudo apt-get install libopencv-highgui-dev libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs.dev
