#!/bin/sh

COMMON_PKGS="libeigen3-dev"

if [ -z "$OPENCV" ]; then
  echo "Must set environment variable OPENCV"
  exit 1
fi

if [ $OPENCV -eq 2 ]; then
  echo "Using OpenCV 2 from Ubuntu"
  apt-get update -q
  apt-get install -y libopencv-highgui-dev libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev $COMMON_PKGS
elif [ $OPENCV -eq 3 ]; then
  echo "Using OpenCV 3 from PPA"
  add-apt-repository ppa:amarburg/opencv3 -y
  apt-get update -q
  apt-get install -y libopencv3-highgui-dev libopencv3-core-dev libopencv3-imgproc-dev libopencv3-imgcodecs-dev $COMMON_PKGS
fi
