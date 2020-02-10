#!/bin/bash
g++ opencv_version.cpp -o opencv_version `pkg-config --libs opencv`
./opencv_version
rm opencv_version