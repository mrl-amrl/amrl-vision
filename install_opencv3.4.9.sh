#!/bin/sh
echo "Installing dependency of OpenCV"
sudo apt-get update
sudo apt-get install -y build-essential cmake unzip pkg-config libjpeg-dev libpng-dev libtiff-dev
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install -y libjasper1 libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran
echo "Downloading OpenCV file"
mkdir -p /tmp/cv
wget -O /tmp/cv/opencv.zip https://github.com/opencv/opencv/archive/3.4.9.zip
wget -O /tmp/cv/opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.9.zip
cd /tmp/cv
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-3.4.9 opencv
mv opencv_contrib-3.4.9 opencv_contrib
cd opencv/
mkdir build && cd build
echo "making OpenCV"
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/cv/opencv_contrib/modules \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D BUILD_EXAMPLES=OFF ..

make -j4
sudo make install -y
sudo ldconfig
sudo rm -rf opencv.zip opencv_contrib.zip
