#!/bin/bash

set -e

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

echo "Cloning Amrl-Vision ..."
#git clone https://github.com/mrl-amrl/amrl-vision.git

echo "Installing drivers and necessary files ..."
apt install -yq ros-melodic-tf ros-melodic-compressed-image-transport ros-melodic-image-transport ros-melodic-camera-info-manager ros-melodic-camera-calibration-parsers ros-melodic-dynamic-reconfigure ros-melodic-cv-bridge
apt install -yq udev libudev-dev libzbar-dev libmagick++-dev python-dev python3-dev python-pip
pip install imutils
cd ~/catkin_ws/src/amrl-vision/optris_drivers/config && wget http://mrl-amrl.ir/assets/optris_config.zip && unzip optris_config.zip
cd ~/catkin_ws/src/amrl-vision/optris_drivers/3rd_party
dpkg --unpack libirimager-2.0.7-amd64.deb
rm /var/lib/dpkg/info/libirimager.postinst
dpkg --configure libirimager
ldconfig

#Please add OpenCV installation to this shell script
