#!/bin/bash
sudo apt update
sudo apt install -yq ros-melodic-tf ros-melodic-compressed-image-transport ros-melodic-image-transport ros-melodic-camera-info-manager ros-melodic-camera-calibration-parsers ros-melodic-dynamic-reconfigure ros-melodic-cv-bridge
sudo apt install -yq udev libudev-dev libzbar-dev libmagick++-dev python-dev python3-dev python-pip
sudo pip install imutils
cd $HOME/catkin_ws/src/amrl-vision/optris-drivers/3rd_party    
sudo dpkg --unpack libirimager-2.0.7-amd64.deb 
sudo rm /var/lib/dpkg/info/libirimager.postinst
sudo dpkg --configure libirimager
sudo ldconfig
