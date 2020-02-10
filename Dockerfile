FROM ubuntu:bionic

RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get install -y software-properties-common && \
    add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"

RUN apt-get install -y build-essential cmake unzip pkg-config libjpeg-dev libpng-dev libtiff-dev \
    libjasper1 libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev libatlas-base-dev gfortran python3.6-dev python-dev

RUN apt-get install -y wget curl git zip unzip python3-pip python-pip

WORKDIR /tmp
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.2.0.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.2.0.zip

RUN unzip opencv.zip && unzip opencv_contrib.zip && \
    mv opencv-3.2.0 opencv && \
    mv opencv_contrib-3.2.0 opencv_contrib

RUN apt-get install -y python-numpy python3-numpy

RUN mkdir -p /tmp/opencv/build
WORKDIR /tmp/opencv/build
RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D ENABLE_PRECOMPILED_HEADERS=OFF \
    -D WITH_EIGEN=OFF \
    -D BUILD_EXAMPLES=OFF ..

RUN make -j4
RUN make install && ldconfig && pkg-config --modversion opencv
RUN rm -rf /tmp/opencv.zip /tmp/opencv_contrib.zip /tmp/opencv /tmp/opencv_contrib

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get install -y ca-certificates tzdata
RUN update-ca-certificates
RUN ln -fs /usr/share/zoneinfo/Asia/Tehran /etc/localtime && dpkg-reconfigure -f noninteractive tzdata

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y ros-melodic-ros-base
RUN rosdep init && sudo rosdep fix-permissions && rosdep update
RUN apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential