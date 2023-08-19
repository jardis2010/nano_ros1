#!/bin/sh

export ROS_PKG=ros_base
export ROS_DISTRO=melodic
export ROS_ROOT=/opt/ros/${ROS_DISTRO}
export MACHINE_TYPE=JetAuto
export JETSON_MODEL_NAME=JETSON_NANO

export CV_CSV="/etc/nvidia-container-runtime/host-files-for-container.d/opencv.csv"

apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        vim \
        nano \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates \
    && rm -rf /var/lib/apt/lists/*

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


# 
# install ROS packages
#
apt-get update && \
    apt-get install -y --no-install-recommends \
		ros-melodic-`echo "${ROS_PKG}" | tr '_' '-'` \
		ros-melodic-image-transport \
		ros-melodic-vision-msgs \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        usbutils \
        python3-pip \
        python3-yaml \
        python3-opencv \
        python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

pip3 install rospkg catkin_pkg smbus2 Jetson.GPIO pyserial

#
# init/update rosdep
#
apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep fix-permissions && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

apt-get update 
apt-get upgrade -y
apt-get install -y \
    ros-melodic-vision-opencv \
    ros-melodic-tf* \
    ros-melodic-joy \
    ros-melodic-control-msgs \
    ros-melodic-rviz
 
# Build Catkin workspace
cd /workspace \
     && mkdir -p /catkin_ws/src 
 
# for Orbbrc Astra Camera
# - dependencies
apt install  -y libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev \
    libusb-1.0-0-dev libeigen3-dev


# - Install libuvc
git clone https://github.com/libuvc/libuvc.git  \
    && cd libuvc  \
    && mkdir build  &&  cd build  \
    && cmake .. && make -j4  \
    && make install && ldconfig
 
mkdir -p /catkin_ws/src/third_party && cd /catkin_ws/src/third_party \
     && git clone https://github.com/orbbec/ros_astra_camera.git 

# make a directory for my robot
mkdir -p /catkin_ws/src/jetauto
# COPY ./jetauto/ /catkin_ws/src/jetauto/


# 
# setup entrypoint
#
cp ./packages/ros_entrypoint.sh /ros_entrypoint.sh
chmod +x /ros_entrypoint.sh
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc