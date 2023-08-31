#!/bin/bash
set -e

ros_env_setup="/opt/ros/$ROS_DISTRO/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

# Build Catkin workspace
#cd /workspace \
#    && mkdir -p /catkin_ws/src \
#    && cd /catkin_ws \
#    && catkin_make 

# - Install libuvc
#git clone https://github.com/libuvc/libuvc.git && cd libuvc 
#rm -r build 
#mkdir build  &&  cd build  
#cmake .. && make -j4  
#make install && ldconfig

#mkdir /catkin_ws/src/third_party \
#    && cd /catkin_ws/src/third_party \
#    && git clone https://github.com/orbbec/ros_astra_camera.git \
#    && cd /catkin_ws \
#    && catkin_make

cd /workspace/jetauto_ws 
catkin_make

source ./devel/setup.bash 
#roscd astra_camera
#./scripts/create_udev_rules

#roslaunch astra_camera astra_pro_plus.launch
echo 'source /workspace/jetauto_ws/devel/setup.bash' >> ~/.bashrc
echo 'export HOST=jetauto_1' >> ~/.bashrc
echo 'export MACHINE_TYPE=JetAuto' >> ~/.bashrc
echo 'export DEPTH_CAMEA_TYPE=AstraProPlus' >> ~/.bashrc
echo 'export LIDAR_TYPE=A1' >> ~/.bashrc

exec "$@"
