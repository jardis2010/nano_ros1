#!/usr/bin/env bash
#
# Builds ROS container(s) by installing packages or from source (when needed)
# See help text below or run './scripts/docker_build_ros.sh --help' for options
#

set -e

die() {
    printf '%s\n' "$1"
    show_help
    exit 1
}

source scripts/opencv_version.sh

ROS_DISTRO="melodic"
ROS_PACKAGE="ros_base"
WITH_PYTORCH="off"
BASE_IMAGE="nvcr.io/nvidia/l4t-base:r32.7.1"
L4T_VERSION="32.7.1"

# opencv.csv mounts files that preclude us installing different version of opencv
# temporarily disable the opencv.csv mounts while we build the container
CV_CSV="/etc/nvidia-container-runtime/host-files-for-container.d/opencv.csv"

echo "ROS_DISTRO:   $ROS_DISTRO"
echo "ROS_PACKAGE:  $ROS_PACKAGE"
echo "WITH_PYTORCH: $WITH_PYTORCH"

BUILD_DISTRO=($ROS_DISTRO)
BUILD_PACKAGES=($ROS_PACKAGE)


# check for local version of PyTorch base container
BASE_IMAGE_PYTORCH="jetson-inference:r$L4T_VERSION"

if [[ "$(sudo docker images -q $BASE_IMAGE_PYTORCH 2> /dev/null)" == "" ]]; then
	BASE_IMAGE_PYTORCH="dustynv/$BASE_IMAGE_PYTORCH"
fi


build_ros()
{
	local distro=$1
	local package=$2
	local base_image=$3
	local extra_tag=$4
	local dockerfile=${5:-"Dockerfile.myros.$distro"}
	local container_tag="ros:${distro}-${extra_tag}l4t-r${L4T_VERSION}"
	
	echo ""
	echo "Building container $container_tag"
	echo "BASE_IMAGE=$base_image"
	echo ""
	
	sh ./scripts/docker_build.sh $container_tag $dockerfile \
			--build-arg ROS_PKG=$package \
			--build-arg BASE_IMAGE=$base_image \
			--build-arg OPENCV_URL=$OPENCV_URL \
			--build-arg OPENCV_DEB=$OPENCV_DEB
			
	# restore opencv.csv mounts
	if [ -f "$CV_CSV.backup" ]; then
		sudo mv $CV_CSV.backup $CV_CSV
	fi
}


for DISTRO in ${BUILD_DISTRO[@]}; do
	for PACKAGE in ${BUILD_PACKAGES[@]}; do
		build_ros $DISTRO $PACKAGE $BASE_IMAGE "`echo $PACKAGE | tr '_' '-'`-"
		
		if [[ "$WITH_PYTORCH" == "on" && "$DISTRO" != "melodic" && "$DISTRO" != "eloquent" ]]; then
			build_ros $DISTRO $PACKAGE $BASE_IMAGE_PYTORCH "pytorch-"
		fi
	done
done
