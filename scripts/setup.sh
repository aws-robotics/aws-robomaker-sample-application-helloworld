#!/usr/bin/env bash
set -ex

if [ -d "/opt/ros/melodic" ];
then
        echo "ROS Melodic is already installed. Skipping ROS Installation."
        source /opt/ros/melodic/setup.bash

elif [ ! -d "/opt/ros" ];
then
        echo "Installing ROS Melodic."
	#Install ROS Prerequisites
	apt update
	apt-get install -y lsb-release gnupg2 curl && apt-get clean all
        rm -f "/etc/apt/sources.list.d/ros-latest.list"
        sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - 
        apt update

        #Install ROS Melodic
        apt install -y ros-melodic-desktop-full
        source /opt/ros/melodic/setup.bash

else    
        #Warning your ROS Distro $ROS_DISTRO is not supported for the Hello World Sample Application
        #it may not work.
        echo "Warning your ROS Distro is not supported for AWS RoboMaker Hello World Sample Application ros1 branch."
fi

#setup key for colcon bundle
apt-key adv --fetch-keys 'http://packages.osrfoundation.org/gazebo.key'

apt update
apt install -y python-rosdep git
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ];
then
        rosdep init
fi
rosdep update

apt-get install -y python3-apt python3-pip python3-vcstool
pip3 install -U setuptools colcon-common-extensions colcon-ros-bundle

cd robot_ws
vcs import < .rosinstall
rosdep install --from-paths src --ignore-src -r -y

cd ..

cd simulation_ws
vcs import < .rosinstall
rosdep install --from-paths src --ignore-src -r -y

