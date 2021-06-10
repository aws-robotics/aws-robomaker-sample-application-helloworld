#!/usr/bin/env bash
set -x

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --install-ros) ros_distro_to_install=$2; shift ;;  
    esac
    shift
done

supported_ros_distros=("melodic")

install_ros(){
        echo "Installing ROS $ros_distro_to_install"
        #Install ROS Prerequisites
        apt update
        apt-get install -y lsb-release gnupg2 curl && apt-get clean all
        rm -f "/etc/apt/sources.list.d/ros-latest.list"
        sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - 
        apt update

        #Install ROS $ros_distro_to_install
        apt install -y ros-$ros_distro_to_install-desktop-full
        source /opt/ros/$ros_distro_to_install/setup.bash
}


setup_sample_app(){
        echo "Setting up sample app"
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
           
}

echo $ros_distro_to_install
if [ -z "$ros_distro_to_install" ];
then
        echo "No ROS installation selected."
        setup_sample_app
        exit 0
elif [[ ! " ${supported_ros_distros[@]} " =~ " ${ros_distro_to_install} " ]]; #check if item in list
then
        echo "The selected --install-ros <ros-distro> is not supported. If you want to try unsupported versions of ROS \
please install it yourself, and rerun this script without specifying --install-ros".
        exit 0
elif [ -d "/opt/ros/$ros_distro_to_install" ];
then
        echo "ROS $ros_distro_to_install is already installed. Skipping ROS Installation."
        source /opt/ros/$ros_distro_to_install/setup.bash
elif [ ! -d "/opt/ros" ];
then
        install_ros 
else    
        echo "Warning your installed ROS Distro is not supported."
fi

setup_sample_app
