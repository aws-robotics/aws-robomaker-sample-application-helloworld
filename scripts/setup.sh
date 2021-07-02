#!/usr/bin/env bash
set -ex

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --install-ros) ros_distro=$2; shift ;;  
    esac
    shift
done

supported_ros_distros=("foxy")

install_ros(){
        echo "Installing ROS $ros_distro" 
        export DEBIAN_FRONTEND=noninteractive

        #https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
        #Install ROS Prerequisites
        locale  # check for UTF-8

        apt update && apt -y install locales
        locale-gen en_US en_US.UTF-8
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8

        locale  # verify settings

        apt update &&  apt install -y curl gnupg2 lsb-release
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

        apt update

        # #Install ROS $ros_distro
        apt install -y ros-$ros_distro-desktop
        source /opt/ros/$ros_distro/setup.bash

        #install gazebo
        apt -y install ros-$ros_distro-gazebo-ros-pkgs

}


setup_sample_app(){
        echo "Setting up sample application. Installing tools and dependencies."
        #setup key for colcon bundle
        apt-key adv --fetch-keys 'http://packages.osrfoundation.org/gazebo.key'
        apt update
        apt install -y python3-rosdep git wget
        if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ];
        then
                rosdep init
        fi
        rosdep update

        apt-get install -y python3-apt python3-pip python3-vcstool python3-testresources
        pip3 install -U pytest setuptools colcon-ros-bundle

        cd robot_ws
        vcs import < .rosinstall
        rosdep install --from-paths src --ignore-src -r -y
        cd ..

        cd simulation_ws
        vcs import < .rosinstall
        rosdep install --from-paths src --ignore-src -r -y
        cd ..

        #https://github.com/colcon/colcon-bundle/issues/100
        wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
}

if [ -d "/opt/ros" ];
then
        echo "A ROS installation already exists in your environment."
        if [ ! -z "$ros_distro" ];
        then
                echo "Ignoring request to install $ros_distro because a ROS installation already exists in your environment."
        fi

        found_supported_ros=false
        #check if their ros installation(s) are supported
        for distro in $supported_ros_distros
        do
                if [ -d "/opt/ros/$distro" ]
                then
                        source /opt/ros/$distro/setup.bash
                        #save to verify installation below
                        ros_distro=$distro
                        found_supported_ros=true
                        break
                fi
        done

        #if supported versions are not found
        if [ ! found_supported_ros ]
        then
                echo "ERROR: your installed ROS Distro(s) is not supported.  Exiting." 
                exit 1
        fi

elif [ -z "$ros_distro" ];
then
        echo "No ROS Installation was found and no ROS Distro was specified.  Defaulting to installing ROS Foxy"
        ros_distro="foxy"
        install_ros 
        source /opt/ros/$ros_distro/setup.bash
elif [[  " ${supported_ros_distros[@]} " =~ " ${ros_distro} " ]]; #check if item in list
then
        echo "No ROS Installation found.  Installing $ros_distro"
        install_ros 
        source /opt/ros/$ros_distro/setup.bash
elif [[ ! " ${supported_ros_distros[@]} " =~ " ${ros_distro} " ]]; #check if item in list
then
        echo "The selected ROS Distro $ros_distro is not supported. Exiting."
        exit 0
fi

#Verify Installation
if [ $ROS_DISTRO != $ros_distro ];
then
        echo "The ROS installation was unsuccessful, Sample Application setup cannot continue.  Exiting."
        exit 1
else
        echo "ROS Installation was successful, continuing with Sample Application setup."
fi


setup_sample_app