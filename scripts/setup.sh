#!/usr/bin/env bash
set -ex

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --install-ros) ros_distro=$2; shift ;;  
    esac
    shift
done

supported_ros_distros=("melodic")

install_ros(){
        echo "Installing ROS $ros_distro" 
        #Install ROS Prerequisites
        apt update
        apt-get install -y lsb-release gnupg2 curl && apt-get clean all
        rm -f "/etc/apt/sources.list.d/ros-latest.list"
        sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - 
        apt update

        #Install ROS $ros_distro
        apt install -y ros-$ros_distro-desktop-full
        source /opt/ros/$ros_distro/setup.bash
}


setup_sample_app(){
        echo "Setting up sample application. Installing tools and dependencies."
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
        # need to install pyparsing==2.0.2 at the end to make sure that it overwrites 
        # pyparsing-3.0 to make sure colcon build works
        pip3 install -U pyparsing==2.0.2
        cd ..
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
        echo "No ROS Installation was found and no ROS Distro was specified.  Defaulting to installing ROS Melodic"
        ros_distro="melodic"
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
