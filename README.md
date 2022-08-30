# AWS RoboMaker Sample Application - Hello World

This demonstrates the structure of a ROS project that works with RoboMaker by creating a robot spinning in an empty world.

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://github.com/aws-robotics/aws-robomaker-sample-application-helloworld)._

## Installing Requirements

The following will be installed:
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html) - Used for building and bundling the application.
- [vcstool](https://github.com/dirk-thomas/vcstool#how-to-install-vcstool) - Used to pull in sample app dependencies that are only available from source, not from apt or pip.
- [rosdep](http://wiki.ros.org/rosdep#Installing_rosdep) - rosdep is a command-line tool for installing system dependencies of ROS packages.
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - Other versions may work, however they have not been tested. ROS installation will be skipped if a ROS distro is already installed.

```bash
source scripts/setup.sh
```

## Build

- *Robot Application*
```bash
cd robot_ws
colcon build
```

- *Simulation Application*
```bash
cd simulation_ws
colcon build
```

## Run
The `TURTLEBOT3_MODEL` environment variable is optional when running both robot and simulation application. Default value is `waffle_pi`. Valid values are `burger`, `waffle`, and `waffle_pi`. Set it by

```bash
export TURTLEBOT3_MODEL=<robot-model>
```

Launch the application with the following commands:

- *Running Robot Application on a Robot*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch hello_world_robot deploy_rotate.launch
    ```

- *Running Robot Application in a Simulation*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch hello_world_robot rotate.launch
    ```

- *Running Simulation Application*
    ```bash
    source simulation_ws/install/local_setup.sh
    roslaunch hello_world_simulation empty_world.launch
    ```

Note that when running robot applications on a robot, `use_sim_time` should be set to `false` (which is the default value in `deploy_rotate.launch.py`). When running robot applications along with simulation applications, `use_sim_time` should be set to `true` for both applications (which is the default value in both `rotate.launch.py` and `empty_word.launch.py`).
   		  
When running simulation applications, run command with `gui:=true` to run gazebo client for visualization

## Run with a WorldForge world

After exporting a world from WorldForge, we can unzip the content and move under simulation_ws package:

```bash
unzip exported_world.zip
mv aws_robomaker_worldforge_pkgs simulation_ws/src/
```

Build it again

```bash
cd simulation_ws
colcon build
```

Launch the application with the following commands:

```bash
source simulation_ws/install/local_setup.sh
roslaunch hello_world_simulation worldforge_world.launch
```

By default, WorldForge packages will load the exported world. To override, specify the environment variable `WORLD_ID`. 

```bash
# use worldId found in "src/aws_robomaker_worldforge_worlds/worlds"
# e.g, generation_05wq8sybdcn2_world_1
export WORLD_ID=<worldId>  
```

## Using this sample with RoboMaker

You first need to install [Docker](https://docs.docker.com/get-docker/) and [VCS Import Tool](http://wiki.ros.org/vcstool) (if you use VCS Import Tool). Python 3.5 or above is required.

```bash
pip3 install vcstool
```

After Docker and VCS Import Tool is installed you need to build your robot or simulation docker images:

```bash
# Import dependencies defined in .rosinstall to each source directory using vcs import
vcs import robot_ws < robot_ws/.rosinstall
vcs import simulation_ws < simulation_ws/.rosinstall

# Building Robot Application Docker Image
DOCKER_BUILDKIT=1 docker build . \
--build-arg ROS_DISTRO=melodic \
--build-arg LOCAL_WS_DIR=./robot_ws \
--build-arg APP_NAME=helloworld-robot-app \
-t robomaker-helloworld-robot-app

# Building Simulation Application Docker Image
DOCKER_BUILDKIT=1 docker build . \
--build-arg GAZEBO_VERSION=gazebo-9 \
--build-arg ROS_DISTRO=melodic \
--build-arg LOCAL_WS_DIR=./simulation_ws \
--build-arg APP_NAME=helloworld-sim-app \
-t robomaker-helloworld-sim-app
```

This produces the Docker Images `robomaker-helloworld-robot-app` and `robomaker-helloworld-sim-app` respectively which you can view by running:

```bash
# Listing your Docker Images
docker images
```

You'll need to [upload these images to Amazon ECR](https://docs.aws.amazon.com/robomaker/latest/dg/development-publish-app-containers.html), then you can use these files to
[create a robot application](https://docs.aws.amazon.com/robomaker/latest/dg/create-robot-application.html),
[create a simulation application](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-application.html),
and [create a simulation job](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-job.html) in RoboMaker. Visit the [preparing-ros-application-and-simulation-containers-for-aws-robomaker](https://aws.amazon.com/blogs/robotics/preparing-ros-application-and-simulation-containers-for-aws-robomaker/#:~:text=Bash-,Publish%20docker%20images%20to%20Amazon%20ECR,-Containers%20used%20by) blog post to find the steps to upload these docker images to Amazon ECR.

## ROS Nodes launched by this Sample

### Nodes created by this sample

```
/rotate
```

## ROS Topics used by this Sample

```
/clock
/cmd_vel
```

## License

MIT-0 - See LICENSE for further information

## How to Contribute

Create issues and pull requests against this Repository on Github
