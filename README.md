# AWS RoboMaker Sample Application - Hello World

This demonstrates the structure of a ROS project that works with RoboMaker by creating a robot spinning in an empty world.

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://s3.console.aws.amazon.com/s3/buckets/robomaker-applications-us-east-1-72fc243f9355/hello-world/?region=us-east-1)._

## Requirements

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) / [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - Other versions may work, however they have not been tested
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html) - Used for building and bundling the application. 

## Build

### Pre-build commands

```bash
sudo apt-get update
rosdep update
```

### Robot

```bash
cd robot_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Simulation

```bash
cd simulation_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Run

Launch the application with the following commands:

- *Running Robot Application on a Robot*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch hello_world_robot deploy_rotate.launch
    ```

- *Running Robot Application Elsewhere*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch hello_world_robot rotate.launch
    ```

- *Running Simulation Application*
    ```bash
    source simulation_ws/install/local_setup.sh
    roslaunch hello_world_simulation empty_world.launch
    ```

## Using this sample with RoboMaker

You first need to install colcon-ros-bundle. Python 3.5 or above is required. 

```bash
pip3 install -U setuptools
pip3 install colcon-ros-bundle
```

After colcon-ros-bundle is installed you need to build your robot or simulation, then you can bundle with:

```bash
# Bundling Robot Application
cd robot_ws
source install/local_setup.sh
colcon bundle

# Bundling Simulation Application
cd simulation_ws
source install/local_setup.sh
colcon bundle
```

This produces the artifacts `robot_ws/bundle/output.tar` and `simulation_ws/bundle/output.tar` respectively. 
You'll need to upload these to an s3 bucket, then you can use these files to 
[create a robot application](https://docs.aws.amazon.com/robomaker/create-robot-application.html),  
[create a simulation application](https://docs.aws.amazon.com/robomaker/create-simulation-application.html), 
and [create a simulation job](https://docs.aws.amazon.com/robomaker/create-simulation-job.html) in RoboMaker.

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

MIT-0 - See LICENSE.txt for further information

## How to Contribute

Create issues and pull requests against this Repository on Github


