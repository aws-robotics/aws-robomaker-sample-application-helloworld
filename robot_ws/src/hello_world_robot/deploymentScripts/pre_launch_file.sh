#!/bin/sh
# This is a script which can be run before running the ROS launch file.
# You can include a pre-check of robot environment in this pre-launch file. 
# Non-zero exit status from script would cause robot deployment failure. 
# Use "deploymentScripts/pre_launch_file.sh" as the preLaunchFile path.

echo Hello World, pre-launch
