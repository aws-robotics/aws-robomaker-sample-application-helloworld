#!/bin/bash

ament_flake8 ../simulation_ws/src/hello_world_simulation ../robot_ws/src/hello_world_robot
ament_pep257 ../simulation_ws/src/hello_world_simulation ../robot_ws/src/hello_world_robot
ament_xmllint ../simulation_ws/src/hello_world_simulation ../robot_ws/src/hello_world_robot