cmake_minimum_required(VERSION 3.5)

project(turtlebot3_description_reduced_mesh)

find_package(ament_cmake_ros REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
