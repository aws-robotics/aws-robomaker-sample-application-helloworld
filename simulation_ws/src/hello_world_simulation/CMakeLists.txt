cmake_minimum_required(VERSION 3.5)

project(hello_world_simulation)

find_package(ament_cmake_ros REQUIRED)
find_package(turtlebot3_description REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Copy the rviz model for easier access in AWS RoboMaker RViz 
install(FILES ${turtlebot3_description_DIR}/../rviz/model.rviz
  DESTINATION share/rviz
  RENAME turtlebot3_model.rviz
)

ament_package()
