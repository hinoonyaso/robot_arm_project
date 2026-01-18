cmake_minimum_required(VERSION 3.8)
project(arm_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
