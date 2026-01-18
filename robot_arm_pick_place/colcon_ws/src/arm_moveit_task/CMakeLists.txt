cmake_minimum_required(VERSION 3.8)
project(arm_moveit_task)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  arm_moveit_task/pick_place_task.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_task
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
