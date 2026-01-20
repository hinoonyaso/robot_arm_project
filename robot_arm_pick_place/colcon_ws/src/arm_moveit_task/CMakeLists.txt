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

install(PROGRAMS
  arm_moveit_task/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME perception_node
)

install(PROGRAMS
  arm_moveit_task/color_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME color_detector
)

install(PROGRAMS
  arm_moveit_task/pose_estimator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_estimator
)

install(PROGRAMS
  arm_moveit_task/grasp_candidate_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME grasp_candidates
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
