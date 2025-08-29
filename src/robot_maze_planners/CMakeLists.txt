cmake_minimum_required(VERSION 3.8)
project(robot_maze_planners)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)

ament_python_install_module(
  PACKAGE robot_maze_planners
)

install(DIRECTORY
  robot_maze_planners
  resource
  launch
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
