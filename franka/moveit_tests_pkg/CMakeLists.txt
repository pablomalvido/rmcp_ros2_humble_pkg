cmake_minimum_required(VERSION 3.8)
project(moveit_tests_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(move_cartesian_program
  src/move_cartesian_program.cpp)
target_include_directories(move_cartesian_program
  PUBLIC include)
ament_target_dependencies(move_cartesian_program rclcpp moveit_ros_planning_interface)

add_executable(move_target_program
  src/move_target_program.cpp)
target_include_directories(move_target_program
  PUBLIC include)
ament_target_dependencies(move_target_program rclcpp moveit_ros_planning_interface)

add_executable(move_named_target_example
  src/move_named_target_example.cpp)
target_include_directories(move_named_target_example
  PUBLIC include)
ament_target_dependencies(move_named_target_example rclcpp moveit_ros_planning_interface)

install(TARGETS  
  move_cartesian_program
  move_target_program
  move_named_target_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()