cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Inlcude Directories
include_directories(vendor/glm)

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

# Adding all source files
file(GLOB_RECURSE CPP_SOURCES "${CMAKE_SOURCE_DIR}/src/*.cpp")

# Adding all include files
file(GLOB_RECURSE HEADERS "${CMAKE_SOURCE_DIR}/include/*.h")

# Create the executable
add_executable(path_planner ${CPP_SOURCES} ${HEADERS})

ament_target_dependencies(path_planner rclcpp std_msgs)

# Add the header's folder
target_include_directories(path_planner
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Making visible to the command "ros2 run ..."
install(TARGETS
  path_planner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
