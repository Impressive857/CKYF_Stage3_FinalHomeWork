cmake_minimum_required(VERSION 3.8)
project(answer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EXEC_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(EXEC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(EXEC_LIBRARY_DIR ${PROJECT_SOURCE_DIR}/lib)

file(GLOB_RECURSE EXEC_SRC ${EXEC_SRC_DIR}/*.cpp)
file(GLOB_RECURSE EXEC_INCLUDE ${EXEC_INCLUDE_DIR}/*.h)

set(EXEC_DEPEND rclcpp OpenCV cv_bridge sensor_msgs geometry_msgs example_interfaces info_interfaces)
set(ALL_EXEC img2map_node navigation_node)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${EXEC_INCLUDE_DIR}/${PROJECT_NAME})

ament_auto_add_executable(img2map_node ${EXEC_SRC_DIR}/img2map_node.cpp)
ament_target_dependencies(img2map_node ${EXEC_DEPEND})

ament_auto_add_executable(navigation_node ${EXEC_SRC_DIR}/navigation_node.cpp)
ament_target_dependencies(navigation_node ${EXEC_DEPEND})

install(TARGETS ${ALL_EXEC} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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

ament_auto_package()