cmake_minimum_required(VERSION 3.8)
project(mpt_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options( -Wextra -Wpedantic)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)
find_package(FCL REQUIRED)
find_package(Assimp REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionPlanRequest.msg"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  external/nigh/src
  external/mpt/src
)

add_executable(node src/mpt_ros_node.cpp)
ament_target_dependencies(node rclcpp std_msgs)

add_executable(client src/mpt_ros_client.cpp)
ament_target_dependencies(client rclcpp std_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(node
  Eigen3::Eigen
  ${FCL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  "${cpp_typesupport_target}"
  )
target_link_libraries(client
  Eigen3::Eigen
  "${cpp_typesupport_target}"
  )

install(TARGETS
client
node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()