cmake_minimum_required(VERSION 2.8.3)
project(uav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dji_sdk
  std_msgs
  geometry_msgs
  actionlib_msgs 
  actionlib
)

find_package(OpenCV REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# add_message_files(
#   FILES
#   Message1.msg
# )

add_service_files(
  FILES
  Charge.srv
)

#add_action_files(
#  FILES
#)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
 	${catkin_INCLUDE_DIRS}
)

add_executable(uav src/uav.cpp
  src/cfg.cpp 
  src/pid.cpp 
  src/uart.cpp 
  src/crc8.cpp
)

target_link_libraries(uav
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#add_dependencies(uav dji_sdk_generate_messages_cpp)