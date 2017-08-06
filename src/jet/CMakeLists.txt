cmake_minimum_required(VERSION 2.8.3)
project(jet)

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
  CmdGrabber.srv
  StatGrabber.srv
)

add_action_files(
  FILES
  JetNav.action
)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  nav_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
 	${catkin_INCLUDE_DIRS}
)

add_executable(jet src/jet.cpp
  src/pid.cpp 
  src/uart.cpp 
  src/crc8.cpp
  src/kbhit.cpp
)

add_dependencies(jet
  jet_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(jet
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_dependencies(jet jet_generate_messages_cpp)
