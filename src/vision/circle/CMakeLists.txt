cmake_minimum_required(VERSION 2.8.3)
project(uav_vision)

set (Park_NAME uav_park_node)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs 
    geometry_msgs
)
find_package(OpenCV REQUIRED)

# add_message_files(
#   FILES
#   ParkPosition.msg
# )

# add_service_files(
#     FILES
#     park.srv
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs  # Or other packages containing msgs
# )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES video_capture
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${Park_NAME}
    src/uav_vision_node.cpp
    src/uav_comm.cpp
    src/uav_park.cpp
)

add_dependencies(${Park_NAME} uav_vision_generate_messages_cpp)

target_link_libraries(${Park_NAME} ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} )

