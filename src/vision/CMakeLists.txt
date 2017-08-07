cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs 
    geometry_msgs
    cmake_modules
)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

# add_message_files(
#   FILES
# )

# add_service_files(
#     FILES
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs  # Or other packages containing msgs
# )


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES apriltags
    DEPENDS Eigen
)

add_library(apriltags
  apriltags/src/Edge.cc 
  apriltags/src/FloatImage.cc 
  apriltags/src/GLine2D.cc 
  apriltags/src/GLineSegment2D.cc 
  apriltags/src/Gaussian.cc 
  apriltags/src/GrayModel.cc 
  apriltags/src/Homography33.cc 
  apriltags/src/MathUtil.cc 
  apriltags/src/Quad.cc 
  apriltags/src/Segment.cc 
  apriltags/src/TagDetection.cc 
  apriltags/src/TagDetector.cc 
  apriltags/src/TagFamily.cc 
  apriltags/src/UnionFindSimple.cc
)

target_link_libraries(apriltags
  ${Eigen_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(circle_detector circle/src/circle_detector.cpp)
target_link_libraries(circle_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

include_directories(
    include
    apriltags/include
    apriltags/include/AprilTags
    circle/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(vision
    src/vision_node.cpp
)

add_dependencies(vision vision_generate_messages_cpp)

target_link_libraries(vision 
    ${catkin_LIBRARIES}  
    ${OpenCV_LIBRARIES} 
    apriltags
    circle_detector
)

