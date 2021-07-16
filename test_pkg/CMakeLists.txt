cmake_minimum_required(VERSION 3.0.2)
project(test_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
)

add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} ${OpenCV_LIBS} )
