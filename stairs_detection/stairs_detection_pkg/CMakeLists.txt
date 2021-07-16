cmake_minimum_required(VERSION 3.0.2)
project(stairs_detection_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  camera_info_manager
  dynamic_reconfigure
  tf
  cmake_modules
)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/stairs_detector.cfg
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_definitions(${EIGEN_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS dynamic_reconfigure
)

add_executable(depth_converter_node src/depth_converter_node.cpp)
target_link_libraries(depth_converter_node ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(stairs_detector_node src/stairs_detector_node.cpp src/stairs_detector.cpp)
target_link_libraries(stairs_detector_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS 
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)