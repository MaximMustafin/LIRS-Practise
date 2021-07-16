cmake_minimum_required(VERSION 2.8)

project( ImageProcessing )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Find_contours_demo src/findContours_demo.cpp )
add_executable( Hull_demo src/hull_demo.cpp )
add_executable( General_contours_demo1 src/general_contours_demo1.cpp )

target_link_libraries( Find_contours_demo ${OpenCV_LIBS} )
target_link_libraries( Hull_demo ${OpenCV_LIBS} )
target_link_libraries( General_contours_demo1 ${OpenCV_LIBS} )