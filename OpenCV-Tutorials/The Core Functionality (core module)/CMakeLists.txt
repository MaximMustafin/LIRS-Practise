cmake_minimum_required(VERSION 2.8)

project( DisplayImage )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Mat_the_Basic_Image_Container src/mat_the_basic_image_container.cpp )
add_executable( How_to_scan_images src/how_to_scan_images.cpp )
add_executable( Mat_mask_operations src/mat_mask_operations.cpp )
add_executable( Adding_two_images src/adding_two_images.cpp )
add_executable( Basic_linear_transoforms src/basic_linear_transforms.cpp )
add_executable( Disscrete_fourier_transform src/discrete_fourier_transform.cpp )
add_executable( File_input_output src/file_input_output.cpp )

target_link_libraries( Mat_the_Basic_Image_Container ${OpenCV_LIBS} )
target_link_libraries( How_to_scan_images ${OpenCV_LIBS} )
target_link_libraries( Mat_mask_operations ${OpenCV_LIBS} )
target_link_libraries( Adding_two_images ${OpenCV_LIBS} )
target_link_libraries( Basic_linear_transoforms ${OpenCV_LIBS} )
target_link_libraries( Disscrete_fourier_transform ${OpenCV_LIBS} )
target_link_libraries( File_input_output ${OpenCV_LIBS} )