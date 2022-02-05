cmake_minimum_required(VERSION 2.8)
project( DisplayImage )

find_package( OpenCV REQUIRED )
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( Boost 1.55.0 COMPONENTS program_options REQUIRED )


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )
include_directories( ${Boost_INCLUDE_DIR} )


add_executable( DisplayImage main.cpp )

target_link_libraries( DisplayImage ${OpenCV_LIBS} )
target_link_libraries (DisplayImage ${Eigen_LIBRARIES})
target_link_libraries (DisplayImage ${Boost_INCLUDE_DIR})


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
