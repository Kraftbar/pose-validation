cmake_minimum_required(VERSION 3.16)
project(pose_validation_native LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV REQUIRED opencv4)

add_executable(simple_slam_opt simple_slam_opt.cpp)
add_executable(simple_slam_c simple_slam_c.c simple_slam_c_shim.cpp)

target_include_directories(simple_slam_opt PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(simple_slam_c PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_directories(simple_slam_opt PRIVATE ${OpenCV_LIBRARY_DIRS})
target_link_directories(simple_slam_c PRIVATE ${OpenCV_LIBRARY_DIRS})
target_link_libraries(simple_slam_opt PRIVATE ${OpenCV_LIBRARIES})
target_link_libraries(simple_slam_c PRIVATE ${OpenCV_LIBRARIES})
set_property(TARGET simple_slam_c PROPERTY LINKER_LANGUAGE CXX)

target_compile_options(simple_slam_opt PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -O3
  -DNDEBUG
)

target_compile_options(simple_slam_c PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -O3
  -DNDEBUG
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i.86")
  target_compile_options(simple_slam_opt PRIVATE -march=native)
  target_compile_options(simple_slam_c PRIVATE -march=native)
endif()
