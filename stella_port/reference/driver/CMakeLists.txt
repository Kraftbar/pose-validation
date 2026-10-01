# Builds run_stella_reference against an already-built+installed reference
# copy of stella_vslam (see tools/build_stella_reference.py, which copies
# external/candidates/stella_vslam, applies stella_port/reference/patches/,
# and installs to runs/stella_port/reference_build/install).
cmake_minimum_required(VERSION 3.10)
project(stella_reference_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(stella_vslam REQUIRED)
find_package(OpenCV 4.0 REQUIRED)

add_executable(run_stella_reference
    main.cc
    tum_rgbd_util.cc)
target_link_libraries(run_stella_reference PRIVATE stella_vslam::stella_vslam ${OpenCV_LIBS})
target_include_directories(run_stella_reference PRIVATE ${OpenCV_INCLUDE_DIRS})

# CMAKE_PREFIX_PATH also has to include external/candidates/deps/root (for
# g2o/Eigen/spdlog/...), and that prefix happens to carry its own, unpatched
# stella_vslam/ headers from an earlier build. Force our reference install's
# include dir ahead of everything stella_vslam::stella_vslam's transitive
# INTERFACE_INCLUDE_DIRECTORIES would otherwise put first, so the patched
# system.h/mapping_module.h/global_optimization_module.h win.
if(DEFINED STELLA_REFERENCE_INSTALL_DIR)
    target_include_directories(run_stella_reference BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
endif()
