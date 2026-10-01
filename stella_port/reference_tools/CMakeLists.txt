# Builds dump_frame_bow against an already-built+installed reference copy
# of stella_vslam (see tools/build_stella_reference.py, which builds
# runs/stella_port/reference_build/install -- the same install the
# stella_port/reference/driver links against). Mirrors that driver's
# CMakeLists.txt (same include-order fix for the patched headers).
cmake_minimum_required(VERSION 3.10)
project(stella_reference_frame_bow_tool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(stella_vslam REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(fbow REQUIRED)

add_executable(dump_frame_bow
    dump_frame_bow.cc
    tum_rgbd_util.cc)
target_link_libraries(dump_frame_bow PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
target_include_directories(dump_frame_bow PRIVATE ${OpenCV_INCLUDE_DIRS})

# Module 3: monocular initializer dumper (see dump_stella_init.cc header
# comment). Does not need fbow directly (no vocab arg used for BoW), but
# links it anyway since system::system() requires a vocab path to load.
add_executable(dump_stella_init
    dump_stella_init.cc
    tum_rgbd_util.cc)
target_link_libraries(dump_stella_init PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
target_include_directories(dump_stella_init PRIVATE ${OpenCV_INCLUDE_DIRS})

# Module 4a: map data model + monocular initial-map creation dumper (see
# dump_stella_map_init.cc header comment). Needs the real global bundle
# adjuster (g2o), already linked into the installed reference library.
add_executable(dump_stella_map_init
    dump_stella_map_init.cc
    tum_rgbd_util.cc)
target_link_libraries(dump_stella_map_init PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
target_include_directories(dump_stella_map_init PRIVATE ${OpenCV_INCLUDE_DIRS})

# Module 4b: pose_optimizer_g2o call dumper (see dump_stella_g2o_pose.cc
# header comment). Needs the 0006-pose-optimizer-g2o-trace.patch trace
# hooks built into the installed reference library.
add_executable(dump_stella_g2o_pose
    dump_stella_g2o_pose.cc
    tum_rgbd_util.cc)
target_link_libraries(dump_stella_g2o_pose PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
target_include_directories(dump_stella_g2o_pose PRIVATE ${OpenCV_INCLUDE_DIRS})

# Module 4b part 2: BA call dumper (see dump_stella_g2o_ba.cc header comment).
# Needs the 0007-ba-trace.patch hooks built into the installed reference
# library.
add_executable(dump_stella_g2o_ba
    dump_stella_g2o_ba.cc
    tum_rgbd_util.cc)
target_link_libraries(dump_stella_g2o_ba PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
target_include_directories(dump_stella_g2o_ba PRIVATE ${OpenCV_INCLUDE_DIRS})

# Module 4b part 2: re-solve captured global BA graphs with the REAL
# g2o LinearSolverEigen (see replay_ba_eigen.cc).
add_executable(replay_ba_eigen replay_ba_eigen.cc)
target_link_libraries(replay_ba_eigen PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
target_include_directories(replay_ba_eigen PRIVATE ${OpenCV_INCLUDE_DIRS})

find_package(yaml-cpp QUIET)
add_executable(debug_decompose debug_decompose.cc)
target_link_libraries(debug_decompose PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(debug_decompose PRIVATE yaml-cpp::yaml-cpp)
endif()

add_executable(debug_h21 debug_h21.cc)
target_link_libraries(debug_h21 PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS})
if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(debug_h21 PRIVATE yaml-cpp::yaml-cpp)
endif()
target_include_directories(debug_h21 PRIVATE ${OpenCV_INCLUDE_DIRS})

if(DEFINED STELLA_REFERENCE_INSTALL_DIR)
    target_include_directories(dump_frame_bow BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
    target_include_directories(dump_stella_init BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
    target_include_directories(dump_stella_map_init BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
    target_include_directories(debug_h21 BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
    target_include_directories(dump_stella_g2o_pose BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
    target_include_directories(dump_stella_g2o_ba BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
    target_include_directories(replay_ba_eigen BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
endif()

# Module 7 (loop closing): bit-exactness replay of stella_port/c/sv_g2o_sim3.c (pose graph + transform
# optimizer) against the real g2o / stella_vslam classes (see replay_sim3_opt.cc). The C sources are built as C99
# with the same floating-point flags as the harnesses.
enable_language(C)
add_library(sv_sim3_c OBJECT
    ../c/sv_g2o_sim3.c ../c/sv_sim3.c ../c/sv_eigen_lu3.c ../c/sv_linalg.c ../c/sv_eigen_quaternion.c
    ../c/sv_eigen_llt.c ../c/sv_eigen_amd.c ../c/sv_g2o_edge.c ../c/sv_g2o_se3.c)
set_target_properties(sv_sim3_c PROPERTIES C_STANDARD 99 C_EXTENSIONS OFF)
target_compile_options(sv_sim3_c PRIVATE -O2 -ffp-contract=off -fno-fast-math)
add_executable(replay_sim3_opt replay_sim3_opt.cc $<TARGET_OBJECTS:sv_sim3_c>)
target_link_libraries(replay_sim3_opt PRIVATE stella_vslam::stella_vslam fbow::fbow ${OpenCV_LIBS} m)
target_include_directories(replay_sim3_opt PRIVATE ${OpenCV_INCLUDE_DIRS})
if(DEFINED STELLA_REFERENCE_INSTALL_DIR)
    target_include_directories(replay_sim3_opt BEFORE PRIVATE "${STELLA_REFERENCE_INSTALL_DIR}/include")
endif()
