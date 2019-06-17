cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )




file(GLOB project_SRC
    "./utils/*.cpp"    
    "./utils/*.h"
    "./classes/*.h"
)

add_executable( DisplayImage main.cpp   ${project_SRC})
target_link_libraries( DisplayImage ${OpenCV_LIBS} ${Boost_LIBRARIES})

# 
add_executable( estEss estimateE.cpp   ${project_SRC})

target_link_libraries( estEss ${OpenCV_LIBS} ${Boost_LIBRARIES})


