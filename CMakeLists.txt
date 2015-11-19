cmake_minimum_required(VERSION 2.8)
project(robostat)
find_package(OpenCV 3.0.0 EXACT REQUIRED )
find_package(Boost REQUIRED COMPONENTS
    thread
    system
)
find_package(GTSAM)
find_package(GTSAMCMakeTools)
set( NAME_SRC 
    featuredpa.cpp
    DynamicFactorGraph.cpp
    main.cpp    
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include ${GTSAM_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories( ${CMAKE_BINARY_DIR}/bin ${GTSAM_LINK_DIR})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable( robostat ${NAME_SRC} ${NAME_HEADERS} )

target_link_libraries( robostat ${OpenCV_LIBS} ${GTSAM_LIBS} ${Boost_LIBRARIES} tbb tbbmalloc gtsam)
