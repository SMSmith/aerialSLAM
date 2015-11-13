project(robostat)
find_package(OpenCV 3.0.0 EXACT REQUIRED )
set( NAME_SRC 
    featuredpa.cpp
    main.cpp    
)

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include )
link_directories( ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable( robostat ${NAME_SRC} ${NAME_HEADERS} )

target_link_libraries( robostat ${OpenCV_LIBS} )
