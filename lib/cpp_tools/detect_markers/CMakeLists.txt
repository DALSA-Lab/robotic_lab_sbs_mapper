cmake_minimum_required(VERSION 3.10)
project(detect_markers)

# Build executable
add_executable(markers_v2
    src/detect_markersv2.cpp
)

# Include local headers
target_include_directories(markers_v2 PRIVATE
    ../include
)

# Link OpenCV libraries (found by root CMake)
target_link_libraries(markers_v2 PRIVATE ${OpenCV_LIBS})