cmake_minimum_required(VERSION 3.10)
project(capture_images)

# Build executable
add_executable(capture
    src/capture_images.cpp
)

# Link OpenCV libraries (found by root CMake)
target_link_libraries(capture PRIVATE ${OpenCV_LIBS})