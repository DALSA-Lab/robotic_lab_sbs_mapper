cmake_minimum_required(VERSION 3.10)
project(compute_pose)

# Build executable
add_executable(compute_pose
    src/compute_pose.cpp
)

# Include local headers
target_include_directories(compute_pose PRIVATE
    ../include
)

# Link OpenCV libraries (found by root CMake)
target_link_libraries(compute_pose PRIVATE ${OpenCV_LIBS})