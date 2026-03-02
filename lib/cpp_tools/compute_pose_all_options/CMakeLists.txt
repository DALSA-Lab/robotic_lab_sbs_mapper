cmake_minimum_required(VERSION 3.10)
project(compute_pose_all_options)

# Build executable
add_executable(compute_pose_all_options
    src/compute_pose_all_options.cpp
)

# Include local headers
target_include_directories(compute_pose_all_options PRIVATE
    ../include
)

# Link OpenCV libraries (found by root CMake)
target_link_libraries(compute_pose_all_options PRIVATE ${OpenCV_LIBS})