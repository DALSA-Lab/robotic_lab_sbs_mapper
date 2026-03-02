cmake_minimum_required(VERSION 3.10)
project(cpp_tools)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies once for all subprojects
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

# Add each tool as a subdirectory
add_subdirectory(detect_markers)
add_subdirectory(capture_images)
add_subdirectory(calib_hand_eye)
add_subdirectory(compute_pose)
add_subdirectory(compute_pose_all_options)