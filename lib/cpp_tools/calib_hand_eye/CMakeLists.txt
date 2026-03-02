cmake_minimum_required(VERSION 3.10)
project(calib_hand_eye)

# Build executable
add_executable(calib
    src/calib_cam_hand_eye.cpp
)


# Link OpenCV libraries (found by root CMake)
target_link_libraries(calib PRIVATE ${OpenCV_LIBS})