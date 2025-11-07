cmake_minimum_required(VERSION 3.10)
project(cpp_pkg)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES Off)

include_directories(include)

add_library(math_utils src/math_utils.cpp)
add_library(string_utils src/string_utils.cpp)

add_executable(test_main tests/test_main.cpp)

target_link_libraries(test_main PRIVATE math_utils string_utils)

# cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER_TARGET=x86_64-w64-windows-gnu -DCMAKE_CXX_COMPILER_TARGET=x86_64-w64-windows-gnu -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_CLANG_TIDY="clang-tidy;-header-filter=.;-checks=*;"
# cmake --build build -j

# cmake  -S . -B build -G "Ninja" -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_CXX_CLANG_TIDY="clang-tidy;-header-filter=.;-checks=*;"