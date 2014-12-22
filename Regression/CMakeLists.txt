cmake_minimum_required (VERSION 2.8.11)
project (box2d11_regression)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

include_directories (
    ./googletest/include
    box2d-ref
    ../
    )

add_subdirectory (googletest)
add_subdirectory (../ box2d)
add_subdirectory (box2d-ref)

add_executable (regression_tests tests/helloworld.cpp)
target_link_libraries (regression_tests gtest Box2D Box2DRef)
