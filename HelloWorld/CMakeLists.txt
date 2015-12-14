# Hello World examples
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++14")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

include_directories (${Box2D_SOURCE_DIR})
add_executable(HelloWorld HelloWorld.cpp)
target_link_libraries (HelloWorld Box2D)
