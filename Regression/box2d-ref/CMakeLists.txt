cmake_minimum_required(VERSION 2.6)

project(Box2DRef)

option(BOX2D_BUILD_SHARED "Build Box2D shared libraries" OFF)
option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)

set(BOX2DREF_VERSION 2.3.2)
set(LIB_INSTALL_DIR lib${LIB_SUFFIX})

# The Box2D library.
add_subdirectory(Box2DRef)

