cmake_minimum_required(VERSION 2.6)
project(landmarks)

find_package(Lua51 REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)

include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIRS})
add_library(landmarks SHARED landmarks.h landmarks.cpp)
target_link_libraries(landmarks ${ARGOS_LIBRARIES} ${LUA_LIBRARIES} argos3plugin_simulator_footbot)
