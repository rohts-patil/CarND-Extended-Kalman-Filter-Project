project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

file(GLOB ekf_sources src/*.cpp)
file(GLOB to_remove src/rmse.cpp)
list(REMOVE_ITEM ekf_sources ${to_remove})

file(GLOB rmse_sources src/*.cpp)
file(GLOB to_remove src/main.cpp)
list(REMOVE_ITEM rmse_sources ${to_remove})

add_executable(ExtendedKF ${ekf_sources})
add_executable(CalcRMSE ${rmse_sources})