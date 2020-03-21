cmake_minimum_required(VERSION 3.1)
project(CppRobotics)
enable_testing()
set(3RDPARTY_DIR "${PROJECT_SOURCE_DIR}/3rdparty")

message(STATUS "Configuring modules")
add_subdirectory(modules)