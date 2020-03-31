cmake_minimum_required(VERSION 3.1)
project(RRTPlannerScripts)
message(STATUS "COMPILING RRTPlannerScripts")

add_executable(RunRRTPlanner RunRRTPlanner.cpp)
target_link_libraries(RunRRTPlanner LibRRTPlanner)
