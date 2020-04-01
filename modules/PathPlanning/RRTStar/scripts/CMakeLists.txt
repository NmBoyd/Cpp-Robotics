cmake_minimum_required(VERSION 3.1)
project(RRTStarPlannerScripts)
message(STATUS "COMPILING RRTPlannerScripts")

add_executable(RunRRTStarPlanner RunRRTStarPlanner.cpp)
target_link_libraries(RunRRTStarPlanner LibRRTStarPlanner)
