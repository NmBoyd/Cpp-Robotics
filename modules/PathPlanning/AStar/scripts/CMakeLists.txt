cmake_minimum_required(VERSION 3.1)
project(AStarPlannerScripts)
message(STATUS "COMPILING AStarPlannerScripts")

add_executable(RunAStarPlanner RunAStarPlanner.cpp)
target_link_libraries(RunAStarPlanner LibAStarPlanner)
