cmake_minimum_required(VERSION 3.1)
project(DijkstraPlannerScripts)
message(STATUS "COMPILING AStarPlannerScripts")

add_executable(RunDijkstraPlanner RunDijkstraPlanner.cpp)
target_link_libraries(RunDijkstraPlanner LibDijkstraPlanner)
