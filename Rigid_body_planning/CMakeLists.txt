cmake_minimum_required(VERSION 2.8)

ADD_DEFINITIONS(
  -Wall
  -g
  -O2
  -std=c++11
)

project(RigidBodyPlanning)

find_package(OMPL REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})
add_definitions(${OMPL_DEFINITIONS})

FIND_PACKAGE(Boost COMPONENTS system REQUIRED)

add_executable (RigidBodyPlanning RigidBodyPlanning.cpp)
target_link_libraries (RigidBodyPlanning ${Boost_LIBRARIES} ${OMPL_LIBRARIES})
