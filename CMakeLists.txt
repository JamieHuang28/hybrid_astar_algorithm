cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar_algorithm)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection_null.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize_ros.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
    )
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection_null.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vector2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize_ros.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lookup.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/gradient.h #Andrew Noske
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamicvoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
    ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard

    )

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

# Boost and its components
find_package( Boost REQUIRED COMPONENTS serialization)

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()  
endif()

include_directories(include ${OMPL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(include include)

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(example_${PROJECT_NAME} src/example.cpp)
target_link_libraries(example_${PROJECT_NAME} ${PROJECT_NAME})