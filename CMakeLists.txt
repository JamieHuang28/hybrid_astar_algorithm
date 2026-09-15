cmake_minimum_required(VERSION 3.12)
project(hybrid_astar_algorithm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/apa_config.cpp
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

set(HYBRID_ASTAR_DEFAULT_APA_JSON
    "${CMAKE_CURRENT_SOURCE_DIR}/config/apa.json")
set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection_null.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geometry2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vector2d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize_ros.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/apa_config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann_json.h
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

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
target_compile_definitions(${PROJECT_NAME} PRIVATE
  HYBRID_ASTAR_DEFAULT_APA_JSON="${HYBRID_ASTAR_DEFAULT_APA_JSON}")
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(example src/example.cpp)
target_link_libraries(example ${PROJECT_NAME})

# -------------------- Python bindings (pybind11) --------------------
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
execute_process(
  COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
  OUTPUT_VARIABLE _PYBIND11_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _PYBIND11_DIR_OK
)
if(_PYBIND11_DIR_OK EQUAL 0)
  set(pybind11_DIR "${_PYBIND11_DIR}")
endif()

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(hybrid_astar python/hybrid_astar_pybind.cpp)
target_include_directories(hybrid_astar PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})
target_compile_definitions(hybrid_astar PRIVATE
  HYBRID_ASTAR_DEFAULT_APA_JSON="${HYBRID_ASTAR_DEFAULT_APA_JSON}")
target_link_libraries(hybrid_astar PRIVATE ${PROJECT_NAME} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(hybrid_astar PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python
  BUILD_RPATH "${CMAKE_BINARY_DIR}"
)
message(STATUS "Building Python module hybrid_astar (pybind11_DIR=${pybind11_DIR})")
message(STATUS "Default apa.json: ${HYBRID_ASTAR_DEFAULT_APA_JSON}")
