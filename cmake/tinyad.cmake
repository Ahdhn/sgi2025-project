cmake_minimum_required(VERSION 3.19)

if(TARGET TinyAD)
    return()
endif()

set(TINYAD_EXAMPLES_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/output)
option(TINYAD_UNIT_TESTS "" OFF)
option(TINYAD_GEOMETRYCENTRAL "Build Geometry Central examples" OFF)

include(FetchContent)
FetchContent_Declare(
  tinyad
  GIT_REPOSITORY https://github.com/patr-schm/tinyad.git
  GIT_TAG 75093e14ef0d7bb39657c5f3b2aba1251afaa38c
)
FetchContent_GetProperties(tinyad)
if(NOT tinyad_POPULATED)  
  FetchContent_Populate(tinyad)
  message(STATUS "tinyad_SOURCE_DIR: ${tinyad_SOURCE_DIR}")
  message(STATUS "tinyad_BINARY_DIR: ${tinyad_BINARY_DIR}")
  add_subdirectory(${tinyad_SOURCE_DIR} ${tinyad_BINARY_DIR})
endif()