cmake_minimum_required(VERSION 3.19)

project(LocRemesh)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(libigl)
include(tinyad)
include(polyscope)
include(stb)

igl_include(core)

if (NOT TARGET OpenMP::OpenMP_CXX)
    find_package(OpenMP)
endif()


file(GLOB SRC_FILES src/*.* remesh/src/*.*)

add_executable(${PROJECT_NAME} ${SRC_FILES})



target_link_libraries(${PROJECT_NAME} PUBLIC 
  igl::core
  polyscope
  TinyAD
  stb_image           
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)