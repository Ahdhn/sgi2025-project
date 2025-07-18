cmake_minimum_required(VERSION 3.19)

include(FetchContent)

FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG        master
  SOURCE_SUBDIR  .
)
FetchContent_MakeAvailable(stb)

add_library(stb_image INTERFACE)

target_include_directories(stb_image INTERFACE ${stb_SOURCE_DIR})