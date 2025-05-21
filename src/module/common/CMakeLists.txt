cmake_minimum_required(VERSION 3.10)
project(common)

# Header-only INTERFACE library
add_library(common INTERFACE)
add_library(common::common ALIAS common)

# Provide include path to consumers
target_include_directories(common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Optional install step (only if you install the project)
# install(DIRECTORY include/ DESTINATION include)
