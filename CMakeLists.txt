cmake_minimum_required(VERSION 3.2)
project(hmc6343)

# Set C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Build
include_directories(include utilities)

add_executable(${PROJECT_NAME}
  src/hmc6343.cpp
  src/i2c.cpp
  example/hmc6343_example.cpp
)

target_link_libraries(${PROJECT_NAME})
