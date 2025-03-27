cmake_minimum_required(VERSION 3.8)
project(decision_base)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(SRC_LIST
  src/decision_base.cpp
)

set(HEAD_LIST
  include/decision_base/decision_base.hpp
)

ament_auto_add_library(decision_base_lib SHARED
  ${SRC_LIST}
  ${HEAD_LIST}
)

ament_auto_package()