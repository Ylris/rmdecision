cmake_minimum_required(VERSION 3.8)
project(information_widgets)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(SRC_LIST
  src/chessboard_def.cpp
  src/prism_def.cpp
)

set(HEAD_LIST
  include/information_widgets/chessboard_def.hpp
  include/information_widgets/prism_def.hpp
  include/information_widgets/rm_decision_defs.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SRC_LIST}
  ${HEAD_LIST}
)

ament_auto_package()
