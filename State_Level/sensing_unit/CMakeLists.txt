cmake_minimum_required(VERSION 3.10)
project(sensing_unit)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
include_directories(include)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/sensing_unit.cpp
        src/node_exec.cpp
        include/sensing_unit/sensing_unit.hpp
        include/sensing_unit/rm_msg_includes.hpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
        src/node_exec.cpp
)

#############
## Testing ##
#############

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
            ament_cmake_copyright
            ament_cmake_uncrustify
    )
    ament_lint_auto_find_test_dependencies()
endif ()

#############
## Install ##
#############

ament_auto_package(
        INSTALL_TO_SHARE
        launch
)
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}
)

