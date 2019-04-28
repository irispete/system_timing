cmake_minimum_required(VERSION 2.8.3)
project(system_timing)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 17)
set(VERSION_PATCH 0)
set(BUILD_NUMBER 1)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        std_msgs
        iris_common
        )

set(cpprestsdk_DIR /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/)
find_package(cpprestsdk REQUIRED NAMES
        cpprestsdk
        cpprest
        )

find_program(CMAKE_CXX_CPPCHECK NAMES cppcheck)
list(
        APPEND CMAKE_CXX_CPPCHECK
        --xml
        --enable=warning,performance,information,style
        --std=c++11
        --verbose
        --quiet
        --suppress=noExplicitConstructor
        --suppress=*:/usr/*
        --suppress=*:/opt/ros/*
        --suppress=missingInclude
        --suppress=missingIncludeSystem
        --suppress=unmatchedSuppression
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
        ${iris_common_INCLUDE_DIRS}
        include/
)

# get git commit hash
find_package(Git)
execute_process(
        COMMAND git log -1 --format=%h
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/utm_link
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions(-DGIT_COMMIT_HASH="${GIT_COMMIT_HASH}")
add_definitions(-DPROJECT_NAME="${PROJECT_NAME}")
add_definitions(-DVERSION_MAJOR="${VERSION_MAJOR}")
add_definitions(-DVERSION_MINOR="${VERSION_MINOR}")
add_definitions(-DVERSION_PATCH="${VERSION_PATCH}")
add_definitions(-DBUILD_NUMBER="${BUILD_NUMBER}")


catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)

# define project dir
add_definitions(-DPROJECT_SRC_DIR=\"${PROJECT_SOURCE_DIR}\")

find_package(cmake_modules)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror ${CMAKE_CXX_FLAGS}")


if (COVERAGE)
    set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -march=native -Wall -fprofile-arcs -ftest-coverage")
    add_definitions(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} " --coverage")
    set(gcov_LIBS "gcov")
else ()
    set(gcov_LIBS "")
endif ()


#	${YAMLCPP_LIBRARY}

# compile utm
add_library(system_timing
        src/ros_node.cpp
        src/system_timing_ros_node.cpp
        src/event_time_manager.cpp
        src/event_time_publisher.cpp
        )
add_dependencies(system_timing ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_timing PRIVATE cpprestsdk::cpprest
        ${gcov_LIBS}
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        )

# ros node
add_executable(system_timing_node
        src/system_timing_node.cpp
        )
add_dependencies(system_timing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(system_timing_node
        system_timing
        iris_common
        ${gcov_LIBS}
        ${catkin_LIBRARIES}
        )

if (EXISTS ${PROJECT_SOURCE_DIR}/test/iris_unit_test_framework/CMakeLists.txt)
    add_subdirectory(${PROJECT_SOURCE_DIR}/test/iris_unit_test_framework)
    iris_add_all_tests(LIBRARIES system_timing iris_common)
endif ()
