cmake_minimum_required(VERSION 3.15)
project(Blackhole_6)

set(CMAKE_CXX_STANDARD 14)

if (NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(OpenCV REQUIRED)

add_executable(Blackhole_6
        main.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(Blackhole_6 PUBLIC
        ${OpenCV_LIBS})

target_compile_definitions(Blackhole_6 PRIVATE PWD=${CMAKE_CURRENT_LIST_DIR})
