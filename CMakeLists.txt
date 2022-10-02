cmake_minimum_required(VERSION 3.15)
project(Blackhole_6)

set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(OpenCV REQUIRED)

set(EXTRA_FLAGS "")
macro(ADD_FLAGS)
    string(REPLACE ";" " " output "${ARGV}")
    set(EXTRA_FLAGS "${EXTRA_FLAGS} ${output}")
endmacro()


if (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    ADD_FLAGS(/O2)
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    ADD_FLAGS(-O3 -Wno-deprecated-register)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    ADD_FLAGS(-O3 -Wno-deprecated-register)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    if (WIN32)
        ADD_FLAGS(/O2)
    else()
        ADD_FLAGS(-O3)
    endif()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_FLAGS}")

add_executable(Blackhole_6
        main.cpp)

include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(Blackhole_6 PUBLIC
        ${OpenCV_LIBS})

target_compile_definitions(Blackhole_6 PRIVATE PWD=${CMAKE_CURRENT_LIST_DIR})
