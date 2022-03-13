cmake_minimum_required(VERSION 3.21)
set(CMAKE_CXX_STANDARD 23)
project(virtual_cabinet)

set(LIB_NAME ${PROJECT_NAME})
add_library(${LIB_NAME} INTERFACE)
add_library(${LIB_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(
    ${PROJECT_NAME}
    INTERFACE
    $<BUILD_INTERFACE:${INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

add_subdirectory(example)
