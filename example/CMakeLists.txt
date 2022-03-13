project(virtual_cabinet)
add_executable(example example.cpp)

set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
message(STATUS "INCLUDE_DIR" ${INCLUDE_DIR})
target_include_directories(example PRIVATE ${INCLUDE_DIR})