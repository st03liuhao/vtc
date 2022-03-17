project(virtual_cabinet)
add_executable(virtual_cabinet_test virtual_cabinet_test.cpp)

set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include" "${CMAKE_SOURCE_DIR}/doctest/doctest")
target_include_directories(virtual_cabinet_test PRIVATE ${INCLUDE_DIR})