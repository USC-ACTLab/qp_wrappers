cmake_minimum_required(VERSION 3.10)

project(qp_wrappers)
set (CMAKE_CXX_STANDARD 14)


add_subdirectory(third_party/eigen)
add_subdirectory(third_party/qpoases-mirror)
add_subdirectory(third_party/osqp)


add_library(qp_wrappers INTERFACE)
target_include_directories(
    qp_wrappers INTERFACE 
    include 
    third_party/qpoases-mirror/include
    third_party/osqp/include)
target_link_libraries(qp_wrappers INTERFACE eigen qpOASES osqp)

add_executable(linear_regression example/linear_regression.cpp)
target_link_libraries(linear_regression qp_wrappers)