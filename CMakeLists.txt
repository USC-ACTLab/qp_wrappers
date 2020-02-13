cmake_minimum_required(VERSION 3.10)

project(qp_wrappers)
set (CMAKE_CXX_STANDARD 14)

add_subdirectory(third_party/eigen)
add_subdirectory(third_party/qpoases-mirror)

SET(DLONG OFF CACHE BOOL "osqp index type (ON for long long int, OFF for int)")
add_subdirectory(third_party/osqp)

find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Core PATHS third_party/cgal)

add_library(qp_wrappers INTERFACE)
target_include_directories(
    qp_wrappers INTERFACE 
    include 
    third_party/qpoases-mirror/include
    third_party/osqp/include
    ${CGAL_INCLUDE_DIRECTORIES})
target_link_libraries(qp_wrappers INTERFACE eigen qpOASES osqp CGAL::CGAL)

add_executable(linear_regression example/linear_regression/linear_regression.cpp)
target_link_libraries(linear_regression qp_wrappers)

add_executable(general_solver example/general_qp_solver/solver.cpp)
target_link_libraries(general_solver qp_wrappers)