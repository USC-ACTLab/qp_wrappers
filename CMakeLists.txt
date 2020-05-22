cmake_minimum_required(VERSION 3.9)

project(qp_wrappers)
set (CMAKE_CXX_STANDARD 17)

add_subdirectory(third_party/qpoases-mirror)

SET(DFLOAT OFF CACHE BOOL "osqp data type(OFF for double, ON for float)")
SET(DLONG OFF CACHE BOOL "osqp index type (ON for long long int, OFF for int)")
add_subdirectory(third_party/osqp)
include(cmake/FindCPLEX.cmake)
include(cmake/FindGUROBI.cmake)
find_package(Eigen3 REQUIRED)

add_library(qp_wrappers INTERFACE)
target_include_directories(
  qp_wrappers
  INTERFACE 
  include 
  third_party/qpoases-mirror/include
  third_party/osqp/include
  ${GUROBI_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(
  qp_wrappers 
  INTERFACE
  qpOASES 
  osqp
  ilocplex
  ${CMAKE_DL_LIBS}
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
)

add_executable(
  cplex_multiple_times
  example/cplex/cplex_multiple_times.cpp
)
target_link_libraries (
  cplex_multiple_times
  qp_wrappers
)

add_library(
  qp_wrappers_problem
  INTERFACE
)
target_include_directories(
  qp_wrappers_problem
  INTERFACE
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(
  gurobi_multiple_times
  example/gurobi/gurobi_multiple_times.cpp
)
target_link_libraries (
  gurobi_multiple_times
  qp_wrappers
)

add_executable(
  qpoases_multiple_times
  example/qpoases/qpoases_multiple_times.cpp
)
target_link_libraries (
  qpoases_multiple_times
  qp_wrappers
)

add_executable(
  osqp_multiple_times
  example/osqp/osqp_multiple_times.cpp
)
target_link_libraries (
  osqp_multiple_times
  qp_wrappers
)

add_executable(
  mpc_solver
  example/mpc/mpc_solver.cpp
)
target_link_libraries (
  mpc_solver
  qp_wrappers
  stdc++fs
)
