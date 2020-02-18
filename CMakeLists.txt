cmake_minimum_required(VERSION 3.10)

project(qp_wrappers)
set (CMAKE_CXX_STANDARD 14)

add_subdirectory(third_party/eigen)
add_subdirectory(third_party/qpoases-mirror)

SET(DFLOAT OFF CACHE BOOL "osqp data type(OFF for double, ON for float)")
SET(DLONG OFF CACHE BOOL "osqp index type (ON for long long int, OFF for int)")
add_subdirectory(third_party/osqp)

# SET(NDEBUG ON CACHE BOOL "cgal qp solver perf on")
# SET(CGAL_QP_NO_ASSERTIONS ON CACHE BOOL "cgal qp solver perf on")
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Core PATHS third_party/cgal)

add_library(qp_wrappers INTERFACE)
target_include_directories(
  qp_wrappers
  INTERFACE 
  include 
  third_party/qpoases-mirror/include
  third_party/osqp/include
  ${CGAL_INCLUDE_DIRECTORIES}
)
target_link_libraries(
  qp_wrappers 
  INTERFACE 
  eigen 
  qpOASES 
  osqp 
  CGAL::CGAL
  ${CMAKE_DL_LIBS}
)
target_compile_definitions(
  qp_wrappers 
  INTERFACE 
  CGAL_QP_NO_ASSERTIONS
)

include_directories( ${GUROBI_INCLUDE_DIRS} )
target_link_libraries( qp_wrappers ${GUROBI_LIBRARIES} )

set (CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -std=c++11")
target_link_libraries(qp_wrappers INTERFACE "-I /Library/gurobi901/mac64/include/ -L /Library/gurobi901/mac64/lib -lgurobi_c++ -lgurobi90 -lm")

add_executable(
  linear_regression 
  example/linear_regression/linear_regression.cpp
)
target_link_libraries(
  linear_regression 
  qp_wrappers
)

add_executable(
  general_solver 
  example/general_qp_solver/solver.cpp
)
target_link_libraries(
  general_solver
  qp_wrappers
)