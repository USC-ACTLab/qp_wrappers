cmake_minimum_required(VERSION 3.10)

project(qp_wrappers)
set (CMAKE_CXX_STANDARD 14)

if (GUROBI_INCLUDE_DIR)
  # in cache already
  set(GUROBI_FOUND TRUE)
  set(GUROBI_INCLUDE_DIRS "${GUROBI_INCLUDE_DIR}" )
  set(GUROBI_LIBRARIES "${GUROBI_LIBRARY};${GUROBI_CXX_LIBRARY}" )
else (GUROBI_INCLUDE_DIR)

find_path(GUROBI_INCLUDE_DIR 
          NAMES gurobi_c++.h
          PATHS "$ENV{GUROBI_HOME}/include"
                  "/Library/gurobi901/mac64/include"
                 "C:\\libs\\gurobi502\\include"
          )

find_library( GUROBI_LIBRARY 
              NAMES gurobi
        gurobi45
        gurobi46
        gurobi50 
        gurobi51
        gurobi52
        gurobi55
        gurobi56
        gurobi60
        gurobi65
        gurobi90
              PATHS "$ENV{GUROBI_HOME}/lib" 
                    "/Library/gurobi901/mac64/lib"
                    "C:\\libs\\gurobi502\\lib"
              )

find_library( GUROBI_CXX_LIBRARY 
              NAMES gurobi_c++
              PATHS "$ENV{GUROBI_HOME}/lib" 
                    "/Library/gurobi901/mac64/lib"
                    "C:\\libs\\gurobi502\\lib"
              )

set(GUROBI_INCLUDE_DIRS "${GUROBI_INCLUDE_DIR}" )
set(GUROBI_LIBRARIES "${GUROBI_LIBRARY};${GUROBI_CXX_LIBRARY}" )

# use c++ headers as default
# set(GUROBI_COMPILER_FLAGS "-DIL_STD" CACHE STRING "Gurobi Compiler Flags")

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCPLEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(GUROBI  DEFAULT_MSG
                                  GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_INCLUDE_DIR)

mark_as_advanced(GUROBI_INCLUDE_DIR GUROBI_LIBRARY GUROBI_CXX_LIBRARY)

endif(GUROBI_INCLUDE_DIR)

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
  ${GUROBI_INCLUDE_DIRS}
)
target_link_libraries(
  qp_wrappers 
  INTERFACE 
  eigen 
  qpOASES 
  osqp 
  CGAL::CGAL
  ${CMAKE_DL_LIBS}
  "-I /Library/gurobi901/mac64/include -L /Library/gurobi901/mac64/lib -lgurobi_c++ -lgurobi90 -lm"
)
target_compile_definitions(
  qp_wrappers 
  INTERFACE 
  CGAL_QP_NO_ASSERTIONS
)

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
