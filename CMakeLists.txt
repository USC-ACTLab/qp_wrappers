cmake_minimum_required(VERSION 3.9)

project(qp_wrappers)
set (CMAKE_CXX_STANDARD 17)

option(QPWRAPPERS_WITH_CPLEX "with cplex" ON)
option(QPWRAPPERS_WITH_GUROBI "with cplex" ON)
option(QPWRAPPERS_WITH_QPOASES "with cplex" ON)
option(QPWRAPPERS_WITH_OSQP "with cplex" ON)

option(QPWRAPPERS_BUILD_EXAMPLES "build examples" OFF)

if(QPWRAPPERS_WITH_QPOASES)
    SET(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "qpoases examples")
    add_subdirectory(third_party/qpoases-mirror)
    find_package(LAPACK REQUIRED)
    find_package(BLAS REQUIRED)

    SET(QPWRAPPERS_QPOASES_LIBRARIES
            qpOASES
            ${LAPACK_LIBRARIES}
            ${BLAS_LIBRARIES})

    SET(QPWRAPPERS_QPOASES_INCLUDE_DIRS
            third_party/qpoases-mirror/include)
endif()

if(QPWRAPPERS_WITH_OSQP)
    SET(DFLOAT OFF CACHE BOOL "osqp data type(OFF for double, ON for float)")
    SET(DLONG OFF CACHE BOOL "osqp index type (ON for long long int, OFF for int)")
    add_subdirectory(third_party/osqp)

    SET(QPWRAPPERS_OSQP_LIBRARIES
            osqp)

    SET(QPWRAPPERS_OSQP_INCLUDE_DIRS
            third_party/osqp/include)
endif()

if(QPWRAPPERS_WITH_CPLEX)
    include(cmake/FindCPLEX.cmake)
    SET(QPWRAPPERS_CPLEX_LIBRARIES ilocplex)
endif()

if(QPWRAPPERS_WITH_GUROBI)
    include(cmake/FindGUROBI.cmake)
endif()

find_package(Eigen3 REQUIRED)

add_library(qp_wrappers INTERFACE)
target_include_directories(
    qp_wrappers
    INTERFACE
    include
    ${QPWRAPPERS_QPOASES_INCLUDE_DIRS}
    ${QPWRAPPERS_OSQP_INCLUDE_DIRS}
    ${CPLEX_ILOCPLEX_INCLUDE_DIR}
    ${GUROBI_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(
    qp_wrappers
    INTERFACE
    ${QPWRAPPERS_QPOASES_LIBRARIES}
    ${QPWRAPPERS_OSQP_LIBRARIES}
    ${QPWRAPPERS_CPLEX_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${GUROBI_CXX_LIBRARY}
    ${GUROBI_LIBRARY}
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

if(QPWRAPPERS_WITH_CPLEX AND QPWRAPPERS_BUILD_EXAMPLES)
    add_executable(
            cplex_multiple_times
            example/cplex/cplex_multiple_times.cpp
    )
    target_link_libraries (
            cplex_multiple_times
            qp_wrappers
    )
endif()


if(QPWRAPPERS_WITH_GUROBI AND QPWRAPPERS_BUILD_EXAMPLES)
add_executable(
        gurobi_multiple_times
        example/gurobi/gurobi_multiple_times.cpp
)
target_link_libraries (
        gurobi_multiple_times
        qp_wrappers
)
endif()

if(QPWRAPPERS_WITH_QPOASES AND QPWRAPPERS_BUILD_EXAMPLES)
    add_executable(
            qpoases_multiple_times
            example/qpoases/qpoases_multiple_times.cpp
    )
    target_link_libraries (
            qpoases_multiple_times
            qp_wrappers
    )
endif()

if(QPWRAPPERS_WITH_OSQP AND QPWRAPPERS_BUILD_EXAMPLES)
    add_executable(
            osqp_multiple_times
            example/osqp/osqp_multiple_times.cpp
    )
    target_link_libraries (
            osqp_multiple_times
            qp_wrappers
    )
endif()
