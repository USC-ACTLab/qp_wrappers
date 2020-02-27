#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/osqp.hpp>
#include <qp_wrappers/cgal.hpp>
#include <qp_wrappers/qpoases.hpp>
#include <qp_wrappers/gurobi.hpp>
#include <qp_wrappers/cplex.hpp>
#include <qp_wrappers/alglib.hpp>

#include <iostream>
#include <chrono>
#include <ctime>

int main() {


    qp_wrappers::qp<double> problem(0);
    std::cin >> problem;

    bool is_consistent = problem.is_consistent();
    std::cout << "is consistent? " << is_consistent << std::endl;

// #ifdef CGAL_QP_NO_ASSERTIONS
//     std::cout << " CGAL_QP_NO_ASSERTIONS" << std::endl;
// #else
//     std::cout << "no CGAL_QP_NO_ASSERTIONS" << std::endl;
// #endif

    auto alglib_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector alglib_soln;
    qp_wrappers::alglib::solver alglib;
    qp_wrappers::return_type alglib_ret = alglib.solve(problem, alglib_soln);
    std::cout << "alglib: " << alglib_ret << std::endl;
    if(alglib_ret == qp_wrappers::success) {
        std::cout << alglib_soln << std::endl;
    }
    auto alglib_end = std::chrono::system_clock::now();
    bool alglib_verif = problem.verify(alglib_soln);
    std::cout << "alglib verification: " << alglib_verif << std::endl;



    auto osqp_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector osqp_soln;
    qp_wrappers::osqp::solver osqp;
    qp_wrappers::return_type osqp_ret = osqp.solve(problem, osqp_soln);
    std::cout << "osqp: " << osqp_ret << std::endl;
    if(osqp_ret == qp_wrappers::success) {
        std::cout << osqp_soln << std::endl;
    }
    auto osqp_end = std::chrono::system_clock::now();
    bool osqp_verif = problem.verify(osqp_soln);
    std::cout << "osqp verification: " << osqp_verif << std::endl;



    auto qpoases_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector qpoases_soln;
    qp_wrappers::qpoases::solver qpoases;
    qp_wrappers::return_type qpoases_ret = qpoases.solve(problem, qpoases_soln);
    std::cout << "qpoases: " << qpoases_ret << std::endl;
    if(qpoases_ret == qp_wrappers::success) {
        std::cout << qpoases_soln << std::endl;
    }
    auto qpoases_end = std::chrono::system_clock::now();
    bool qpoases_verif = problem.verify(qpoases_soln);
    std::cout << "qpoases verification: " << qpoases_verif << std::endl;



    auto cgal_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector cgal_soln;
    qp_wrappers::cgal::solver cgal;
    qp_wrappers::return_type cgal_ret = cgal.solve(problem, cgal_soln);
    std::cout << "cgal: " << cgal_ret << std::endl;
    if(cgal_ret == qp_wrappers::success) {
        std::cout << cgal_soln << std::endl;
    }
    auto cgal_end = std::chrono::system_clock::now();
    bool cgal_verif = problem.verify(cgal_soln);
    std::cout << "cgal verification: " << cgal_verif << std::endl;




    auto gurobi_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector gurobi_soln;
    qp_wrappers::gurobi::solver gurobi;
    qp_wrappers::return_type gurobi_ret = gurobi.solve(problem, gurobi_soln);
    std::cout << "gurobi: " << gurobi_ret << std::endl;
    if(gurobi_ret == qp_wrappers::success) {
        std::cout << gurobi_soln << std::endl;
    }
    auto gurobi_end = std::chrono::system_clock::now();
    bool gurobi_verif = problem.verify(gurobi_soln);
    std::cout << "gurobi verification: " << gurobi_verif << std::endl;



    auto cplex_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector cplex_soln;
    qp_wrappers::cplex::solver cplex;
    qp_wrappers::return_type cplex_ret = cplex.solve(problem, cplex_soln);
    std::cout << "cplex: " << cplex_ret << std::endl;
    if(cplex_ret == qp_wrappers::success) {
        std::cout << cplex_soln << std::endl;
    }
    auto cplex_end = std::chrono::system_clock::now();
    bool cplex_verif = problem.verify(cplex_soln);
    std::cout << "cplex verification: " << cplex_verif << std::endl;




    std::cout << std::endl << "Run durations:" << std::endl <<
                 "\tosqp: " <<  std::chrono::duration<double>(osqp_end - osqp_start).count() << std::endl << 
                 "\tqpoases: " <<  std::chrono::duration<double>(qpoases_end - qpoases_start).count() << std::endl <<
                 "\tcgal: " <<  std::chrono::duration<double>(cgal_end - cgal_start).count() << std::endl << 
                 "\tgurobi: "<< std::chrono::duration<double>(gurobi_end - gurobi_start).count() << std::endl <<
                 "\tcplex: " <<  std::chrono::duration<double>(cplex_end - cplex_start).count() << std::endl <<
                 "\talglib: " <<  std::chrono::duration<double>(alglib_end - alglib_start).count() << std::endl;






    return 0;
}
