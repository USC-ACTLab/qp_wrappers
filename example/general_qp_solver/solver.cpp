#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/osqp.hpp>
#include <qp_wrappers/cgal.hpp>
#include <qp_wrappers/qpoases.hpp>
#include <qp_wrappers/gurobi.hpp>
#include <qp_wrappers/cplex.hpp>

#include <iostream>
#include <chrono>
#include <ctime>

int main() {


    qp_wrappers::qp<double> problem(0);
    std::cin >> problem;


// #ifdef CGAL_QP_NO_ASSERTIONS
//     std::cout << " CGAL_QP_NO_ASSERTIONS" << std::endl;
// #else
//     std::cout << "no CGAL_QP_NO_ASSERTIONS" << std::endl;
// #endif



    auto osqp_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector osqp_soln;
    qp_wrappers::osqp::solver osqp;
    std::cout << "osqp: " << osqp.solve(problem, osqp_soln) << std::endl << osqp_soln << std::endl;
    auto osqp_end = std::chrono::system_clock::now();
    bool osqp_verif = problem.verify(osqp_soln);
    std::cout << "osqp verification: " << osqp_verif << std::endl;

    auto qpoases_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector qpoases_soln;
    qp_wrappers::qpoases::solver qpoases;
    std::cout << "qpoases: " << qpoases.solve(problem, qpoases_soln) << std::endl << qpoases_soln << std::endl;
    auto qpoases_end = std::chrono::system_clock::now();
    bool qpoases_verif = problem.verify(qpoases_soln);
    std::cout << "qpoases verification: " << qpoases_verif << std::endl;

    auto cgal_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector cgal_soln;
    qp_wrappers::cgal::solver cgal;
    std::cout << "cgal: " << cgal.solve(problem, cgal_soln) << std::endl << cgal_soln << std::endl;
    auto cgal_end = std::chrono::system_clock::now();
    bool cgal_verif = problem.verify(cgal_soln);
    std::cout << "cgal verification: " << cgal_verif << std::endl;


    auto gurobi_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector gurobi_soln;
    qp_wrappers::gurobi::solver gurobi;
    std::cout << "gurobi: " << gurobi.solve(problem, gurobi_soln) << std::endl << gurobi_soln << std::endl;
    auto gurobi_end = std::chrono::system_clock::now();
    bool gurobi_verif = problem.verify(gurobi_soln);
    std::cout << "gurobi verification: " << gurobi_verif << std::endl;


    auto cplex_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector cplex_soln;
    qp_wrappers::cplex::solver cplex;
    std::cout << "cplex: " << cplex.solve(problem, cplex_soln) << std::endl << cplex_soln << std::endl;
    auto cplex_end = std::chrono::system_clock::now();
    bool cplex_verif = problem.verify(cplex_soln);
    std::cout << "cplex verification: " << cplex_verif << std::endl;




    std::cout << std::endl << "Run durations:" << std::endl <<
                 "\tosqp: " <<  std::chrono::duration<double>(osqp_end - osqp_start).count() << std::endl << 
                 "\tqpoases: " <<  std::chrono::duration<double>(qpoases_end - qpoases_start).count() << std::endl <<
                 "\tcgal: " <<  std::chrono::duration<double>(cgal_end - cgal_start).count() << std::endl << 
                 "\tgurobi: "<< std::chrono::duration<double>(gurobi_end - gurobi_start).count() << std::endl <<
                 "\tcplex: " <<  std::chrono::duration<double>(cplex_end - cplex_start).count() << std::endl;






    return 0;
}
