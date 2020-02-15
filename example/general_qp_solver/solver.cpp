#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/osqp.hpp>
#include <qp_wrappers/cgal.hpp>
#include <qp_wrappers/qpoases.hpp>
#include <iostream>


#include <chrono>
#include <ctime>

int main() {


    qp_wrappers::qp<double> problem;
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

    auto qpoases_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector qpoases_soln;
    qp_wrappers::qpoases::solver qpoases;
    std::cout << "qpoases: " << qpoases.solve(problem, qpoases_soln) << std::endl << qpoases_soln << std::endl;
    auto qpoases_end = std::chrono::system_clock::now();


    auto cgal_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector cgal_soln;
    qp_wrappers::cgal::solver cgal;
    std::cout << "cgal: " << cgal.solve(problem, cgal_soln) << std::endl << cgal_soln << std::endl;
    auto cgal_end = std::chrono::system_clock::now();

    std::cout << "cgal: " <<  std::chrono::duration<double>(cgal_end - cgal_start).count() << std::endl << 
                 "osqp: " <<  std::chrono::duration<double>(osqp_end - osqp_start).count() << std::endl << 
                 "qpoases: " <<  std::chrono::duration<double>(qpoases_end - qpoases_start).count() << std::endl;

    return 0;
}