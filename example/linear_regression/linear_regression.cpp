#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/qpoases.hpp>
#include <qp_wrappers/osqp.hpp>
#include <qp_wrappers/cgal.hpp>
#include <qp_wrappers/gurobi.hpp>
#include <qp_wrappers/cplex.hpp>
#include <qp_wrappers/alglib.hpp>
#include <iostream>
#include <chrono>

int main() {
    unsigned int var_count;
    std::cout << "var count (dimensions of samples + 1): ";
    std::cin >> var_count;

    qp_wrappers::qp<double> problem(var_count);

    while(true) {
        char c;
        std::cin >> c;
        if(c == 'p') {
            qp_wrappers::qp<double>::Vector x(var_count);
            double y;

            for(int i = 0; i < var_count - 1; i++) {
                std::cin >> x(i);
            }
            x(var_count - 1) = 1;

            std::cin >> y;

            problem.add_Q(2 * x * x.transpose());
            problem.add_c(-2 * x * y);
        } else if (c == 's') {
            break;
        }
    }

    // std::cout << std::endl <<  problem.Q << std::endl << std::endl << problem.A << std::endl << std::endl << problem.c << std::endl;

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


    auto alglib_start = std::chrono::system_clock::now();
    qp_wrappers::qp<double>::Vector alglib_soln;
    qp_wrappers::alglib::solver alglib;
    std::cout << "alglib: " << alglib.solve(problem, alglib_soln) << std::endl << alglib_soln << std::endl;
    auto alglib_end = std::chrono::system_clock::now();
    bool alglib_verif = problem.verify(alglib_soln);
    std::cout << "alglib verification: " << alglib_verif << std::endl;




    std::cout << std::endl << "Run durations:" << std::endl <<
                 "\tosqp: " <<  std::chrono::duration<double>(osqp_end - osqp_start).count() << std::endl << 
                 "\tqpoases: " <<  std::chrono::duration<double>(qpoases_end - qpoases_start).count() << std::endl <<
                 "\tcgal: " <<  std::chrono::duration<double>(cgal_end - cgal_start).count() << std::endl << 
                 "\tgurobi: "<< std::chrono::duration<double>(gurobi_end - gurobi_start).count() << std::endl <<
                 "\tcplex: " <<  std::chrono::duration<double>(cplex_end - cplex_start).count() << std::endl <<
                 "\talglib: " <<  std::chrono::duration<double>(alglib_end - alglib_start).count() << std::endl;




    return 0;
}
