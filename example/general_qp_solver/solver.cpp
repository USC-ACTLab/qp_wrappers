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


#include <mutex>
#include <optional>
#include <future>
#include <functional>
#include <thread>

std::mutex print_mutex;

template<typename Solver, typename Problem>
double run(const Solver& solver, const Problem& problem, std::string name) {
    typename Problem::Vector soln;
    auto start = std::chrono::system_clock::now();
    QPWrappers::ReturnType return_value;
    try {
        return_value = solver.solve(problem, soln);
    } catch(...) {
        return_value = QPWrappers::ReturnType::failure;
    }
    auto end = std::chrono::system_clock::now();

    std::unique_lock<std::mutex> lck(print_mutex);

    if(return_value == QPWrappers::ReturnType::success) {
        bool verif = problem.verify(soln, 1e-5);
        std::cout << name << " success" << std::endl;
        // std::cout << name << " ### solution ###" << std::endl;
        // std::cout << soln << std::endl;
        std::cout << name << " verification: " << verif << std::endl;
    } else {
        std::cout << name << " failed" << std::endl;
    }

    return std::chrono::duration<double>(end - start).count();
}

int main() {


    using ProblemType = QPWrappers::Problem<double>;
    ProblemType problem(0);
    std::cin >> problem;

    bool is_consistent = problem.is_consistent();
    std::cout << "is consistent? " << is_consistent << std::endl;

// #ifdef CGAL_QP_NO_ASSERTIONS
//     std::cout << " CGAL_QP_NO_ASSERTIONS" << std::endl;
// #else
//     std::cout << "no CGAL_QP_NO_ASSERTIONS" << std::endl;
// #endif


    // QPWrappers::OSQP::Solver osqp;
    QPWrappers::qpOASES::Solver qpoases;
    // QPWrappers::CGAL::Solver cgal;
    QPWrappers::GUROBI::Solver gurobi;
    QPWrappers::CPLEX::Solver cplex;
    // QPWrappers::ALGLIB::Solver alglib;

    
    // std::packaged_task<double(QPWrappers::OSQP::Solver, ProblemType, std::string)> osqp_task(run<QPWrappers::OSQP::Solver, ProblemType>);
    std::packaged_task<double(QPWrappers::qpOASES::Solver, ProblemType, std::string)> qpoases_task(run<QPWrappers::qpOASES::Solver, ProblemType>);
    // std::packaged_task<double(QPWrappers::CGAL::Solver, ProblemType, std::string)> cgal_task(run<QPWrappers::CGAL::Solver, ProblemType>);
    std::packaged_task<double(QPWrappers::GUROBI::Solver, ProblemType, std::string)> gurobi_task(run<QPWrappers::GUROBI::Solver, ProblemType>);
    std::packaged_task<double(QPWrappers::CPLEX::Solver, ProblemType, std::string)> cplex_task(run<QPWrappers::CPLEX::Solver, ProblemType>);
    // std::packaged_task<double(QPWrappers::ALGLIB::Solver, ProblemType)> alglib_task(run<QPWrappers::ALGLIB::Solver, ProblemType>);


    // std::future<double> osqp_time = osqp_task.get_future();
    std::future<double> qpoases_time = qpoases_task.get_future();
    // std::future<double> cgal_time = cgal_task.get_future();
    std::future<double> gurobi_time = gurobi_task.get_future();
    std::future<double> cplex_time = cplex_task.get_future();
    // std::future<double> alglib_time = alglib_task.get_future();

    // std::thread osqp_thread(std::move(osqp_task), osqp, problem, "OSQP");
    std::thread qpoases_thread(std::move(qpoases_task), qpoases, problem, "qpOASES");
    // std::thread cgal_thread(std::move(cgal_task), cgal, problem, "CGAL");
    std::thread gurobi_thread(std::move(gurobi_task), gurobi, problem, "GUROBI");
    std::thread cplex_thread(std::move(cplex_task), cplex, problem, "CPLEX");
    // std::thread alglib_thread(std::move(alglib_task), alglib, problem, "ALGLIB");

    // osqp_thread.join();
    qpoases_thread.join();
    // cgal_thread.join();
    gurobi_thread.join();
    cplex_thread.join();
    // alglib_thread.join();

    // double osqp_real_time = osqp_time.get();
    double qpoases_real_time = qpoases_time.get();
    // double cgal_real_time = cgal_time.get();
    double gurobi_real_time = gurobi_time.get();
    double cplex_real_time = cplex_time.get();
    // double alglib_real_time = alglib_time.get();



    std::cout << std::endl << "Run durations:" << std::endl <<
                 /*"\tosqp: " <<  osqp_real_time << std::endl << */
                 "\tqpoases: " << qpoases_real_time << std::endl <<
                 /*"\tcgal: " << cgal_real_time << std::endl << */
                 "\tgurobi: "<< gurobi_real_time << std::endl <<
                 "\tcplex: " << cplex_real_time << std::endl <<
                 /*"\talglib: " << alglib_real_time <<*/ std::endl;



    return 0;
}
