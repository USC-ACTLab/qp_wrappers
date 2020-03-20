#include <experimental/filesystem>
#include <qp_wrappers/problem.hpp>
#include <qp_wrappers/cplex.hpp>
#include <qp_wrappers/gurobi.hpp>
#include <qp_wrappers/qpoases.hpp>
#include <qp_wrappers/osqp.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

namespace fs = std::experimental::filesystem;


int main(int argc, char* argv[]) {
    if(argc != 2) {
        throw (std::string("Usage: ")
                + argv[0]
                + std::string(" <problems-folder>"));
    }


    QPWrappers::CPLEX::Engine<double> cplexEngine;
    cplexEngine.setFeasibilityTolerance(1e-8);


    QPWrappers::GUROBI::Engine<double> gurobiEngine;
    gurobiEngine.setFeasibilityTolerance(1e-8);


    QPWrappers::qpOASES::Engine<double> qpoasesEngine;


    QPWrappers::OSQP::Engine<double> osqpEngine;
    osqpEngine.setFeasibilityTolerance(1e-8);

    double cplex_total_time = 0;
    int cplex_success_count = 0;
    double gurobi_total_time = 0;
    int gurobi_success_count = 0;
    double qpoases_total_time = 0;
    int qpoases_success_count = 0;
    double osqp_total_time = 0;
    int osqp_success_count = 0;

    int instance_count = 0;

    for(auto& p: fs::directory_iterator(argv[1])) {
        QPWrappers::Problem<double> problem{0};
        std::ifstream inp(p.path().string());
        inp >> problem;
        
        QPWrappers::Problem<double>::Vector initial_guess(problem.num_vars());
        for(QPWrappers::Problem<double>::Index i = 0; i < problem.num_vars(); i++) {
            inp >> initial_guess(i);
        }

        inp.close();

        problem.regularize_Q(1e-6);

        QPWrappers::Problem<double>::Vector result;

        try {
            auto start = std::chrono::system_clock::now();
            auto ret = cplexEngine.next(problem, result, initial_guess);
            if(ret == QPWrappers::OptReturnType::Optimal) {
                std::cout << "cplex verification: " << problem.verify(result, 1e-6) << std::endl;
                std::cout << "cplex objective value: " << problem.objective(result) << std::endl;
                auto end = std::chrono::system_clock::now();
                double duration = std::chrono::duration<double>(end - start).count();
                std::cout << "cplex time: " << duration << std::endl;
                cplex_total_time += duration;
                cplex_success_count++;
            }
        } catch(...) {
            std::cout << "cplex exp." << std::endl;
        }
        result.resize(0);

        try {
            auto start = std::chrono::system_clock::now();
            auto ret = osqpEngine.next(problem, result, initial_guess);
            if(ret == QPWrappers::OptReturnType::Optimal) {
                std::cout << "osqp verification: " << problem.verify(result, 1e-6) << std::endl;
                std::cout << "osqp objective value: " << problem.objective(result) << std::endl;
                auto end = std::chrono::system_clock::now();
                double duration = std::chrono::duration<double>(end - start).count();
                std::cout << "osqp time: " << duration << std::endl;
                osqp_total_time += duration;
                osqp_success_count++;
            }
        } catch(...) {
            std::cout << "osqp exp." << std::endl;
        }
        result.resize(0);

        try {
            auto start = std::chrono::system_clock::now();
            auto ret = qpoasesEngine.next(problem, result, initial_guess);
            if(ret == QPWrappers::OptReturnType::Optimal) {
                std::cout << "qpoases verification: " << problem.verify(result, 1e-6) << std::endl;
                std::cout << "qpoases objective value: " << problem.objective(result) << std::endl;
                auto end = std::chrono::system_clock::now();
                double duration = std::chrono::duration<double>(end - start).count();
                std::cout << "qpoases time: " << duration << std::endl;
                qpoases_total_time += duration;
                qpoases_success_count++;
            }
        } catch(...) {
            std::cout << "qpoases exp." << std::endl;
        }
        result.resize(0);

        try {
            auto start = std::chrono::system_clock::now();
            auto ret = gurobiEngine.next(problem, result, initial_guess);
            if(ret == QPWrappers::OptReturnType::Optimal) {
                std::cout << "gurobi verification: " << problem.verify(result, 1e-6) << std::endl;
                std::cout << "gurobi objective value: " << problem.objective(result) << std::endl;
                auto end = std::chrono::system_clock::now();
                double duration = std::chrono::duration<double>(end - start).count();
                std::cout << "gurobi time: " << duration << std::endl;
                gurobi_total_time += duration;
                gurobi_success_count++;
            }
        } catch(...) {
            std::cout << "gurobi exp." << std::endl;
        }
        result.resize(0);


        instance_count++;
    }

    std::cout << "cplex average time: " << cplex_total_time / instance_count << ", cplex success rate: " << cplex_success_count << "/" << instance_count << std::endl;
    std::cout << "osqp average time: " << osqp_total_time / instance_count << ", osqp success rate: " << osqp_success_count << "/" << instance_count << std::endl;
    std::cout << "qpoases average time: " << qpoases_total_time / instance_count << ", qpoases success rate: " << qpoases_success_count << "/" << instance_count << std::endl;
    std::cout << "gurobi average time: " << gurobi_total_time / instance_count << ", gurobi success rate: " << gurobi_success_count << "/" << instance_count << std::endl;
    return 0;
}