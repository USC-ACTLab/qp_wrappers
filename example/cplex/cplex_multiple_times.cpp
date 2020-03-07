#include <qp_wrappers/cplex.hpp>
#include <qp_wrappers/problem.hpp>

int main() {

    QPWrappers::Problem<double> problem(0);
    std::cin >> problem;

    QPWrappers::CPLEX::Engine<double> cplexEngine;
    cplexEngine.setFeasibilityTolerance(1e-8);
    cplexEngine.setPSDCheckEigenvalueTolerance(1e-8);
    
    
    QPWrappers::Problem<double>::Vector result;
    cplexEngine.init(problem, result);

    for(int i = 0; i < 10000000; i++) {
        cplexEngine.next(problem, result);

        std::cout << "verification: " << problem.verify(result, 1e-7) << std::endl;
        std::cout << "objective value: " << problem.objective(result) << std::endl;
    }


    return 0;
}