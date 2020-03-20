#include <qp_wrappers/qpoases.hpp>
#include <qp_wrappers/problem.hpp>

int main() {

    QPWrappers::Problem<double> problem(0);
    std::cin >> problem;

    problem.regularize_Q(1e-6);

    QPWrappers::qpOASES::Engine<double> qpoasesEngine;
    qpoasesEngine.setPSDCheckEigenvalueTolerance(1e-6);
    
    
    QPWrappers::Problem<double>::Vector result;
    qpoasesEngine.init(problem, result);
    int cnt = 10000000;
    for(int i = 0; i < cnt; i++) {
        std::cout << i << "/" << cnt << std::endl;
        qpoasesEngine.next(problem, result);
        // std::cout << result;

        std::cout << "verification: " << problem.verify(result, 1e-10) << std::endl;
        std::cout << "objective value: " << problem.objective(result) << std::endl;
    }


    return 0;
}