#include <qp_wrappers/qpoases.hpp>
#include <qp_wrappers/osqp.hpp>
#include <qp_wrappers/problem.hpp>
#include <iostream>

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

            problem.Q += 2 * x * x.transpose();
            problem.c += -2 * x * y;
        } else if (c == 's') {
            break;
        }
    }

    std::cout << std::endl <<  problem.Q/2 << std::endl << std::endl << problem.c << std::endl;

    auto solution = qp_wrappers::qpoases::solve(problem);

    qp_wrappers::osqp::solve(problem);
    std::cout << solution << std::endl;
}