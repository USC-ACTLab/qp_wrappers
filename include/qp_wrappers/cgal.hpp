#ifndef QPWRAPPERS_CGAL_HPP
#define QPWRAPPERS_CGAL_HPP

#include "problem.hpp"
#include "types.hpp"
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <limits>

namespace qp_wrappers{

namespace cgal {

    class solver {
        public:

            solver() {

            }

            template<typename T>
            return_type solve(const qp<T>& problem, typename qp<T>::Vector& primal_solution) {
                typedef CGAL::Quadratic_program<T> Program;
                typedef CGAL::Quadratic_program_solution<T> Solution;

                typename qp<T>::Matrix A = problem.A;
                A.conservativeResize(A.rows() * 2, A.cols());
                A.block(problem.A.rows(), 0, problem.A.rows(), problem.A.cols()) = -1 * problem.A;
                typename qp<T>::Vector ub = problem.ub;
                ub.conservativeResize(ub.rows() * 2, 1);
                ub.block(problem.ub.rows(), 0, problem.ub.rows(), 1) = -1 * problem.lb;


                Program qpr(CGAL::SMALLER, false, std::numeric_limits<T>::lowest(), false, std::numeric_limits<T>::max());
                
                for(int i = 0; i < problem.variable_count; i++) {
                    for(int j = 0; j <= i; j++) {
                        qpr.set_d(i, j, problem.Q(i, j));
                    }

                    qpr.set_c(i, problem.c(i));
                    qpr.set_l(i, false, problem.lbx(i));                    
                    qpr.set_u(i, false, problem.ubx(i));
                    
                }

                for(int i = 0; i < A.rows(); i++) {
                    for(int j = 0; j < problem.variable_count; j++) {
                        qpr.set_a(j, i, A(i, j));
                    }
                    qpr.set_b(i, ub(i));
                    
                }

                qpr.set_c0(0);

                // std::cout << qpr << std::endl;

                // std::cout << "n: " << qpr.get_n() << std::endl;
                // std::cout << "m: " << qpr.get_m() << std::endl;
                // std::cout << "A: " << std::endl;
                // for(int i = 0; i < qpr.get_m(); i++) {
                //     for(int j = 0; qpr.get_n(); j++) {
                //         std::cout << *(*(qpr.get_a() + j) + i) << " ";
                //     }
                //     std::cout << std::endl;
                // }
                // std::cout << "b: ";
                // for(int i = 0; i < qpr.get_m(); i++) {
                //     std::cout << *(qpr.get_b() + i) << " ";
                // }
                // std::cout << std::endl;
                
                // std::cout << "2D: " << std::endl;
                // for(int i = 0; i < qpr.get_n(); i++) {
                //     for(int j = 0; j <= i; j++) {
                //         std::cout << *(*(qpr.get_d() + i) + j) << " ";
                //     }
                //     std::cout << std::endl;
                // }

                // std::cout << "c: ";
                // for(int i = 0; i < qpr.get_n(); i++) {
                //     std::cout << *(qpr.get_c() + i) << " ";
                // }
                // std::cout << std::endl;

                // std::cout << "l: ";
                // for(int i = 0; i < qpr.get_n(); i++) {
                //     std::cout << *(qpr.get_l() + i) << " ";
                // }
                // std::cout << std::endl;

                // std::cout << "u: ";
                // for(int i = 0; i < qpr.get_n(); i++) {
                //     std::cout << *(qpr.get_u() + i) << " ";
                // }
                // std::cout << std::endl;

                Solution s = CGAL::solve_quadratic_program(qpr, T());
                assert(s.solves_quadratic_program(qpr));

                if(s.is_infeasible())
                    return infeasible;

                primal_solution.resize(problem.variable_count);
                for(int i = 0; i < problem.variable_count; i++) {
                    /*
                    * hate to_double!
                    */
                    const auto& quot = *(s.variable_values_begin() + i);
                    primal_solution(i) = quot.numerator() / quot.denominator();
                }

                return success;
            }
        private:
    };
}
}


#endif