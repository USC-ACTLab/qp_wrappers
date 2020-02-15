#ifndef QPWRAPPERS_CGAL_HPP
#define QPWRAPPERS_CGAL_HPP

#include "problem.hpp"
#include "types.hpp"
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <limits>
#include <CGAL/MP_Float.h>

namespace qp_wrappers{

namespace cgal {

    class solver {
        public:

            solver() {

            }

            template<typename T>
            return_type solve(const qp<T>& problem, typename qp<T>::Vector& primal_solution) {
                typedef CGAL::MP_Float ET;
                typedef CGAL::Quadratic_program<T> Program;
                typedef CGAL::Quadratic_program_solution<ET> Solution;

                typename qp<T>::Matrix A(problem.A.rows() * 2, problem.variable_count);
                typename qp<T>::Vector ub(problem.A.rows() * 2);
                std::vector<CGAL::Comparison_result> comparisons(problem.A.rows() * 2);
                int constraint_count = 0;
                for(int i = 0; i < problem.A.rows(); i++) {
                    if(problem.lb(i) == problem.ub(i)) {
                        comparisons[constraint_count] = CGAL::EQUAL;
                        A.block(constraint_count, 0, 1, problem.variable_count) = problem.A.block(i, 0, 1, problem.variable_count);
                        ub(constraint_count) = problem.ub(i);
                        constraint_count++;
                    } else {
                        comparisons[constraint_count] = CGAL::SMALLER;
                        A.block(constraint_count, 0, 1, problem.variable_count) = problem.A.block(i, 0, 1, problem.variable_count);
                        ub(constraint_count) = problem.ub(i);
                        constraint_count++;

                        comparisons[constraint_count] = CGAL::SMALLER;
                        A.block(constraint_count, 0, 1, problem.variable_count) = -1 * problem.A.block(i, 0, 1, problem.variable_count);
                        ub(constraint_count) = -1 * problem.lb(i);
                        constraint_count++;
                    }
                }
                A.conservativeResize(constraint_count, A.cols());
                ub.conservativeResize(constraint_count);

                // std::cout << problem.A.rows() << " " << problem.A.cols() << " " 
                        //   << problem.variable_count << " " << A.rows() << " " << A.cols() << " " << ub.rows() << std::endl;

                Program qpr(CGAL::SMALLER, false, std::numeric_limits<T>::lowest(), false, std::numeric_limits<T>::max());
                
                for(int i = 0; i < problem.variable_count; i++) {
                    for(int j = 0; j <= i; j++) {
                        qpr.set_d(i, j, problem.Q(i, j));
                    }

                    qpr.set_c(i, problem.c(i));
                    if(problem.lbx(i) <= std::numeric_limits<T>::lowest()) {
                        qpr.set_l(i, false);
                    } else
                        qpr.set_l(i, true, problem.lbx(i));

                    if(problem.ubx(i) >= std::numeric_limits<T>::max()) {
                        qpr.set_u(i, false);
                    } else
                        qpr.set_u(i, true, problem.ubx(i));
                    
                }

                for(int i = 0; i < A.rows(); i++) {
                    for(int j = 0; j < problem.variable_count; j++) {
                        qpr.set_a(j, i, A(i, j));
                    }
                    qpr.set_b(i, ub(i));
                    qpr.set_r(i, comparisons[i]);
                    
                }

                qpr.set_c0(0);


                // std::cout << "n: " << qpr.get_n() << std::endl;
                // std::cout << "m: " << qpr.get_m() << std::endl;
                // std::cout << "A: " << std::endl;
                // for(int i = 0; i < qpr.get_m(); i++) {
                //     for(int j = 0; j < qpr.get_n(); j++) {
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

                Solution s = CGAL::solve_quadratic_program(qpr, ET());
                assert(s.solves_quadratic_program(qpr));

                if(s.is_infeasible())
                    return infeasible;

                primal_solution.resize(problem.variable_count);
                for(int i = 0; i < problem.variable_count; i++) {
                    /*
                    * hate to_double!
                    */
                    const auto& quot = *(s.variable_values_begin() + i);
                    // std::cout << quot << std::endl;
                    primal_solution(i) = CGAL::to_double(quot);
                }

                return success;
            }
        private:
    };
}
}


#endif