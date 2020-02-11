#ifndef QPWRAPPERS_QPOASES_HPP
#define QPWRAPPERS_QPOASES_HPP

#include <Eigen/Dense>
#include "problem.hpp"
#include <qpOASES.hpp>
#include <iostream>

namespace qp_wrappers {
    namespace qpoases {
        template<typename T>
        typename qp<T>::Vector solve(const qp<T>& problem) {
            qp<qpOASES::real_t> cast_problem = problem.template cast<qpOASES::real_t>();

            qpOASES::QProblem qpoases_problem(cast_problem.variable_count, cast_problem.A.rows());
            int nWSR = 100000;
            auto return_value = 
            qpoases_problem.init(cast_problem.Q.data(),
                                 cast_problem.c.data(),
                                 cast_problem.A.data(),
                                 cast_problem.lbx.data(),
                                 cast_problem.ubx.data(),
                                 cast_problem.lb.data(),
                                 cast_problem.ub.data(),
                                 nWSR,
                                 NULL
            );

            if(return_value == qpOASES::SUCCESSFUL_RETURN) {
                typename qp<T>::Vector soln(problem.variable_count);
                qpoases_problem.getPrimalSolution(soln.data());
                return soln;
            } else if (return_value == qpOASES::RET_MAX_NWSR_REACHED) {
                std::cerr << "qpOASES::RET_MAX_NWSR_REACHED" << std::endl;
                exit(0);
            } else if (return_value == qpOASES::RET_INIT_FAILED) {
                std::cerr << "qpOASES::RET_INIT_FAILED" << std::endl;
                exit(0);
            }
        }
    }
}

#endif