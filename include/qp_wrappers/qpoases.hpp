#ifndef QPWRAPPERS_QPOASES_HPP
#define QPWRAPPERS_QPOASES_HPP

#include <Eigen/Dense>
#include "problem.hpp"
#include <qpOASES.hpp>
#include <iostream>
#include "types.hpp"

namespace qp_wrappers {
    namespace qpoases {

        class solver {
            public:
                solver() {
                    options.setToDefault();
                    options.printLevel = qpOASES::PL_NONE;
                    nWSR = 10000;
                }

                void set_options(const qpOASES::Options& options) {
                    this->options = options;
                }

                void set_nwsr(int nwsr) {
                    this->nWSR = nwsr;
                }

                return_type solve(const qp<qpOASES::real_t>& problem, qp<qpOASES::real_t>::Vector& primal_soln) {
                    qpOASES::QProblem qpoases_problem(problem.variable_count, problem.A.rows());
                    qpoases_problem.setOptions(options);

                    auto return_value = 
                    qpoases_problem.init(problem.Q.data(),
                                        problem.c.data(),
                                        problem.A.data(),
                                        problem.lbx.data(),
                                        problem.ubx.data(),
                                        problem.lb.data(),
                                        problem.ub.data(),
                                        nWSR,
                                        NULL
                    );

                    if(return_value == qpOASES::SUCCESSFUL_RETURN) {
                        primal_soln.resize(problem.variable_count);
                        qpoases_problem.getPrimalSolution(primal_soln.data());
                        return success;
                    } else if (return_value == qpOASES::RET_MAX_NWSR_REACHED) {
                        return parameter_related_infeasibility;
                    } else if (return_value == qpOASES::RET_INIT_FAILED) {
                        return infeasible;
                    } else {
                        std::cerr << "what?" << std::endl;
                        exit(0);
                    }
                }
            private:
                qpOASES::Options options;
                int nWSR;
        };


    }
}

#endif