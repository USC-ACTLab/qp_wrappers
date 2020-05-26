#ifndef QPWRAPPERS_qpOASES_HPP
#define QPWRAPPERS_qpOASES_HPP

#include "problem.hpp"
#include "types.hpp"
#include <iostream>
#include <qpOASES/QProblem.hpp>

namespace QPWrappers {
    namespace qpOASES {
        
        /*
            A QP engine that solves consecutive QP instances where the result of the previous
            instance used as an initial guess to the next one unless an initial guess
            is provided.
        */
        template<typename T>
        class Engine {
            static_assert(std::is_same<T, ::qpOASES::real_t>::value);

            public:
                Engine(): psd_tolerance(0), nWSR(10000), initialized(false) {
                    options.setToDefault();
                    options.printLevel = ::qpOASES::PL_NONE;
                }

                Engine(const Engine& rhs) = delete;
                Engine& operator=(const Engine& rhs) = delete;

                Engine(Engine&& rhs) = delete;
                Engine& operator=(Engine&& rhs) = delete;

                ~Engine() {
                }

                /*
                    By how much are the eigenvalues of the Q matrix are allowed be below 0 during PSD check?
                */
                void setPSDCheckEigenvalueTolerance(T tolerance) {
                    psd_tolerance = tolerance;
                }

                void setnWSR(T nwsr) {
                    nWSR = nwsr;
                }

                /*
                    Solve the first intance of the set of problems.
                    Load solution result to result.
                */
                OptReturnType init(const Problem<T>& problem, typename Problem<T>::Vector& result) {
                    ::qpOASES::QProblem qpoases_problem = create_qpoases_problem(problem);

                    auto return_value = qpoases_problem.init(
                        problem.Q().data(),
                        problem.c().data(),
                        problem.A().data(),
                        problem.lbx().data(),
                        problem.ubx().data(),
                        problem.lb().data(),
                        problem.ub().data(),
                        nWSR,
                        NULL
                    );

                    OptReturnType ret_val = load_and_return_optimization_result(return_value, qpoases_problem, problem, result);

                    if(ret_val == OptReturnType::Optimal) {
                        initialized = true;
                        previous_result = result;
                    } else {
                        initialized = false;
                    }

                    return ret_val;
                }

                /*
                    Solve the next problem of the set of problems. Initializes the engine if not initialized before.
                    If initialized, uses the previous result as the starting point.
                */
                OptReturnType next(const Problem<T>& problem, typename Problem<T>::Vector& result) {
                    if(!initialized || problem.num_vars() != previous_result.rows()) {
                        initialized = false;
                        return init(problem, result);
                    }

                    ::qpOASES::QProblem qpoases_problem = create_qpoases_problem(problem);

                    auto return_value = qpoases_problem.init(
                        problem.Q().data(),
                        problem.c().data(),
                        problem.A().data(),
                        problem.lbx().data(),
                        problem.ubx().data(),
                        problem.lb().data(),
                        problem.ub().data(),
                        nWSR,
                        NULL,
                        previous_result.data()
                    );

                    OptReturnType ret_val = load_and_return_optimization_result(return_value, qpoases_problem, problem, result);

                    if(ret_val == OptReturnType::Optimal) {
                        previous_result = result;
                    }

                    return ret_val;
                }

                /*
                    Solve the next problem of the set of problems with the given initial guess.
                */
                OptReturnType next(const Problem<T>& problem, typename Problem<T>::Vector& result, const typename Problem<T>::Vector& initial_guess) {
                    previous_result = initial_guess;
                    initialized = true;

                    return next(problem, result);
                }

                void setFeasibilityTolerance(T val) {}

            private:
                /*
                    Creates the qpOASES problem instance according to options stored, and the type of the problem
                */
                ::qpOASES::QProblem create_qpoases_problem(const Problem<T>& problem) {
                    ::qpOASES::QProblem qpoases_problem;

                    if(problem.is_Q_pd()) {
                        qpoases_problem = ::qpOASES::QProblem(problem.num_vars(), problem.num_constraints(), ::qpOASES::HST_POSDEF);
                        options.enableRegularisation = ::qpOASES::BT_FALSE;
                    } else if(problem.is_Q_psd(psd_tolerance)) {
                        qpoases_problem = ::qpOASES::QProblem(problem.num_vars(), problem.num_constraints(), ::qpOASES::HST_SEMIDEF);
                        options.enableRegularisation = ::qpOASES::BT_TRUE;
                    } else {
                        qpoases_problem = ::qpOASES::QProblem(problem.num_vars(), problem.num_constraints(), ::qpOASES::HST_INDEF);
                    }

                    qpoases_problem.setOptions(options);

                    return qpoases_problem;
                }

                /*
                    Loads solution from qpoases problem instance to the result if it is solved in the given
                    qpoases_problem.
                */
                OptReturnType load_and_return_optimization_result(::qpOASES::returnValue return_value, 
                                                               const ::qpOASES::QProblem& qpoases_problem, 
                                                               const Problem<T>& problem,
                                                               typename Problem<T>::Vector& result) 
                {
                    if(return_value == ::qpOASES::SUCCESSFUL_RETURN) {
                        result.resize(problem.num_vars());
                        qpoases_problem.getPrimalSolution(result.data());
                        return OptReturnType::Optimal;
                    } else if (return_value == ::qpOASES::RET_MAX_NWSR_REACHED) {
                        return OptReturnType::Error;
                    } else if(return_value == ::qpOASES::RET_INIT_FAILED) {
                        return OptReturnType::Infeasible;
                    }

                    return OptReturnType::Unknown;
                }

                bool initialized;
                typename Problem<T>::Vector previous_result;

                T psd_tolerance;
                ::qpOASES::int_t nWSR;
                ::qpOASES::Options options;


        };
    }
}

#endif