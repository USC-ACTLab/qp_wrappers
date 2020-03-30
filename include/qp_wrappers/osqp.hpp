#ifndef QPWRAPPERS_OSQP_HPP
#define QPWRAPPERS_OSQP_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "problem.hpp"
#include <osqp.h>
#include <iostream>
#include "types.hpp"

namespace QPWrappers {
    namespace OSQP {
        
        /*
            A QP engine that solves consecutive QP instances where the result of the previous
            instance used as an initial guess to the next one unless an initial guess
            is provided.
        */
        template<typename T>
        class Engine {
            static_assert(std::is_same<T, c_float>::value);

            public:
                Engine(): initialized(false) {
                    settings = static_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
                    osqp_set_default_settings(settings);
                    settings->alpha = 1.0;
                    settings->verbose = false;
                    settings->max_iter = std::numeric_limits<c_int>::max();
                }

                Engine(const Engine& rhs) = delete;
                Engine& operator=(const Engine& rhs) = delete;

                Engine(Engine&& rhs) = delete;
                Engine& operator=(Engine&& rhs) = delete;

                ~Engine() {
                    c_free(settings);
                }

                /*
                    By how much are the constraints allowed to be violated?
                */
                void setFeasibilityTolerance(T tolerance) {
                    settings->eps_prim_inf = tolerance;
                }

                /*
                    Solve the first intance of the set of problems.
                    Load solution result to result.
                */
                OptReturnType init(const Problem<T>& problem, typename Problem<T>::Vector& result) {
                    OSQPWorkspace* work;

                    // setup data start
                    OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

                    data->n = problem.num_vars();

                    typename Problem<T>::Matrix QUpperTriangular = problem.Q().template triangularView<Eigen::Upper>();
                    Eigen::SparseMatrix<T> SparseQUpperTriangular = QUpperTriangular.sparseView();
                    SparseQUpperTriangular.makeCompressed();
                    data->P = csc_matrix(
                        problem.num_vars(),
                        problem.num_vars(),
                        SparseQUpperTriangular.nonZeros(),
                        SparseQUpperTriangular.valuePtr(),
                        SparseQUpperTriangular.innerIndexPtr(),
                        SparseQUpperTriangular.outerIndexPtr()
                    );

                    data->q = static_cast<c_float*>(c_malloc(sizeof(T) * problem.num_vars()));
                    std::memcpy(data->q, problem.c().data(), problem.num_vars() * sizeof(T));
                    // data->q = const_cast<T*>(problem.c().data());

                    typename Problem<T>::Matrix A = problem.A();
                    typename Problem<T>::Vector lb = problem.lb();
                    typename Problem<T>::Vector ub = problem.ub();
                    A.conservativeResize(problem.num_constraints() + problem.num_vars(), Eigen::NoChange_t());
                    lb.conservativeResize(problem.num_constraints() + problem.num_vars());
                    ub.conservativeResize(problem.num_constraints() + problem.num_vars());
                    A.block(problem.num_constraints(), 0, problem.num_vars(), problem.num_vars()).setIdentity();
                    lb.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.lbx();
                    ub.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.ubx();
                    Eigen::SparseMatrix<T> SparseA = A.sparseView();
                    SparseA.makeCompressed();
                    data->m = A.rows();
                    data->A = csc_matrix(
                        A.rows(),
                        A.cols(),
                        SparseA.nonZeros(),
                        SparseA.valuePtr(),
                        SparseA.innerIndexPtr(),
                        SparseA.outerIndexPtr()
                    );

                    data->l = lb.data();
                    data->u = ub.data();
                    // setup data end

                    osqp_setup(&work, data, settings);

                    osqp_solve(work);

                    OptReturnType return_value;

                    if(work->info->status_val == OSQP_SOLVED) {
                        loadResult(work, result, problem.num_vars());
                        initialized = true;
                        previous_result = result;
                        return_value = OptReturnType::Optimal;
                    } else if(work->info->status_val == OSQP_NON_CVX 
                           || work->info->status_val == OSQP_UNSOLVED) {

                        return_value = OptReturnType::Error;
                    } else if(work->info->status_val == OSQP_TIME_LIMIT_REACHED
                           || work->info->status_val == OSQP_SIGINT
                           || work->info->status_val == OSQP_MAX_ITER_REACHED
                           || work->info->status_val == OSQP_SOLVED_INACCURATE) {
                        return_value = OptReturnType::Unknown;
                    } else if(work->info->status_val == OSQP_DUAL_INFEASIBLE
                           || work->info->status_val == OSQP_PRIMAL_INFEASIBLE
                           || work->info->status_val == OSQP_PRIMAL_INFEASIBLE_INACCURATE
                           || work->info->status_val == OSQP_DUAL_INFEASIBLE_INACCURATE) {
                        return_value = OptReturnType::Infeasible;
                    }

                    free_osqp_data(data);

                    return return_value;
                }

                /*
                    Solve the next problem of the set of problems.
                */
                OptReturnType next(const Problem<T>& problem, typename Problem<T>::Vector& result) {
                    if(!initialized || problem.num_vars() != previous_result.rows()) {
                        initialized = false;
                        return init(problem, result);
                    }
                    OSQPWorkspace* work;

                    // setup data start
                    OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

                    data->n = problem.num_vars();

                    typename Problem<T>::Matrix QUpperTriangular = problem.Q().template triangularView<Eigen::Upper>();
                    Eigen::SparseMatrix<T> SparseQUpperTriangular = QUpperTriangular.sparseView();
                    SparseQUpperTriangular.makeCompressed();
                    data->P = csc_matrix(
                        problem.num_vars(),
                        problem.num_vars(),
                        SparseQUpperTriangular.nonZeros(),
                        SparseQUpperTriangular.valuePtr(),
                        SparseQUpperTriangular.innerIndexPtr(),
                        SparseQUpperTriangular.outerIndexPtr()
                    );

                    data->q = static_cast<c_float*>(c_malloc(sizeof(T) * problem.num_vars()));
                    std::memcpy(data->q, problem.c().data(), problem.num_vars() * sizeof(T));
                    // data->q = const_cast<T*>(problem.c().data());

                    typename Problem<T>::Matrix A = problem.A();
                    typename Problem<T>::Vector lb = problem.lb();
                    typename Problem<T>::Vector ub = problem.ub();
                    A.conservativeResize(problem.num_constraints() + problem.num_vars(), Eigen::NoChange_t());
                    lb.conservativeResize(problem.num_constraints() + problem.num_vars());
                    ub.conservativeResize(problem.num_constraints() + problem.num_vars());
                    A.block(problem.num_constraints(), 0, problem.num_vars(), problem.num_vars()).setIdentity();
                    lb.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.lbx();
                    ub.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.ubx();
                    Eigen::SparseMatrix<T> SparseA = A.sparseView();
                    SparseA.makeCompressed();
                    data->m = A.rows();
                    data->A = csc_matrix(
                        A.rows(),
                        A.cols(),
                        SparseA.nonZeros(),
                        SparseA.valuePtr(),
                        SparseA.innerIndexPtr(),
                        SparseA.outerIndexPtr()
                    );

                    data->l = lb.data();
                    data->u = ub.data();
                    // setup data end

                    osqp_setup(&work, data, settings);
                    osqp_warm_start_x(work, previous_result.data()); // warm start

                    osqp_solve(work);

                    OptReturnType return_value;

                    if(work->info->status_val == OSQP_SOLVED) {
                        loadResult(work, result, problem.num_vars());
                        previous_result = result;
                        return_value = OptReturnType::Optimal;
                    } else if(work->info->status_val == OSQP_NON_CVX 
                            || work->info->status_val == OSQP_UNSOLVED) {

                        return_value = OptReturnType::Error;
                    } else if(work->info->status_val == OSQP_TIME_LIMIT_REACHED
                            || work->info->status_val == OSQP_SIGINT
                            || work->info->status_val == OSQP_MAX_ITER_REACHED
                            || work->info->status_val == OSQP_SOLVED_INACCURATE) {
                        return_value = OptReturnType::Unknown;
                    } else if(work->info->status_val == OSQP_DUAL_INFEASIBLE
                            || work->info->status_val == OSQP_PRIMAL_INFEASIBLE
                            || work->info->status_val == OSQP_PRIMAL_INFEASIBLE_INACCURATE
                            || work->info->status_val == OSQP_DUAL_INFEASIBLE_INACCURATE) {
                        return_value = OptReturnType::Infeasible;
                    }

                    free_osqp_data(data);

                    return return_value;

                }

                /*
                    Solve the next problem of the set of problems with the given initial guess.
                */
                OptReturnType next(const Problem<T>& problem, typename Problem<T>::Vector& result, const typename Problem<T>::Vector& initial_guess) {
                    initialized = true;
                    previous_result = initial_guess;
                    return next(problem, result);
                }

            private:
                typename Problem<T>::Vector previous_result;
                bool initialized;

                OSQPSettings* settings;

                void loadResult(OSQPWorkspace* work, typename Problem<T>::Vector& result, typename Problem<T>::Index n) {
                    result.resize(n);
                    for(typename Problem<T>::Index i = 0; i < n; i++) {
                        result(i) = work->solution->x[i];
                    }
                }

                // OSQPData* setup_osqp_data(const Problem<T>& problem) {
                //     OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

                //     data->n = problem.num_vars();

                //     typename Problem<T>::Matrix QUpperTriangular = problem.Q().template triangularView<Eigen::Upper>();
                //     Eigen::SparseMatrix<T> SparseQUpperTriangular = QUpperTriangular.sparseView();
                //     SparseQUpperTriangular.makeCompressed();
                //     data->P = csc_matrix(
                //         problem.num_vars(),
                //         problem.num_vars(),
                //         SparseQUpperTriangular.nonZeros(),
                //         SparseQUpperTriangular.valuePtr(),
                //         SparseQUpperTriangular.innerIndexPtr(),
                //         SparseQUpperTriangular.outerIndexPtr()
                //     );

                //     // data->q = static_cast<c_float*>(c_malloc(sizeof(T) * problem.num_vars()));
                //     // std::memcpy(data->q, problem.c().data(), problem.num_vars() * sizeof(T));
                //     data->q = const_cast<T*>(problem.c().data());

                //     typename Problem<T>::Matrix A = problem.A();
                //     typename Problem<T>::Vector lb = problem.lb();
                //     typename Problem<T>::Vector ub = problem.ub();
                //     A.conservativeResize(problem.num_constraints() + problem.num_vars(), Eigen::NoChange_t());
                //     lb.conservativeResize(problem.num_constraints() + problem.num_vars());
                //     ub.conservativeResize(problem.num_constraints() + problem.num_vars());
                //     A.block(problem.num_constraints(), 0, problem.num_vars(), problem.num_vars()).setIdentity();
                //     lb.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.lbx();
                //     ub.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.ubx();
                //     Eigen::SparseMatrix<T> SparseA = A.sparseView();
                //     SparseA.makeCompressed();
                //     data->m = A.rows();
                //     data->A = csc_matrix(
                //         A.rows(),
                //         A.cols(),
                //         SparseA.nonZeros(),
                //         SparseA.valuePtr(),
                //         SparseA.innerIndexPtr(),
                //         SparseA.outerIndexPtr()
                //     );

                //     data->l = lb.data();
                //     data->u = ub.data();

                //     return data;
                // }

                void free_osqp_data(OSQPData* data) {
                    c_free(data->P);
                    c_free(data->q);
                    c_free(data->A);
                    c_free(data);
                }
        };
    }
}

#endif