#ifndef QPWRAPPERS_OSQP_HPP
#define QPWRAPPERS_OSQP_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "problem.hpp"
#include <osqp.h>
#include <iostream>
#include "types.hpp"

namespace qp_wrappers {
    namespace osqp {

        class solver {
            public:
                solver() {
                    settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
                    osqp_set_default_settings(settings);

                    settings->alpha = 1;
                    settings->verbose = false;
                    settings->max_iter = std::numeric_limits<c_int>::max();
                    settings->polish = 1;
                    settings->polish_refine_iter = 100;
                    // std::cout << settings->linsys_solver << std::endl;
                }

                ~solver() {
                    c_free(settings);
                }

                void set_settings(OSQPSettings* settings) {
                    c_free(this->settings);
                    this->settings = settings;
                }

                return_type solve(const qp<c_float>& problem, qp<c_float>::Vector& primal_solution) {
                    qp<c_float>::Matrix q_upper_tri = problem.Q().triangularView<Eigen::Upper>();
                    Eigen::SparseMatrix<c_float> sparse_q = q_upper_tri.sparseView();
                    sparse_q.makeCompressed();


                    qp<c_float>::Matrix A = problem.A();
                    // int m = A.rows();
                    qp<c_float>::Vector lb = problem.lb();
                    qp<c_float>::Vector ub = problem.ub();
                    A.conservativeResize(problem.num_constraints() + problem.num_vars(), Eigen::NoChange_t());
                    lb.conservativeResize(problem.num_constraints() + problem.num_vars());
                    ub.conservativeResize(problem.num_constraints() + problem.num_vars());
                    A.block(problem.num_constraints(), 0, problem.num_vars(), problem.num_vars()).setIdentity();
                    lb.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.lbx();
                    ub.block(problem.num_constraints(), 0, problem.num_vars(), 1) = problem.ubx();
                    Eigen::SparseMatrix<c_float> sparse_a = A.sparseView();
                    sparse_a.makeCompressed();


                    // Workspace structures
                    OSQPWorkspace *work;
                    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

                    data->n = problem.num_vars();
                    data->m = A.rows();
                    data->P = csc_matrix(
                                    data->n, 
                                    data->n, 
                                    sparse_q.nonZeros(),
                                    sparse_q.valuePtr(), 
                                    sparse_q.innerIndexPtr(), 
                                    sparse_q.outerIndexPtr());
                    data->q = const_cast<c_float*>(problem.c().data());
                    data->A = csc_matrix(data->m, 
                                        data->n, 
                                        sparse_a.nonZeros(),
                                        sparse_a.valuePtr(), 
                                        sparse_a.innerIndexPtr(), 
                                        sparse_a.outerIndexPtr());
                    
                    data->l = lb.data();
                    data->u = ub.data();

                    // Setup workspace
                    osqp_setup(&work, data, settings);

                    // Solve Problem
                    osqp_solve(work);

                    // std::cout << "osqp status_val: " << work->info->status_val << std::endl;
                    if(work->info->status_val == OSQP_SOLVED) {
                        primal_solution.resize(problem.num_vars());
                        for(int i = 0; i < data->n; i++)
                            primal_solution(i) = work->solution->x[i];

                        return success;
                    } else {
                        return infeasible;
                    }

                    // cleanup
                    c_free(data->A);
                    c_free(data->P);
                    c_free(data);
                }
            private:
                OSQPSettings* settings;
        };
    }
}

#endif