#ifndef QPWRAPPERS_OSQP_HPP
#define QPWRAPPERS_OSQP_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "problem.hpp"
#include <osqp.h>
#include <iostream>

namespace qp_wrappers {
    namespace osqp {

        class solver{
            public:
                solver() {
                    settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
                    osqp_set_default_settings(settings);

                    settings->alpha = 1;
                    settings->verbose = false;
                }

                ~solver() {
                    c_free(settings);
                }

                void set_settings(OSQPSettings* settings) {
                    c_free(this->settings);
                    this->settings = settings;
                }

                return_type solve(const qp<c_float>& problem, qp<c_float>::Vector& primal_solution) {
                    qp<c_float>::Matrix q_upper_tri = problem.Q.triangularView<Eigen::Upper>();
                    Eigen::SparseMatrix<c_float> sparse_q = q_upper_tri.sparseView();
                    sparse_q.makeCompressed();


                    qp<c_float>::Matrix A = problem.A;
                    int m = A.rows();
                    qp<c_float>::Vector lb = problem.lb;
                    qp<c_float>::Vector ub = problem.ub;
                    A.conservativeResize(A.rows() + problem.variable_count, A.cols());
                    lb.conservativeResize(lb.rows() + problem.variable_count);
                    ub.conservativeResize(ub.rows() + problem.variable_count);
                    A.block(m, 0, problem.variable_count, problem.variable_count).setIdentity();
                    lb.block(m, 0, problem.variable_count, 1) = problem.lbx;
                    ub.block(m, 0, problem.variable_count, 1) = problem.ubx;
                    Eigen::SparseMatrix<c_float> sparse_a = A.sparseView();
                    sparse_a.makeCompressed();



                    // std::cout << "non zeros: " << std::endl;
                    // for(int i = 0; i < sparse_q.nonZeros(); i++) {
                    //     std::cout << sparse_q.valuePtr()[i] << " ";
                    // }
                    // std::cout << std::endl <<  "outerIndex: " ;

                    // for(int i = 0; i < problem.variable_count+1; i++) {
                    //     std::cout << sparse_q.outerIndexPtr()[i] << " ";
                    // }
                    // std::cout << std::endl << "innerIndex: ";
                    // for(int i = 0; i < sparse_q.nonZeros(); i++) {
                    //     std::cout << sparse_q.innerIndexPtr()[i] << " ";
                    // }
                    // std::cout << std::endl;

                    // Workspace structures
                    OSQPWorkspace *work;
                    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

                    data->n = problem.variable_count;
                    data->m = A.rows();
                    data->P = csc_matrix(
                                    data->n, 
                                    data->n, 
                                    sparse_q.nonZeros(),
                                    sparse_q.valuePtr(), 
                                    sparse_q.innerIndexPtr(), 
                                    sparse_q.outerIndexPtr());
                    data->q = const_cast<c_float*>(problem.c.data());
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

                    // cleanup
                    c_free(data->A);
                    c_free(data->P);

                    primal_solution.resize(problem.variable_count);
                    for(int i = 0; i < data->n; i++)
                        primal_solution(i) = work->solution->x[i];

                    return success;
                }
            private:
                OSQPSettings* settings;
        };

        template<typename T>
        typename qp<T>::Vector solve(const qp<T>& problem) {
            qp<c_float> cast_problem = problem.template cast<c_float>();
            qp<c_float>::Matrix q_upper_tri = cast_problem.Q.triangularView<Eigen::Upper>();
            Eigen::SparseMatrix<T> sparse_q = q_upper_tri.sparseView();
            Eigen::SparseMatrix<T> sparse_a = cast_problem.A.sparseView();

            sparse_q.makeCompressed();
            sparse_a.makeCompressed();



            std::cout << "non zeros: " << std::endl;
            for(int i = 0; i < sparse_q.nonZeros(); i++) {
                std::cout << sparse_q.valuePtr()[i] << " ";
            }
            std::cout << std::endl <<  "outerIndex: " ;

            for(int i = 0; i < problem.variable_count+1; i++) {
                std::cout << sparse_q.outerIndexPtr()[i] << " ";
            }
            std::cout << std::endl << "innerIndex: ";
            for(int i = 0; i < sparse_q.nonZeros(); i++) {
                std::cout << sparse_q.innerIndexPtr()[i] << " ";
            }
            std::cout << std::endl;

            // Workspace structures
            OSQPWorkspace *work;
            OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
            OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

             // Populate data
            if (data) {
                data->n = cast_problem.variable_count;
                data->m = cast_problem.A.rows();
                data->P = csc_matrix(
                                data->n, 
                                data->n, 
                                sparse_q.nonZeros(),
                                sparse_q.valuePtr(), 
                                sparse_q.innerIndexPtr(), 
                                sparse_q.outerIndexPtr());
                data->q = (c_float*)problem.c.data();

                data->A = csc_matrix(data->m, 
                                     data->n, 
                                     sparse_a.nonZeros(),
                                     sparse_a.valuePtr(), 
                                     sparse_a.innerIndexPtr(), 
                                     sparse_a.outerIndexPtr());
                
                data->l = cast_problem.lb.data();
                data->u = cast_problem.ub.data();
            }

            // Define solver settings as default
            if (settings) {
                osqp_set_default_settings(settings);
                settings->alpha = 1.0; // Change alpha parameter
            }

            // Setup workspace
            osqp_setup(&work, data, settings);

            // Solve Problem
            osqp_solve(work);

            // cleanup
            if (settings) c_free(settings);

            typename qp<T>::Vector soln(cast_problem.variable_count);
            for(int i = 0; i < data->n; i++)
                soln(i) = work->solution->x[i];

            return soln;
        }
    }
}

#endif