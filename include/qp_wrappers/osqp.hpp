#ifndef QPWRAPPERS_OSQP_HPP
#define QPWRAPPERS_OSQP_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "problem.hpp"
#include <osqp.h>
#include <iostream>

namespace qp_wrappers {
    namespace osqp {
        template<typename T>
        typename qp<T>::Vector solve(const qp<T>& problem) {
            Eigen::SparseMatrix<T> sparse_q = problem.Q.sparseView();
            Eigen::SparseMatrix<T> sparse_a = problem.A.sparseView();

            sparse_q.makeCompressed();
            sparse_a.makeCompressed();


            // Workspace structures
            OSQPWorkspace *work;
            OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
            OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

             // Populate data
            if (data) {
                data->n = problem.variable_count;
                data->m = problem.A.rows();
                data->P = csc_matrix(
                                data->n, 
                                data->n, 
                                sparse_q.nonZeros(), 
                                sparse_q.valuePtr(), 
                                (c_int*)sparse_q.innerIndexPtr(), 
                                (c_int*)sparse_q.outerIndexPtr());
                data->q = (c_float*)problem.c.data();
                data->A = csc_matrix(data->m, 
                                     data->n, 
                                     sparse_a.nonZeros(),
                                     sparse_a.valuePtr(), 
                                     (c_int*)sparse_a.innerIndexPtr(), 
                                     (c_int*)sparse_a.outerIndexPtr());
                data->l = (c_float*)problem.lb.data();
                data->u = (c_float*)problem.ub.data();
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


            for(int i = 0; i < data->n; i++)
                std::cout << work->solution->x[i] << std::endl;
        }
    }
}

#endif