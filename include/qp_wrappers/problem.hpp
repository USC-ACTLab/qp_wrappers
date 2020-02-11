#ifndef QPWRAPPERS_PROBLEM_HPP
#define QPWRAPPERS_PROBLEM_HPP

#include <Eigen/Dense>
#include <limits>
#include <iostream>


namespace qp_wrappers {

template<typename T>
class qp {
    public:
        using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
        using RowVector = Eigen::Matrix<T, 1, Eigen::Dynamic>;

        qp(unsigned int vc):
                A(0, vc),
                lb(0),
                ub(0),
                variable_count(vc) {
            Q.setConstant(variable_count, variable_count, 0);
            c.setConstant(variable_count, 0);
            lbx.setConstant(variable_count, std::numeric_limits<T>::min());
            ubx.setConstant(variable_count, std::numeric_limits<T>::max());
            
        }


        void add_constraint(const RowVector& coeff, T low, T up) {
            A.conservativeResize(A.rows() + 1, variable_count);
            ub.conservativeResize(ub.rows() + 1, variable_count);
            lb.conservativeResize(lb.rows() + 1, variable_count);

            A.block(A.rows() - 1, 0, 1, A.cols()) = coeff;
            ub(ub.rows() - 1) = up;
            lb(lb.rows() - 1) = low;
        }

        void add_var_limit(const Eigen::Index var_idx, T low, T up) {
            lbx(var_idx) = low;
            ubx(var_idx) = up;
        }

        template<class S>
        qp<S> cast() const {
            qp<S> new_problem(variable_count);
            new_problem.Q = Q.template cast<S>();
            new_problem.A = A.template cast<S>();
            new_problem.c = c.template cast<S>();
            new_problem.lb = lb.template cast<S>();
            new_problem.ub = ub.template cast<S>();
            new_problem.lbx = lbx.template cast<S>();
            new_problem.ubx = ubx.template cast<S>();
            return new_problem;
        }

        /*
        * minimize 1/2 x^T Q x + c^T x
        * subject to
        *   lb <= Ax <= ub
        *   lbx <= x <= ubx
        */

        Matrix Q, A;
        Vector c, lb, ub, lbx, ubx;
        unsigned int variable_count;
};

}


#endif