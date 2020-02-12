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
            lbx.setConstant(variable_count, std::numeric_limits<T>::lowest());
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

        template<typename S>
        friend std::ostream& operator<<(std::ostream& os, const qp<S>& problem);
        template<typename S>
        friend std::istream& operator>>(std::istream& is, qp<S>& problem);

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

template<typename T>
std::ostream& operator<<(std::ostream& os, const qp<T>& problem) {
    auto before_precision = os.precision();
    
    os.precision(std::numeric_limits<T>::max_digits10);

    int n = problem.variable_count, m = problem.A.rows();
    os << n << " " << m << std::endl;
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            os << problem.Q(i, j) << " ";
        }
        os << std::endl;
    }

    for(int i = 0; i < m; i++) {
        for(int j = 0; j < n; j++) {
            os << problem.A(i, j) << " ";
        }
        os << std::endl;
    }

    for(int i = 0; i < m; i++) {
        os << problem.lb(i) << " ";
    }
    os << std::endl;


    for(int i = 0; i < m; i++) {
        os << problem.ub(i) << " ";
    }

    os << std::endl;

    for(int i = 0; i < n; i++) {
        os << problem.lbx(i) << " ";
    }
    os << std::endl;


    for(int i = 0; i < n; i++) {
        os << problem.ubx(i) << " ";
    }
    os << std::endl;

    for(int i = 0; i < n; i++) {
        os << problem.c(i) << " ";
    }
    os << std::endl;

    os.precision(before_precision);

    return os;
}

template<typename T>
std::istream& operator>>(std::istream& is, qp<T>& problem) {
    int n,m;
    is >> n >> m;
    problem.variable_count = n;
    problem.Q.resize(n, n);
    problem.c.resize(n);
    problem.lbx.resize(n);
    problem.ubx.resize(n);
    problem.A.resize(m, n);
    problem.lb.resize(m);
    problem.ub.resize(m);
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            is >> problem.Q(i, j);
        }
    }

    for(int i = 0; i < m; i++) {
        for(int j = 0; j < n; j++) {
            is >> problem.A(i, j);
        }
    }

    for(int i = 0; i < m; i++) {
        is >> problem.lb(i);
    }


    for(int i = 0; i < m; i++) {
        is >> problem.ub(i);
    }


    for(int i = 0; i < n; i++) {
        is >> problem.lbx(i);
    }


    for(int i = 0; i < n; i++) {
        is >> problem.ubx(i);
    }

    for(int i = 0; i < n; i++) {
        is >> problem.c(i);
    }

    return is;
}

}


#endif