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
        using Index = Eigen::Index;

        qp(Index N, Index M = 0):
                A_mtr(M, N),
                lb_mtr(M),
                ub_mtr(M) {
            Q_mtr.setConstant(N, N, 0);
            c_mtr.setConstant(N, 0);
            lbx_mtr.setConstant(N, std::numeric_limits<T>::lowest());
            ubx_mtr.setConstant(N, std::numeric_limits<T>::max());
        }

        inline bool is_ubx_unbounded(Index var_idx) const {
            return ubx_mtr(var_idx) == std::numeric_limits<T>::max();
        }

        inline bool is_lbx_unbounded(Index var_idx) const {
            return lbx_mtr(var_idx) == std::numeric_limits<T>::lowest();
        }

        inline Index num_vars() const {
            return c_mtr.rows();
        }

        inline Index num_constraints() const {
            return A_mtr.rows();
        }

        void set_constraint(Index constraint_idx, const RowVector& coeff, T low, T up) {
            if(coeff.cols() != A_mtr.cols()) {
                throw std::domain_error(
                    std::string("Problem has ")
                    + std::to_string(A_mtr.cols())
                    + std::string(" variables, but the provided row vector for constraint has ")
                    + std::to_string(coeff.cols())
                    + std::string(" columns.")
                );
            }
            
            A_mtr.row(constraint_idx) = coeff;
            ub_mtr(constraint_idx) = up;
            lb_mtr(constraint_idx) = low;
        }

        void add_constraint(const RowVector& coeff, T low, T up) {
            if(coeff.cols() != A_mtr.cols()) {
                throw std::domain_error (
                    std::string("Problem has ")
                    + std::to_string(A_mtr.cols())
                    + std::string(" variables, but the provided row vector for constraint has ")
                    + std::to_string(coeff.cols())
                    + std::string(" columns.")
                );
            }

            A_mtr.conservativeResize(A_mtr.rows() + 1, Eigen::NoChange_t());
            ub_mtr.conservativeResize(ub_mtr.rows() + 1, Eigen::NoChange_t());
            lb_mtr.conservativeResize(lb_mtr.rows() + 1, Eigen::NoChange_t());

            A_mtr.row(A_mtr.rows() - 1) = coeff;
            ub_mtr(ub_mtr.rows() - 1) = up;
            lb_mtr(lb_mtr.rows() - 1) = low;
        }

        void set_var_limits(Index var_idx, T low, T up) {
            lbx_mtr(var_idx) = low;
            ubx_mtr(var_idx) = up;
        }

        void add_Q(const Matrix& Q) {
            if(Q.rows() != Q_mtr.rows() || Q.cols() != Q_mtr.cols()) {
                throw std::domain_error(
                            std::string("Q of the problem is ")
                            + std::to_string(Q_mtr.rows())
                            + std::string("x")
                            + std::to_string(Q_mtr.cols())
                            + std::string(" but Q of size ")
                            + std::to_string(Q.rows())
                            + std::string("x")
                            + std::to_string(Q.cols())
                            + std::string(" was provided to add to it.")
                            );
            }
            Q_mtr += Q;
        }

        void add_c(const Vector& c) {
            if(c.rows() != c_mtr.rows()) {
                throw std::domain_error(
                            std::string("c of the problem has ")
                            + std::to_string(c_mtr.rows())
                            + std::string(" columns, but the provided c has")
                            + std::to_string(c.rows())
                            + std::string(" rows.")
                            );
            }
            c_mtr += c;
        }

        const Matrix& Q() const {
            return Q_mtr;
        }

        const Vector& c() const {
            return c_mtr;
        }

        const Matrix& A() const {
            return A_mtr;
        }

        const Vector& lb() const {
            return lb_mtr;
        }

        const Vector& ub() const {
            return ub_mtr;
        }
 
        const Vector& lbx() const {
            return lbx_mtr;
        }

        const Vector& ubx() const {
            return ubx_mtr;
        }


        template<class S>
        qp<S> cast() const {
            qp<S> new_problem;
            new_problem.Q_mtr = Q_mtr.template cast<S>();
            new_problem.A_mtr = A_mtr.template cast<S>();
            new_problem.c_mtr = c_mtr.template cast<S>();
            new_problem.lb_mtr = lb_mtr.template cast<S>();
            new_problem.ub_mtr = ub_mtr.template cast<S>();
            new_problem.lbx_mtr = lbx_mtr.template cast<S>();
            new_problem.ubx_mtr = ubx_mtr.template cast<S>();
            return new_problem;
        }

        bool is_consistent() const {
            bool consistent = true;
            for(int i = 0; i < num_vars(); i++) {
                if(ubx_mtr(i) < lbx_mtr(i)) {
                    consistent = false;
                    std::cout << "bound on variable " << i << " is inconsistent. lbx " << lbx_mtr(i) << " ubx " << ubx_mtr(i) << std::endl;
                }
            }

            for(int i = 0; i < num_constraints(); i++) {
                if(ub_mtr(i) < lb_mtr(i)) {
                    consistent = false;
                    std::cout << "bound on constraint " << i << " is inconsistent. lb " << lb_mtr(i) << " ub " << ub_mtr(i) << std::endl; 
                }
            }
            return consistent;
        }

        bool verify(const Vector& solution) const {
            std::cout 
                    << "obj: "  
                    << 0.5 * solution.transpose() * Q_mtr * solution + solution.transpose() * c_mtr 
                    << std::endl;

            Vector constraint_mtr = A_mtr * solution;

            bool constraints_satisfied = true;
            for(int i = 0; i < num_constraints(); i++) {
                constraints_satisfied = constraints_satisfied && lb_mtr(i) <= constraint_mtr(i) && constraint_mtr(i) <= ub_mtr(i);
                if(!(lb_mtr(i) <= constraint_mtr(i) && constraint_mtr(i) <= ub_mtr(i)))
                    std::cout << "constraint " << i << " violated: " 
                              << ": lb " << lb_mtr(i) << "<=" << constraint_mtr(i) << " " << (lb_mtr(i) <= constraint_mtr(i)) 
                              << " ub " << constraint_mtr(i) <<  "<=" << ub_mtr(i) << " " << (constraint_mtr(i) <= ub_mtr(i)) << std::endl;
            }

            bool bounds_satisfied = true;
            for(int i = 0; i < num_vars(); i++) {
                bounds_satisfied = bounds_satisfied && lbx_mtr(i) <= solution(i) && solution(i) <= ubx_mtr(i); 
                if(!(lbx_mtr(i) <= solution(i) && solution(i) <= ubx_mtr(i)))
                    std::cout << "bounds for variable " << i << " violated: " 
                              << ": lb " << lbx_mtr(i) <<  "<=" << solution(i) << " " << (lbx_mtr(i) <= solution(i)) 
                              << " ub " << solution(i) << "<=" << ubx_mtr(i) << " " << (solution(i) <= ubx_mtr(i)) << std::endl; 
            }
            
            return constraints_satisfied && bounds_satisfied;
        }

        template<typename S>
        friend std::ostream& operator<<(std::ostream& os, const qp<S>& problem);
        template<typename S>
        friend std::istream& operator>>(std::istream& is, qp<S>& problem);

    private:

        /*
        * minimize 1/2 x^T Q x + c^T x
        * subject to
        *   lb <= Ax <= ub
        *   lbx <= x <= ubx
        */


        qp() {} // for internal operations (like casting)

        Matrix Q_mtr, A_mtr;
        Vector c_mtr, lb_mtr, ub_mtr, lbx_mtr, ubx_mtr;
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const qp<T>& problem) {
    auto before_precision = os.precision();
    
    os.precision(std::numeric_limits<T>::max_digits10);

    int n = problem.A_mtr.cols(), m = problem.A_mtr.rows();
    os << n << " " << m << std::endl;
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            os << problem.Q_mtr(i, j) << " ";
        }
        os << std::endl;
    }

    for(int i = 0; i < m; i++) {
        for(int j = 0; j < n; j++) {
            os << problem.A_mtr(i, j) << " ";
        }
        os << std::endl;
    }

    for(int i = 0; i < m; i++) {
        os << problem.lb_mtr(i) << " ";
    }
    os << std::endl;


    for(int i = 0; i < m; i++) {
        os << problem.ub_mtr(i) << " ";
    }

    os << std::endl;

    for(int i = 0; i < n; i++) {
        os << problem.lbx_mtr(i) << " ";
    }
    os << std::endl;


    for(int i = 0; i < n; i++) {
        os << problem.ubx_mtr(i) << " ";
    }
    os << std::endl;

    for(int i = 0; i < n; i++) {
        os << problem.c_mtr(i) << " ";
    }
    os << std::endl;

    os.precision(before_precision);

    return os;
}

template<typename T>
std::istream& operator>>(std::istream& is, qp<T>& problem) {
    int n,m;
    is >> n >> m;
    // problem.variable_count = n;
    problem.Q_mtr.resize(n, n);
    problem.c_mtr.resize(n);
    problem.lbx_mtr.resize(n);
    problem.ubx_mtr.resize(n);
    problem.A_mtr.resize(m, n);
    problem.lb_mtr.resize(m);
    problem.ub_mtr.resize(m);
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            is >> problem.Q_mtr(i, j);
        }
    }

    for(int i = 0; i < m; i++) {
        for(int j = 0; j < n; j++) {
            is >> problem.A_mtr(i, j);
        }
    }

    for(int i = 0; i < m; i++) {
        is >> problem.lb_mtr(i);
    }


    for(int i = 0; i < m; i++) {
        is >> problem.ub_mtr(i);
    }


    for(int i = 0; i < n; i++) {
        is >> problem.lbx_mtr(i);
    }


    for(int i = 0; i < n; i++) {
        is >> problem.ubx_mtr(i);
    }

    for(int i = 0; i < n; i++) {
        is >> problem.c_mtr(i);
    }

    return is;
}

}


#endif