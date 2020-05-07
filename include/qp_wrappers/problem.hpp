#ifndef QPWRAPPERS_PROBLEM_HPP
#define QPWRAPPERS_PROBLEM_HPP

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <limits>
#include <iostream>


namespace QPWrappers {

/**
    Defines a quadratic program in the form 
         minimize 1/2 x^T Q x + c^T x
         subject to
           lb <= Ax <= ub
           lbx <= x <= ubx
    where Q is a symmetric matrix.
*/
template<typename T>
class Problem {
    public:
        using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
        using RowVector = Eigen::Matrix<T, 1, Eigen::Dynamic>;
        using Index = Eigen::Index;

        /*
            Construct a QP with N variables with M constraints.
            M can be omitted, in which case required constraints must
            be added one by one.
        */
        Problem(Index N, Index M = 0):
                A_mtr(M, N),
                lb_mtr(M),
                ub_mtr(M),
                is_Q_psd_calculated(false) {
            Q_mtr.setConstant(N, N, 0);
            c_mtr.setConstant(N, 0);
            lbx_mtr.setConstant(N, std::numeric_limits<T>::lowest());
            ubx_mtr.setConstant(N, std::numeric_limits<T>::max());
        }

        /*
        * Resets the problem so that there is constraint, objective is 0, and
        * there is no upper and lower limit
        */
        void reset() {
            A_mtr = Matrix(0, this->num_vars());
            lb_mtr = Vector(0);
            ub_mtr = Vector(0);
            Q_mtr.setZero();
            c_mtr.setZero();
            lbx_mtr.setConstant(this->num_vars(), std::numeric_limits<T>::lowest());
            ubx_mtr.setConstant(this->num_vars(), std::numeric_limits<T>::max());
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

        /*
            Sets the constraint with index constraint_idx.
            After the operation, low <= coeff * x <= up is enforced by the
            row with index constraint_idx of A, lb, and ub.
        */
        void set_constraint(Index constraint_idx, const RowVector& coeff, T low, T up) {
            if(coeff.cols() != num_vars()) {
                throw std::domain_error(
                    std::string("Problem has ")
                    + std::to_string(num_vars())
                    + std::string(" variables, but the provided row vector for constraint has ")
                    + std::to_string(coeff.cols())
                    + std::string(" columns.")
                );
            }

            if(constraint_idx >= num_constraints()) {
                throw std::domain_error(
                    std::string("constraint index out of range. ")
                    + std::string("constraint_idx: ") 
                    + std::to_string(constraint_idx)
                    + std::string(", num constraints: ") 
                    + std::to_string(num_constraints())
                );
            }
            
            A_mtr.row(constraint_idx) = coeff;
            ub_mtr(constraint_idx) = up;
            lb_mtr(constraint_idx) = low;
        }

        /*
            Adds a new constraint which enforces
                low <= coeff * x <= up
        */
        void add_constraint(const RowVector& coeff, T low, T up) {
            if(coeff.cols() != num_vars()) {
                throw std::domain_error (
                    std::string("Problem has ")
                    + std::to_string(num_vars())
                    + std::string(" variables, but the provided row vector for constraint has ")
                    + std::to_string(coeff.cols())
                    + std::string(" columns.")
                );
            }

            A_mtr.conservativeResize(A_mtr.rows() + 1, Eigen::NoChange);
            ub_mtr.conservativeResize(ub_mtr.rows() + 1, Eigen::NoChange);
            lb_mtr.conservativeResize(lb_mtr.rows() + 1, Eigen::NoChange);

            A_mtr.row(A_mtr.rows() - 1) = coeff;
            ub_mtr(ub_mtr.rows() - 1) = up;
            lb_mtr(lb_mtr.rows() - 1) = low;
        }

        /*
            Sets the limits of variable with index var_idx. Enforces
                low <= x[var_idx] <= up
        */
        void set_var_limits(Index var_idx, T low, T up) {
            if(var_idx >= num_vars()) {
                throw std::domain_error(
                    std::string("Problem has ")
                    + std::to_string(num_vars())
                    + std::string(" variables. set_var_limits is used for variable with index ")
                    + std::to_string(var_idx)
                );
            }

            lbx_mtr(var_idx) = low;
            ubx_mtr(var_idx) = up;
        }

        /*
            Adds given matrix to the Q of the problem.
            If given matrix is not symmetric, makes it symmetric first.
        */
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

            ensure_Q_symmetry();

            // set psd calculated flag to false
            // is_Q_psd_calculated = false;
        }

        /*
        * Adds given matrix to the block of Q of the problem where
        * block starts from row i and column j and spans Q.rows() rows
        * and Q.cols() columns
        */
        void add_Q_block(Index i, Index j, const Matrix& Q) {
            if(i + Q.rows() > Q_mtr.rows() || j + Q.cols() > Q_mtr.cols()) {
                throw std::domain_error(
                    std::string("given Q block matrix runs of the Q of the problem")
                );
            }

            Q_mtr.block(i, j, Q.rows(), Q.cols()) += Q;
            ensure_Q_symmetry();
        }

        /*
            return if Q is a PSD matrix
            eigen values are allowed to be more than -tolerance.
        */
        bool is_Q_psd(T tolerance = 0) const {
            // compute of Q is psd on the fly, and store the result into m_is_Q_psd.
            Eigen::EigenSolver<Matrix> eigen_solver(Q_mtr, false);
            const auto& eigen_values = eigen_solver.eigenvalues();

            for(Index i = 0; i < eigen_values.rows(); i++) {
                auto eigen_value = eigen_values(i);
                if(eigen_value.real() < -tolerance) {
                    return false;
                }
            }
            return true;
        }

        /*
            Regulatizes Q to be a PSD if it is not and the minimum eigenvalue is not less than
            psd_tolerance
        */
        void regularize_Q(T psd_tolerance = 0) {
            Eigen::EigenSolver<Matrix> eigen_solver(Q_mtr, false);
            const auto& eigen_values = eigen_solver.eigenvalues();

            T min_eig = std::numeric_limits<T>::max();
            for(Index i = 0; i < eigen_values.rows(); i++) {
                auto eigen_value = eigen_values(i);
                min_eig = std::min(min_eig, eigen_value.real());
            }
            // std::cout << std::endl;
            while(min_eig < 0 && min_eig >= -psd_tolerance) {
                Q_mtr += Matrix::Identity(num_vars(), num_vars()) * psd_tolerance;

                Eigen::EigenSolver<Matrix> eigen_solver(Q_mtr, false);
                const auto& eigen_values = eigen_solver.eigenvalues();
                min_eig = std::numeric_limits<T>::max();
                for(Index i = 0; i < eigen_values.rows(); i++) {
                    auto eigen_value = eigen_values(i);
                    min_eig = std::min(min_eig, eigen_value.real());
                }
            }
        }

        /*
            return if Q is a PD matrix
        */
        bool is_Q_pd() const {
            Eigen::EigenSolver<Matrix> eigen_solver(Q_mtr, false);
            const auto& eigen_values = eigen_solver.eigenvalues();

            for(Index i = 0; i < eigen_values.rows(); i++) {
                auto eigen_value = eigen_values(i);
                if(eigen_value.real() <= 0) {
                    return false;
                }
            }
            return true;
        }

        /*
            Adds given vector to c of the problem.
        */
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

        /*
        * Adds given vector to the block of c of the problem where
        * block starts from row i and spans c.rows() rows
        */
        void add_c_block(Index i, const Vector& c) {
            if(i + c.rows() > c_mtr.rows()) {
                throw std::domain_error(
                    std::string("given c block is out of bounds of the original c")
                );
            }

            c_mtr.block(i, 0, c.rows(), 1) += c;
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

        /*
            Cast problem into type S
        */
        template<class S>
        Problem<S> cast() const {
            Problem<S> new_problem;
            new_problem.Q_mtr = Q_mtr.template cast<S>();
            new_problem.A_mtr = A_mtr.template cast<S>();
            new_problem.c_mtr = c_mtr.template cast<S>();
            new_problem.lb_mtr = lb_mtr.template cast<S>();
            new_problem.ub_mtr = ub_mtr.template cast<S>();
            new_problem.lbx_mtr = lbx_mtr.template cast<S>();
            new_problem.ubx_mtr = ubx_mtr.template cast<S>();
            new_problem.m_is_Q_psd = m_is_Q_psd;
            new_problem.is_Q_psd_calculated = is_Q_psd_calculated;

            return new_problem;
        }

        /*
            Simply check if constraints are consistent.
            Basically checks if any lower bound is more than any upper bound.
        */
        bool is_consistent() const {
            for(int i = 0; i < num_vars(); i++) {
                if(ubx_mtr(i) < lbx_mtr(i)) {
                    return false;
                }
            }

            for(int i = 0; i < num_constraints(); i++) {
                if(ub_mtr(i) < lb_mtr(i)) {
                    return false;
                }
            }

            return true;
        }

        /*
            Verifies that solution is consistent with the constraints,
            that is it doesn't violate any constraint under the given tolarance.
            Violation by tolerance amount is accepted
        */
        bool verify(const Vector& solution, T tolerance = 0) const {

            if(solution.rows() != num_vars()) {
                throw std::domain_error(
                    std::string("Problem has ")
                    + std::to_string(num_vars())
                    + std::string(" variables, but given solution has ")
                    + std::to_string(solution.rows())
                    + std::string(" rows.")
                );
            }

            Vector constraint_mtr = A_mtr * solution;

            for(int i = 0; i < num_constraints(); i++) {
                if(lb_mtr(i) - tolerance > constraint_mtr(i)) {
                    return false;
                }

                if(constraint_mtr(i) > ub_mtr(i) + tolerance) {
                    return false;
                }
            }

            for(int i = 0; i < num_vars(); i++) {
                if(lbx_mtr(i) - tolerance > solution(i)) {
                    return false;
                }

                if(solution(i) > ubx_mtr(i) + tolerance) {
                    return false; 
                }
            }
            
            return true;
        }

        T objective(const Vector& solution) const {
            if(solution.rows() != num_vars()) {
                throw std::domain_error(
                    std::string("Problem has ")
                    + std::to_string(num_vars())
                    + std::string(" variables, but given solution has ")
                    + std::to_string(solution.rows())
                    + std::string(" rows.")
                );
            }

            return (solution.transpose() * Q() * solution + c().transpose() * solution)(0, 0);
        }

        template<typename S>
        friend std::ostream& operator<<(std::ostream& os, const Problem<S>& problem);
        template<typename S>
        friend std::istream& operator>>(std::istream& is, Problem<S>& problem);

    private:
        Problem() {} // for internal operations (like casting)

        Matrix Q_mtr, A_mtr;
        Vector c_mtr, lb_mtr, ub_mtr, lbx_mtr, ubx_mtr;
        bool m_is_Q_psd, is_Q_psd_calculated;

        static constexpr Eigen::NoChange_t no_change();

        void ensure_Q_symmetry() {
            for(Index i = 0; i < Q_mtr.rows(); i++) {
                for (Index j = i+1; j < Q_mtr.cols(); j++) {
                    Q_mtr(i, j) = Q_mtr(j, i) = (Q_mtr(i, j) / 2) + (Q_mtr(j, i) / 2);
                }
            }
        }

};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Problem<T>& problem) {
    auto before_precision = os.precision();
    
    os.precision(std::numeric_limits<T>::max_digits10);

    int n = problem.A_mtr.cols(), m = problem.A_mtr.rows();
    os << std::fixed << n << " " << m << std::endl;
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
std::istream& operator>>(std::istream& is, Problem<T>& problem) {
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

    problem.ensure_Q_symmetry();


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