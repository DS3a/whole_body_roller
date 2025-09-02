#pragma once

#include <casadi/casadi.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>


namespace casadi_helpers {
    // 1) Dense (MatrixXd or any dense expression) -> DM
    template <class Derived>
    casadi::DM toDM(const Eigen::MatrixBase<Derived>& M) {
    // Make a concrete, contiguous, column-major copy (eval() materializes expressions/views)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> C = M.eval();
    casadi::DM dm = casadi::DM::zeros(C.rows(), C.cols());
    std::cout << "shape of basee mat " << C.rows() << ", " << C.cols() << "\n";
    std::memcpy(dm.ptr(), C.data(), sizeof(double)*C.rows()*C.cols());
    std::cout << "done memcpy\n";
    return dm;
    }

    // 2) Sparse -> DM (via dense)
    inline casadi::DM toDM(const Eigen::SparseMatrix<double>& S) {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> C = Eigen::MatrixXd(S);
    casadi::DM dm = casadi::DM::zeros(C.rows(), C.cols());
    std::cout << "shape of sparse mat " << C.rows() << ", " << C.cols() << "\n";
    std::memcpy(dm.ptr(), C.data(), sizeof(double)*C.rows()*C.cols());
    return dm;
    }

    // 3) Vector convenience (ensures n×1 shape)
    inline casadi::DM toDMcol(const Eigen::VectorXd& v) {
    casadi::DM dm = casadi::DM::zeros(v.size(), 1);
    std::memcpy(dm.ptr(), v.data(), sizeof(double)*v.size());
    return dm;
    }

    class CaSolver {
    public:
        casadi::Opti opti;
        casadi::MX z; // Decision variables
        casadi::MX obj; // Objective function
        casadi::MX eq_con; // Equality constraints
        casadi::MX eq_bias; // Equality constraint bias
        casadi::MX ineq_con; // Inequality constraints
        casadi::MX ineq_bias; // Inequality constraint bias

        CaSolver(int ndv, int neq, int nineq) {
            this->opti = casadi::Opti();
            this->z = this->opti.variable(ndv); // decision variables
            if (neq > 0) {
                this->eq_con = this->opti.parameter(neq, ndv);
                this->eq_bias = this->opti.parameter(neq);
                this->opti.subject_to(casadi::MX::mtimes(eq_con, z) == eq_bias); // equality constraints
            }
            if (nineq > 0) {
                this->ineq_con = this->opti.parameter(nineq, ndv);
                this->ineq_bias = this->opti.parameter(nineq);
                this->opti.subject_to(casadi::MX::mtimes(ineq_con, z) >= ineq_bias); // inequality constraints
            }
            this->obj = casadi::MX::dot(z, z); // Initialize objective function to zero
            this->opti.minimize(this->obj); // Set the objective function
            this->opti.solver("ipopt");
        }
    };
}