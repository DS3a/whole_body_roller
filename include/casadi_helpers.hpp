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
    std::memcpy(dm.ptr(), C.data(), sizeof(double)*C.rows()*C.cols());
    return dm;
    }

    // 2) Sparse -> DM (via dense)
    inline casadi::DM toDM(const Eigen::SparseMatrix<double>& S) {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> C = Eigen::MatrixXd(S);
    casadi::DM dm = casadi::DM::zeros(C.rows(), C.cols());
    std::memcpy(dm.ptr(), C.data(), sizeof(double)*C.rows()*C.cols());
    return dm;
    }

    // 3) Vector convenience (ensures n×1 shape)
    inline casadi::DM toDMcol(const Eigen::VectorXd& v) {
    casadi::DM dm = casadi::DM::zeros(v.size(), 1);
    std::memcpy(dm.ptr(), v.data(), sizeof(double)*v.size());
    return dm;
    }
}