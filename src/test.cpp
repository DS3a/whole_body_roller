#include "constraint.hpp"
#include "control_decision_variables.hpp"
#include "task_space_constraints/frame_acceleration.hpp"
// #include "Quadprog++.hh"
#include <casadi/casadi.hpp>
#include <math.h>

#include <iostream>
#include <Eigen/Dense>

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

int main(int argc, const char **argv) {
    std::shared_ptr<whole_body_roller::ControlDecisionVariables> cdv; // Example initialization with 18 variables and 4 contact points

    cdv = std::make_shared<whole_body_roller::ControlDecisionVariables>(7, 2);

    std::cout << "Control Decision Variables initialized with nqdd: " << cdv->nv_ 
                << " and tau: " << cdv->ntau_
              << " and nc: " << cdv->nc_
               << std::endl;

    whole_body_roller::Constraint c(cdv, 1, whole_body_roller::constraint_type_t::INEQUALITY);
    
    cdv->nc_ = 2;
    c.set_qdd_constraints(Eigen::MatrixXd::Ones(1, 7));
    c.set_tau_constraints(Eigen::MatrixXd::Ones(1, 1));
    
    std::cout << "trying to set contact constraints " << 
      c.set_contact_constraints({
        Eigen::MatrixXd::Identity(2, 6), 
        Eigen::MatrixXd::Ones(2, 6), 
        Eigen::MatrixXd(2, 6)}) << std::endl;
      c.set_contact_constraints({
        Eigen::MatrixXd::Ones(1, 6), 
        Eigen::MatrixXd::Ones(1, 6)});
 
    c.ignore_contact_constraints(false);


    std::cout << c.get_constraint_matrix() << std::endl;
    std::cout << "checking if constraint is valid" << c.is_constraint_valid() << std::endl;
    std::cout << "now trying the opti stack\n";

    casadi::Opti opti;
    casadi::MX x = opti.variable(cdv->nv_ + cdv->nv_-6 + 6*cdv->nc_); // decision variables
    casadi::MX ineq_con = opti.parameter(1, cdv->nv_ + cdv->nv_-6 + 6*cdv->nc_);
    casadi::MX ineq_bias = opti.parameter(1);
    // casadi::MX y = opti.variable();

    // using casadi::dot;
    // casadi::MX cost = casadi::MX::dot(x, x);
    opti.minimize(casadi::MX::dot(x, x)); // objective function: minimize the sum of squares of decision variables
    // for (int i=0; i<c.num_constraints_; i++) {
    //   casadi::MX con;
    //   for (int j=0; j<c.get_constraint_matrix().row(i).size(); j++) {
    //     con += c.get_constraint_matrix().row(i)(j) * x(j);
    //   }
      // casadi::MX con = opti.parameter(c.get_constraint_matrix().row(i).size());
      // opti.subject_to(dot(con, x) <= c.constraint_bias(i)); // example constraints
    // }
    // using casadi::mtimes;
    // opti.set_value(ineq_con, eigenToMX(c.get_constraint_matrix()));
    // std::memcpy(ineq_con_dm.ptr(), c.get_constraint_matrix().data(), 14*14*sizeof(double));
    std::cout << "the shape of the constraint matrix is " << c.get_constraint_matrix().rows() << " x " << c.get_constraint_matrix().cols() << "\n";
    std::cout << "the shaep of the constarint bias is " << c.constraint_bias.size() << "\n";
     // std::memcpy(ineq_bias_dm.ptr(), c.constraint_bias.data(), 14*sizeof(double));
      // opti.set_value(ineq_bias, ineq_bias_dm);
    opti.subject_to(casadi::MX::mtimes(ineq_con, x) >= ineq_bias); // example constraints
    // casadi::Dict opts;
    // opts["print_time"] = false;

    // casadi::Dict ipopt_opts;
    // ipopt_opts["print_level"] = 0; // 0 = no output
    // ipopt_opts["sb"] = "yes";      // suppress IPOPT banner

    // opts["ipopt"] = ipopt_opts;

    opti.solver("ipopt");
    for (int i=0; i<100; i++) {
      opti.set_value(ineq_con, toDM(c.get_constraint_matrix()));

      opti.set_value(ineq_bias, toDMcol(5*Eigen::VectorXd::Ones(1)));
 
      
      // opti.solver("ipopt");
      auto sol = opti.solve();
      std::cout << "solution is " << sol.value(x) << "\n";
      std::cout << "the size of x is "  << sol.value(x).size().first << "\n";

    }
    return 0;
}