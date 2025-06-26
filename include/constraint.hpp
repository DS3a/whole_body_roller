#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "control_decision_variables.hpp"

namespace whole_body_roller {
    // enum for constraint type, either equality constraint or an inequality constraint
    // enum 

    class Constraint{
    public:
        std::shared_ptr<whole_body_roller::ControlDecisionVariables> dec_v;
        int num_constraints_;
        
        // Need one matrix of size (num_constraints, nv)
        Eigen::MatrixXd qdd_constraints;

        // One matrix of size (num_constraints, nv-5)
        Eigen::MatrixXd tau_constraints;

        // One vector of size nc with matrices of size (num_constraints, 6)
        std::vector<Eigen::MatrixXd> contact_constraints;

    public:
        Constraint(std::shared_ptr<whole_body_roller::ControlDecisionVariables> dv, int num_constraints) {
            this->dec_v = dv;
            this->num_constraints_ = num_constraints;

            this->qdd_constraints = Eigen::MatrixXd::Zero(this->num_constraints_, this->dec_v->nv_);
            this->tau_constraints = Eigen::MatrixXd::Zero(this->num_constraints_, this->dec_v->nv_-6);
            this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Zero(this->num_constraints_, 6));
            // this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Ones(this->num_constraints_, 6));
            // this is just to test whether the contact constraints get concatenated properly
        }

        bool set_qdd_constraints(Eigen::MatrixXd constraints) {
            if (constraints.rows() == this->num_constraints_ && constraints.cols() == this->dec_v->nv_) {
                this->qdd_constraints = constraints;
                return true;
            }
            return false;
        }

        // TODO create functions to set constraint matrices for tau, and contacts
        // TODO create a function to set the constraint bias
        // TODO create an enum to declare whether it is an equality or an inequality constraint


        Eigen::MatrixXd get_constraint_matrix() {
            Eigen::MatrixXd constraint_matrix(this->num_constraints_, 2*(this->dec_v->nv_)-6+6*(this->dec_v->nc_));
            Eigen::MatrixXd ct_constraints(this->num_constraints_, 6*(this->dec_v->nc_));
            int current_col = 0;
            for (const auto& m : this->contact_constraints) {
                // this is to test whether the concatenation is fine
                // ct_constraints.block(0, currentCol, m.rows(), m.cols()) = m*(currentCol/12.0+1);
                ct_constraints.block(0, current_col, m.rows(), m.cols()) = m;
                current_col += m.cols();
            }
            constraint_matrix << this->qdd_constraints, this->tau_constraints, ct_constraints;

            return constraint_matrix;
        }

    };
}