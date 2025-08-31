#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "control_decision_variables.hpp"

namespace whole_body_roller {
    // enum for constraint type, either equality constraint or an inequality constraint
    enum constraint_type_t {
        EQUALITY,
        INEQUALITY
    };

    class Constraint {
    public:
        // the decision variables should be read only
        std::shared_ptr<whole_body_roller::ControlDecisionVariables const> dec_v;
        int num_constraints_;
        
        // Need one matrix of size (num_constraints, nv)
        Eigen::MatrixXd qdd_constraints;

        // One matrix of size (num_constraints, nv-5)
        Eigen::MatrixXd tau_constraints;


        // One vector(list) of size nc with matrices of size (num_constraints, 6)
        std::vector<Eigen::MatrixXd> contact_constraints;

        bool contacts_are_considered = false; // set to false by default

        // Bias vector for the constraints, size (num_constraints, 1)
        Eigen::VectorXd constraint_bias;

        constraint_type_t constraint_type_;

    public:
        Constraint(std::shared_ptr<whole_body_roller::ControlDecisionVariables> dv, int num_constraints, constraint_type_t constraint_type);
        // { 
        //    this->dec_v = dv;
        //     this->num_constraints_ = num_constraints;

        //     this->constraint_type_ = constraint_type;

        //     this->qdd_constraints = Eigen::MatrixXd::Zero(this->num_constraints_, this->dec_v->nv_);
        //     this->tau_constraints = Eigen::MatrixXd::Zero(this->num_constraints_, this->dec_v->nv_-6);
        //     this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Zero(this->num_constraints_, 6));
            
            
        //     // this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Ones(this->num_constraints_, 6));
        //     // this is just to test whether the contact constraints get concatenated properly
        // }

        // this function updates the number of contact points
        void update_nc(); 
        //     this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Zero(this->num_constraints_, 6));
        // }

        bool set_qdd_constraints(Eigen::MatrixXd constraints);
        // {
        //     if (constraints.rows() == this->num_constraints_ && constraints.cols() == this->dec_v->nv_) {
        //         this->qdd_constraints = constraints;
        //         return true;
        //     }
        //     return false;
        // }

        // DONE create functions to set constraint matrices for tau, and contacts
        bool set_tau_constraints(Eigen::MatrixXd constraints);
        // {
        //     if (constraints.rows() == this->num_constraints_ && constraints.cols() == this->dec_v->nv_-6) {
        //         this->tau_constraints = constraints;
        //         return true;
        //     }
        //     return falselocation;
        // }

        // Done create a function to set the contact constraints
        bool set_contact_constraints(std::vector<Eigen::MatrixXd> constraints);
        // {
        //     if (constraints.size() == this->dec_v->nc_) {
        //         for (size_t i = 0; i < this->dec_v->nc_; ++i) {
        //             if (constraints[i].rows() != this->num_constraints_ || constraints[i].cols() != 6) {
        //                 return false; // Invalid size for one of the contact constraints
        //             }
        //             this->contact_constraints[i] = constraints[i];
        //         }
        //         return true;
        //     }
        //     return false;
        // }
        
        // Done create a function to set the constraint bias
        bool set_constraint_bias(Eigen::VectorXd bias);
        // {
        //     if (bias.size() == this->num_constraints_) {
        //         this->constraint_bias = bias;
        //         return true;
        //     }
        //     return false;
        // }

        // DONE create an enum to declare whether it is an equality or an inequality constraint


        // this function basically checks the dimensions of all
        // the decision variable matrices and the bias vector
        // and makes sure it matches whatever is there in dec_v and num_constraints
        bool is_constraint_valid();

        void ignore_contact_constraints(bool ignore_contacts);
        void ignore_contact_constraints();

        Eigen::MatrixXd get_constraint_matrix();
        // {
        //     Eigen::MatrixXd constraint_matrix(this->num_constraints_, 2*(this->dec_v->nv_)-6+6*(this->dec_v->nc_));
        //     Eigen::MatrixXd ct_constraints(this->num_constraints_, 6*(this->dec_v->nc_));
        //     int current_col = 0;
        //     for (const auto& m : this->contact_constraints) {
        //         // this is to test whether the concatenation is fine
        //         // ct_constraints.block(0, currentCol, m.rows(), m.cols()) = m*(currentCol/12.0+1);
        //         ct_constraints.block(0, current_col, m.rows(), m.cols()) = m;
        //         current_col += m.cols();
        //     }
        //     constraint_matrix << this->qdd_constraints, this->tau_constraints, ct_constraints;

        //     return constraint_matrix;
        // }

    };

    // this is a class for non-static constraints, e.g., dynamics, and 
    // frame acceleration constraints. I want to update all the constraints before adding them to the qp
    class ConstraintHandler {
    public:
        bool is_constraint_active = true; // default value is true
        std::shared_ptr<whole_body_roller::Constraint> constraint;

        virtual bool update_constraint() = 0; // this function basically updates the non-static constraints with new values
        // it also ensures that the constraint will pass `is_constraint_valid()` check for anything where the `contacts_are_considered`
        virtual ~ConstraintHandler() = default;

        void set_constraint_active(bool active) {
            this->is_constraint_active = active;
        }

        bool constraint_is_active() const {
            return this->is_constraint_active;
        }

    };



}