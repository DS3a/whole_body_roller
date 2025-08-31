#include "roller.hpp"
#include <iostream>

namespace whole_body_roller {
    // this function is just to construct the roller object
    // this function also adds the dynamics constraint to the roller which is to be there by default.
    // The dynamics constraint is the one that ensures that
    //      the joint accelerations are consistent with the dynamics model.
    // The dynamics object is also the only constraint that is allowed to modify dec_v
    Roller::Roller(std::shared_ptr<whole_body_roller::Dynamics> dyn) {

        // this->dec_v = dv;
        this->dynamics = dyn;
        this->dec_v = this->dynamics->get_dec_v();
        this->add_constraint(this->dynamics->dynamics_constraint, this->dynamics);
    }

    // this function is to add constraints to the controller.
    // The constraints could be on the contact forces, torques, 
    // or the joint accelerations (or the task space by using the jacobian to transform \ddot{q})
    bool Roller::add_constraint(std::shared_ptr<whole_body_roller::Constraint> constraint, std::shared_ptr<whole_body_roller::ConstraintHandler> constraint_handler) {
        if (constraint->dec_v->nv_ == this->dec_v->nv_) {
            this->constraints.push_back(constraint);
            // we can add the constraint handler to the constraint handler list
            // so that we can update the constraints before solving the qp
            this->constraint_handlers.push_back(constraint_handler);
            return true;
        }
        return false; // size mismatch
    }

    // this function consolidates all the constraints, makes sure they're all valid
    //          adds equality and ineq constraints, to the qp and then solves them
    bool Roller::solve_qp() {
        int num_eq_constraints = 0;
        int num_ineq_constraints = 0;
        for (auto& constraint_handler : this->constraint_handlers) {
            if (!constraint_handler->constraint_is_active()) {
                continue; // skip the constraint if it is not active
            }
            auto constraint = constraint_handler->constraint; // get the constraint from the handler
            if (!constraint->is_constraint_valid()) {
                return false; // if any constraint is not valid, we return false
            }
            if (constraint->constraint_type_ == whole_body_roller::constraint_type_t::EQUALITY) {
                num_eq_constraints += constraint->num_constraints_;
            } else if (constraint->constraint_type_ == whole_body_roller::constraint_type_t::INEQUALITY) {
                num_ineq_constraints += constraint->num_constraints_;
            }
        }

        int nvars = 2 * (this->dec_v->nv_) - 6 + 6 * (this->dec_v->nc_); // number of variables in the qp
        // Create equality the constraint matrix
        Eigen::MatrixXd eq_constraint_matrix(nvars, num_eq_constraints);
        Eigen::VectorXd eq_constraint_bias(num_eq_constraints);
        Eigen::MatrixXd ineq_constraint_matrix(nvars, num_ineq_constraints);
        Eigen::VectorXd ineq_constraint_bias(num_ineq_constraints);

        // iterate through all constraints and append the constraint matrices and biases
        int eq_con_col = 0;
        int ineq_con_col = 0;
        for (auto& constraint_handler : this->constraint_handlers) {
            auto constraint = constraint_handler->constraint; // get the constraint from the handler
            if (!constraint_handler->constraint_is_active()) {
                continue; // skip the constraint if it is not active
            }
            if (constraint->constraint_type_ == whole_body_roller::constraint_type_t::EQUALITY) {
                // append the equality constraint matrix and bias
                eq_constraint_matrix.block(0, eq_con_col, nvars, constraint->num_constraints_) = constraint->get_constraint_matrix().transpose();
                eq_constraint_bias.segment(eq_con_col, constraint->num_constraints_) = constraint->constraint_bias;
                eq_con_col += constraint->num_constraints_;
            } else if (constraint->constraint_type_ == whole_body_roller::constraint_type_t::INEQUALITY) {
                // append the inequality constraint matrix and bias
                ineq_constraint_matrix.block(0, ineq_con_col, nvars, constraint->num_constraints_) = constraint->get_constraint_matrix().transpose();
                ineq_constraint_bias.segment(ineq_con_col, constraint->num_constraints_) = constraint->constraint_bias;
                ineq_con_col += constraint->num_constraints_;
            }
        }
        std::cout << "added all constraints \n";

        Eigen::MatrixXd G = Eigen::MatrixXd::Identity(nvars, nvars); // G matrix for the qp
        Eigen::VectorXd g0 = Eigen::VectorXd::Zero(nvars); // g0 vector for the qp

        Eigen::VectorXd x(nvars); // solution vector for the qp

        // QuadProgpp::Solver qp_solver;
        // std::cout << "attempting solve \n";
        // QuadProgpp::Status::Value v = qp_solver.solve(G, g0, eq_constraint_matrix, eq_constraint_bias, 
        //                 ineq_constraint_matrix, ineq_constraint_bias, x);
        
        // std::cout << "attempted solve \n";
        // if (v == 0) {
        //     this->joint_torques = std::make_shared<Eigen::VectorXd>(x.segment(this->dec_v->nv_, this->dec_v->nv_ - 6)); // extract the joint torques from the solution vector
        //     std::cout << "solve successful : " << this->joint_torques->transpose() << "\n";
        //     return true;
        // }

        return false;
    }


    // TODO we need to provide an interface to the constraint handlers
    // we need to be able to call update_joint_states() on the dynamics
    // and we need to be able to call set_acceleration_target() on the frame acceleration constraints
   /*
    * This funciton is the main control loop, it will update the dynamics 
           and the parameters of the constraints based on dynamics->joint_positions_ and dynamics->joint_velocities_
    */
    bool Roller::step() {
        // first update the dynamics constraint as that changes dec_v
        // then update the frame/joint acceleration constraints
        for (auto& handler : this->constraint_handlers) {
            if (!handler->update_constraint()) {
                std::cout << "checking if constraint is valid\n";
                return false; // if any constraint fails to update, we return false
            }
        }

        std::cout << "updated all constraints \n";
        return this->solve_qp(); // solve the qp with the updated constraints
    }


}
