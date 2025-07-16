#include "roller.hpp"

namespace whole_body_roller {
    // this function is just to construct the roller object
    // this function also adds the dynamics constraint to the roller which is to be there by default.
    // The dynamics constraint is the one that ensures that
    //      the joint accelerations are consistent with the dynamics model.
    // The dynamics object is also the only constraint that is allowed to modify dec_v
    Roller::Roller(std::shared_ptr<whole_body_roller::ControlDecisionVariables> dv, 
            std::shared_ptr<whole_body_roller::Dynamics> dyn) {

        this->dec_v = dv;
        this->dynamics = dyn;
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
    // bool Roller::solve_qp();


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
                return false; // if any constraint fails to update, we return false
            }
        }

        return this->solve_qp(); // solve the qp with the updated constraints
    }


}
