#include "roller.hpp"
#include "dynamics.hpp"

namespace whole_body_roller {
        // this function is just to construct the roller object
        // this function also adds the dynamics constraint to the roller which is to be there by default.
        // The dynamics constraint is the one that ensures that
        //      the joint accelerations are consistent with the dynamics model.
        // The dynamics object is also the only constraint that is allowed to modify dec_v
        Roller::Roller(std::shared_ptr<whole_body_roller::ControlDecisionVariables> dv, 
               std::shared_ptr<whole_body_roller::Dynamics> dyn);

        // this function is to add constraints to the controller.
        // The constraints could be on the contact forces, torques, 
        // or the joint accelerations (or the task space by using the jacobian to transform \ddot{q})
        bool Roller::add_constraint(std::shared_ptr<whole_body_roller::Constraint> constraint);

        // this function consolidates all the constraints, makes sure they're all valid
        //          adds equality and ineq constraints, to the qp and then solves them
        bool Roller::solve_qp();


        /*
         * This funciton is the main control loop, it will update the dynamics 
                and the parameters of the constraints based on dynamics->joint_positions_ and dynamics->joint_velocities_
         */
        bool Roller::step();


}
