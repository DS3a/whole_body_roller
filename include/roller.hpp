/*
    The main control loop will be in this file,
    which will take joint data from the robot, and feed it to the dynamics model,
    along with any setpoints (task based, or joint based, TBD) 
    and update all the constraints that need to be updated 
    before solving a QP to determine the control decision variables.
*/
#pragma once

#include <memory>
#include "constraint.hpp"

#include "dynamics.hpp"
// #include "QuadProg++.hh"
#include <casadi/casadi.hpp>

namespace whole_body_roller {

    class Roller {
    public:
        // the decision variables should be read only
        std::shared_ptr<whole_body_roller::ControlDecisionVariables const> dec_v;
        std::shared_ptr<whole_body_roller::Dynamics> dynamics;

        std::shared_ptr<Eigen::VectorXd> joint_torques;

        std::vector<std::shared_ptr<whole_body_roller::Constraint>> constraints;
        std::vector<std::shared_ptr<whole_body_roller::ConstraintHandler>> constraint_handlers;

    public:
        // this function is just to construct the roller object
        // this function also adds the dynamics constraint to the roller which is to be there by default.
        // The dynamics constraint is the one that ensures that
        //      the joint accelerations are consistent with the dynamics model.
        // The dynamics object is also the only constraint that is allowed to modify dec_v
        Roller(std::shared_ptr<whole_body_roller::Dynamics> dyn);

        // this function is to add constraints to the controller.
        // The constraints could be on the contact forces, torques, 
        // or the joint accelerations (or the task space by using the jacobian to transform \ddot{q})
        bool add_constraint(std::shared_ptr<whole_body_roller::Constraint> constraint, std::shared_ptr<whole_body_roller::ConstraintHandler> constraint_handler);

        // this function consolidates all the constraints, makes sure they're all valid
        //          adds equality and ineq constraints, to the qp and then solves them
        bool solve_qp();


        /*
         * This funciton is the main control loop, it will update the dynamics 
                and the parameters of the constraints based on dynamics->joint_positions_ and dynamics->joint_velocities_
         */
        bool step();

    };
}