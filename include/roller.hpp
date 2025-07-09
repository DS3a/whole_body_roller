/*
    The main control loop will be in this file,
    which will take joint data from the robot, and feed it to the dynamics model,
    along with any setpoints (task based, or joint based, TBD) 
    and update all the constraints that need to be updated 
    before solving a QP to determine the control decision variables.
*/
#pragma once

#include <memory>



namespace whole_body_roller {

    class Roller {
    public:
        // the decision variables should be read only
        std::shared_ptr<whole_body_roller::ControlDecisionVariables const> dec_v;
        std::shared_ptr<whole_body_roller::Dynamics> dynamics;

        std::shared_ptr<Eigen::VectorXd> joint_torques;

        std::vector<std::shared_ptr<whole_body_roller::Constraint>> constraints;

    public:
        Roller(std::shared_ptr<whole_body_roller::ControlDecisionVariables> dv, 
               std::shared_ptr<whole_body_roller::Dynamics> dyn);
        
        bool add_constraint(std::shared_ptr<whole_body_roller::Constraint> constraint);

        bool solve_qp();

        bool step();

    };
}