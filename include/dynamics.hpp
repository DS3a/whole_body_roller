/*
    The main constraint to make sure the torques obey the dynamics of the system

    This also contains the model data from the urdf/mjcf, 
    and can be used for inverse dynamics control, contact constraints, etc.,
 */

#pragma once
#include <memory>
#include <map>


#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "control_decision_variables.hpp"
#include "constraint.hpp"
#include "end_effector.hpp"

namespace whole_body_roller {
    class Dynamics : public whole_body_roller::ConstraintHandler {
    public:
        std::shared_ptr<whole_body_roller::ControlDecisionVariables> dec_v;
        std::shared_ptr<pinocchio::Model> model_;
        std::shared_ptr<pinocchio::Data> data_;
        bool is_dynamics_ready;
        int added_end_effectors;
        int free_end_effectors;
        int num_end_effectors_;
        std::map <std::string, int> end_effector_map_;
        std::vector<whole_body_roller::EndEffector> end_effectors;


        Eigen::VectorXd joint_positions_;
        Eigen::VectorXd joint_velocities_;
        std::shared_ptr<whole_body_roller::Constraint> dynamics_constraint;

    public:
        Dynamics(int num_end_effectors, std::shared_ptr<pinocchio::Model> model);
        bool add_end_effector(std::string frame_name);
        bool change_end_effector_state(std::string frame_name, end_effector_state_t new_state);
        bool change_end_effector_function(std::string frame_name, end_effector_function_t new_function);
        bool update_joint_states(const Eigen::VectorXd &joint_positions, const Eigen::VectorXd &joint_velocities);


        // check the end effectors that are in contact and update nc_ in dec_v
        // then update the dynamics constraint with new M for dec_v.qdd and new contact jacobians for each contact in the constraint 
        // the order of contact jacobians is the same as the order that the end effectors have been added in
        //          the end effectors which are not in contact aren't considered to save computation time
        bool update_dynamics_constraint();

        std::shared_ptr<whole_body_roller::ControlDecisionVariables> get_dec_v();

        bool update_constraint() override;

    };
}