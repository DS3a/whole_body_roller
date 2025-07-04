#include "dynamics.hpp"

namespace whole_body_roller {
    Dynamics::Dynamics(int num_end_effectors, std::shared_ptr<pinocchio::Model> model) {
        this->num_end_effectors_ = num_end_effectors;
        this->model_ = model;
        this->data_ = std::make_shared<pinocchio::Data>(*this->model_);

        this->dynamics_constraint = std::make_shared<whole_body_roller::Constraint>(
            this->dec_v,
            this->model_->nv, // number of variables in the second derivative of the joint positions (incl. floating base)
            whole_body_roller::constraint_type_t::EQUALITY // the dynamics constraint is an equality constraint
        );


        Eigen::MatrixXd selection_matrix = Eigen::MatrixXd::Zero(this->model_->nv, this->model_->nv - 6);
        Eigen::MatrixXd selection_matrix_floating_base = Eigen::MatrixXd::Zero(6, this->model_->nv - 6);
        Eigen::MatrixXd selection_matrix_joints = Eigen::MatrixXd::Identity(this->model_->nv - 6, this->model_->nv - 6);
        selection_matrix << selection_matrix_floating_base, 
                            selection_matrix_joints;
        this->dynamics_constraint->set_tau_constraints(
            selection_matrix
        );

        // all end effectors need to be added
        this->added_end_effectors = 0;
        this->is_dynamics_ready = false;
        this->free_end_effectors = num_end_effectors;


    }


    bool Dynamics::add_end_effector(std::string frame_name) {
        if (this->added_end_effectors < this->num_end_effectors_ &&
            this->model_->existFrame(frame_name)
        ) {
            whole_body_roller::EndEffector ee;
            ee.frame = frame_name;
            ee.state = whole_body_roller::end_effector_state_t::FLOATING; // default state
            ee.fn = whole_body_roller::end_effector_function_t::IDLE; // default function
            this->end_effectors.push_back(ee);
            this->end_effector_map_[frame_name] = this->added_end_effectors;
            this->added_end_effectors++;
            
            if (this->added_end_effectors == this->num_end_effectors_) {
                this->is_dynamics_ready = true;
            }
            return true;
        }
        return false;
    }

    bool Dynamics::change_end_effector_state(std::string frame_name, end_effector_state_t new_state) {
        if (this->is_dynamics_ready &&
            this->model_->existFrame(frame_name) &&
            this->end_effector_map_.count(frame_name) == 1
            ) {
            this->end_effectors[this->end_effector_map_[frame_name]].state = new_state;    
            return true;
        }

        return false;
    }

    bool Dynamics::change_end_effector_function(std::string frame_name, end_effector_function_t new_function) {
        if (this->is_dynamics_ready &&
            this->model_->existFrame(frame_name) &&
            this->end_effector_map_.count(frame_name) == 1
            ) {
            this->end_effectors[this->end_effector_map_[frame_name]].fn = new_function;    
            return true;
        }

        return false;
    }
    
    // bool Dynamics::update_joint_states(const Eigen::VectorXd &joint_positions, const Eigen::VectorXd &joint_velocities);
        
    // bool Dynamics::update_dynamics_constraint();

 
}