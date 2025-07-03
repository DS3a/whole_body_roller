#include "dynamics.hpp"

namespace whole_body_roller {
    Dynamics::Dynamics(int num_end_effectors, std::shared_ptr<pinocchio::Model> model) {
        this->num_end_effectors_ = num_end_effectors;
        this->model_ = model;
        this->data_ = std::make_shared<pinocchio::Data>(*this->model_);

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

    // bool Dynamics::update_joint_states(const Eigen::VectorXd &joint_positions, const Eigen::VectorXd &joint_velocities);
        
    // bool Dynamics::update_dynamics_constraint();

 
}