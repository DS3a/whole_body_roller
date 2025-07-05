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

    bool Dynamics::update_joint_states(const Eigen::VectorXd &joint_positions, const Eigen::VectorXd &joint_velocities) {
        if (joint_positions.size() != this->model_->nq || joint_velocities.size() != this->model_->nv) {
            return false; // size mismatch
        }

        this->joint_positions_ = joint_positions;
        this->joint_velocities_ = joint_velocities;

        // Update the kinematics for the model
        // pinocchio::forwardKinematics(*this->model_, *this->data_, this->joint_positions_, this->joint_velocities_);
        
        // Update the dynamics constraint with the new joint states
        // this->dynamics_constraint->update_dynamics_constraint(this->joint_positions_, this->joint_velocities_);

        return true;
    }
        
    bool Dynamics::update_dynamics_constraint() {
        if (!this->is_dynamics_ready) {
            return false; // dynamics not ready
        }

        std::vector<Eigen::MatrixXd> contact_jacobians;

        bool update_success = true;

        pinocchio::forwardKinematics(*this->model_, *this->data_, this->joint_positions_, this->joint_velocities_);
        // Update the number of contact points in the decision variables
        this->dec_v->nc_ = 0;
        for (const auto& ee : this->end_effectors) {
            if (ee.state == whole_body_roller::end_effector_state_t::IN_CONTACT) {
                this->dec_v->nc_++;
                contact_jacobians.push_back(
                    // TODO need to figure out which frame to use for the contact jacobian
                    pinocchio::getFrameJacobian(*this->model_, 
                                                *this->data_, 
                                                this->model_->getFrameId(ee.frame), 
                                                pinocchio::LOCAL_WORLD_ALIGNED)
                );
            }
        }
        update_success &= this->dynamics_constraint->set_contact_constraints(contact_jacobians);

        // TODO need to figure out which convention to use see: https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio.html#ab48efbd527d1bc9941da1a5f400e751a

        pinocchio::crba(*this->model_, *this->data_, this->joint_positions_);
        update_success &= this->dynamics_constraint->set_qdd_constraints(this->data_->M);
        pinocchio::rnea(*this->model_, 
                        *this->data_, 
                        this->joint_positions_, 
                        this->joint_velocities_, 
                        Eigen::VectorXd::Zero(this->model_->nv));
        update_success &= this->dynamics_constraint->set_constraint_bias(this->data_->tau);
        
        return update_success;
    }

 
}