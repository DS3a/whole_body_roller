#include "constraint.hpp"
#include "dynamics.hpp"

namespace whole_body_roller {
    class FrameAccelerationConstraint : public whole_body_roller::ConstraintHandler {
    public:
        std::shared_ptr<whole_body_roller::Dynamics> dynamics;
        std::shared_ptr<whole_body_roller::Constraint> constraint;
        std::string frame_name_;
        Eigen::VectorXd acceleration_target;

    public:
        FrameAccelerationConstraint(std::shared_ptr<whole_body_roller::Dynamics> dyn, 
                                    std::string frame_name) 
            : dynamics(dyn), frame_name_(frame_name) {
            
            this->acceleration_target = Eigen::VectorXd::Zero(6); // Initialize target acceleration to zero
            this->constraint = std::make_shared<whole_body_roller::Constraint>(
                this->dynamics->dec_v,
                6, // constraints for se3 acceleration
                whole_body_roller::constraint_type_t::EQUALITY // equality constraint for acceleration
            );
            this->constraint->set_tau_constraints(
                Eigen::MatrixXd::Zero(6, this->dynamics->dec_v->nv_ - 6) // No tau constraints for acceleration
            );
        }

        bool set_acceleration_target(const Eigen::VectorXd &acceleration) {
            if (acceleration.size() != 6) {
                return false; // size mismatch
            }
            this->acceleration_target = acceleration;
            return true;
        }

        bool update_constraint() override {
            if (!this->dynamics->is_dynamics_ready || !this->dynamics->model_->existFrame(this->frame_name_)) {
                return false; // dynamics not ready or frame does not exist
            }

            bool update_success = true;

            pinocchio::computeJointJacobians(*this->dynamics->model_, 
                                            *this->dynamics->data_, 
                                            this->dynamics->joint_positions_);
            // Get the Jacobian of the frame
            Eigen::MatrixXd jacobian = pinocchio::getFrameJacobian(*this->dynamics->model_, 
                                                                   *this->dynamics->data_, 
                                                                   this->dynamics->model_->getFrameId(this->frame_name_), 
                                                                   pinocchio::LOCAL_WORLD_ALIGNED);
                                                                   // see dynamics.cpp#111 for explanation as to why we use LOCAL_WORLD_ALIGNED


            // Set the constraints for the acceleration
            update_success &= this->constraint->set_qdd_constraints(jacobian.transpose());
            double dt = 1e-8; // Small time step for numerical stability

            Eigen::VectorXd q_fut = Eigen::VectorXd::Zero(this->dynamics->model_->nq);
            q_fut = pinocchio::integrate(*this->dynamics->model_, 
                                         this->dynamics->joint_positions_, 
                                         this->dynamics->joint_velocities_ * dt);
                                         

            pinocchio::computeJointJacobians(*this->dynamics->model_, 
                                            *this->dynamics->data_, 
                                            q_fut);

            Eigen::MatrixXd jacobian_fut = pinocchio::getFrameJacobian(*this->dynamics->model_, 
                                                                   *this->dynamics->data_, 
                                                                   this->dynamics->model_->getFrameId(this->frame_name_), 
                                                                   pinocchio::LOCAL_WORLD_ALIGNED);

            Eigen::MatrixXd dJ = (jacobian_fut - jacobian) / dt; // Numerical derivative of the Jacobian
            update_success &= this->constraint->set_constraint_bias(this->acceleration_target - dJ * this->dynamics->joint_velocities_); // Bias is the negative of the target acceleration
            
            // the selection matrix is set to all zeros in the constructor,
            // the contact force constraints are undefined. they all need to be zero, 
            // but they need to be defined again here since we don't know 
            // if the num contacts have remained the same since the last time this function was called
            this->constraint->ignore_contact_constraints();
            // this needs to be called everytime as it refreshes the thingi based on the number of contact points
            
            return update_success;
        } 
    };
}