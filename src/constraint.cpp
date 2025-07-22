#include "constraint.hpp"

namespace whole_body_roller {
    Constraint::Constraint(std::shared_ptr<whole_body_roller::ControlDecisionVariables> dv, int num_constraints, constraint_type_t constraint_type) {
        this->dec_v = dv;
        this->num_constraints_ = num_constraints;
        this->constraint_type_ = constraint_type;
        this->qdd_constraints = Eigen::MatrixXd::Zero(this->num_constraints_, this->dec_v->nv_);
        this->tau_constraints = Eigen::MatrixXd::Zero(this->num_constraints_, this->dec_v->nv_-6);
        this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Zero(this->num_constraints_, 6));
        
        
        // this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Ones(this->num_constraints_, 6));
        // this is just to test whether the contact constraints get concatenated properly
    }


    bool Constraint::set_qdd_constraints(Eigen::MatrixXd constraints) {
        if (constraints.rows() == this->num_constraints_ && constraints.cols() == this->dec_v->nv_) {
            this->qdd_constraints = constraints;
            return true;
        }
        return false;
    }


    bool Constraint::set_tau_constraints(Eigen::MatrixXd constraints) {
        if (constraints.rows() == this->num_constraints_ && constraints.cols() == this->dec_v->nv_-6) {
            this->tau_constraints = constraints;
            return true;
        }
        return false;
    }

    void Constraint::ignore_contact_constraints(bool ignore_contacts=true) {
        this->contacts_are_considered = !ignore_contacts;
        // This function is to ignore the contact constraints, 
        // which means that the contact constraints will not be considered in the constraint matrix
        // This is useful for testing purposes or when we don't want to consider contact constraints
        // this->contact_constraints = std::vector<Eigen::MatrixXd>(this->dec_v->nc_, Eigen::MatrixXd::Zero(this->num_constraints_, 6));
    }

    bool Constraint::set_contact_constraints(std::vector<Eigen::MatrixXd> constraints) {
        if (constraints.size() == this->dec_v->nc_) {
            for (size_t i = 0; i < this->dec_v->nc_; ++i) {
                if (constraints[i].rows() != this->num_constraints_ || constraints[i].cols() != 6) {
                    return false; // Invalid size for one of the contact constraints
                }
                this->contact_constraints[i] = constraints[i];
            }
            return true;
        }
        return false;
    }

    bool Constraint::set_constraint_bias(Eigen::VectorXd bias) {
        if (bias.size() == this->num_constraints_) {
            this->constraint_bias = bias;
            return true;
        }
        return false;
    }


    bool Constraint::is_constraint_valid() {
        if (
            this->dec_v->nv_ > 6 && // at least 6 variables for floating base
            this->dec_v->nc_ >= 0 && // at least 0 contact points
            this->num_constraints_ > 0 && // at least one constraint
            this->qdd_constraints.rows() == this->num_constraints_ &&
            this->qdd_constraints.cols() == this->dec_v->nv_ &&
            this->tau_constraints.rows() == this->num_constraints_ &&
            this->tau_constraints.cols() == this->dec_v->nv_-6
        ) {
            // check contact constraints
            if (this->contacts_are_considered && 
                this->contact_constraints.size() == this->dec_v->nc_ &&
                std::all_of(this->contact_constraints.begin(), this->contact_constraints.end(), 
                    [this](const Eigen::MatrixXd& m) { return m.rows() == this->num_constraints_ && m.cols() == 6; })
                ) {
                    return true;
                } else {
                    return false;
                }
            return true;
        }

        return false;
    }

    Eigen::MatrixXd Constraint::get_constraint_matrix() {
        Eigen::MatrixXd constraint_matrix(this->num_constraints_, 2*(this->dec_v->nv_)-6+6*(this->dec_v->nc_));
        
        Eigen::MatrixXd ct_constraints(this->num_constraints_, 6*(this->dec_v->nc_));
        if (this->contacts_are_considered) {
            int current_col = 0;
            for (const auto& m : this->contact_constraints) {
                    // this is to test whether the concatenation is fine
                    // ct_constraints.block(0, currentCol, m.rows(), m.cols()) = m*(currentCol/12.0+1);
                    // this adds the contact constraints from the vector to ct_constraints
                ct_constraints.block(0, current_col, m.rows(), m.cols()) = m;
                current_col += m.cols();
            }
        } else {
            ct_constraints = Eigen::MatrixXd::Zero(this->num_constraints_, 6*(this->dec_v->nc_));
            // if the contacts are not considered, then the contact constraints are all zeros
            // this is to save computation time
        }
        constraint_matrix << this->qdd_constraints, this->tau_constraints, ct_constraints;

        return constraint_matrix;
    }
} 