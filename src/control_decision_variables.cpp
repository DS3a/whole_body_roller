#include "control_decision_variables.hpp"

namespace whole_body_roller {
    ControlDecisionVariables::ControlDecisionVariables(int nv, int nc) {
        this->nv_ = nv;
        this->nc_ = nc;
        this->ntau_ = nv - 6; // nv - 6 for floating base motion which cannot be controlled
        // this->qdd = std::make_shared<Eigen::VectorXd>(nv);
        // this->tau = std::make_shared<Eigen::VectorXd>(nv-6); 
        // // nv - 6 for floating base motion which cannot be controlled
        // lambda_c is a vector of nc contact forces, each of size 6
        // this->lambda_c = 
        // std::make_shared<std::vector<Eigen::VectorXd>>(nc, Eigen::VectorXd(6));
        // slack variables can also be added here when we get to tasks    
    }
   
} // namespace whole_body_roller