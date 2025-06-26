#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <memory>

namespace whole_body_roller {
    class ControlDecisionVariables {
    public:
        int nv_; // number of variables in qdd
        int nc_; // number of contact points
        std::shared_ptr<Eigen::VectorXd> qdd;
        std::shared_ptr<Eigen::VectorXd> tau;
        std::shared_ptr<std::vector<Eigen::VectorXd>> lambda_c;
    public:
        ControlDecisionVariables(int nv, int nc) {
            this->nv_ = nv;
            this->nc_ = nc;
            this->qdd = std::make_shared<Eigen::VectorXd>(nv);
            this->tau = std::make_shared<Eigen::VectorXd>(nv-6); 
            // nv - 6 for floating base motion which cannot be controlled

            // lambda_c is a vector of nc contact forces, each of size 6
            this->lambda_c = 
            std::make_shared<std::vector<Eigen::VectorXd>>(nc, Eigen::VectorXd(6));}
    };
} // namespace whole_body_roller