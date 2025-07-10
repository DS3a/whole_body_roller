#include "constraint.hpp"
#include "control_decision_variables.hpp"
#include "task_space_constraints/frame_acceleration.hpp"


#include <iostream>

int main(int argc, const char **argv) {
    std::shared_ptr<whole_body_roller::ControlDecisionVariables> cdv; // Example initialization with 18 variables and 4 contact points

    cdv = std::make_shared<whole_body_roller::ControlDecisionVariables>(7, 2);

    std::cout << "Control Decision Variables initialized with nqdd: " << cdv->qdd->size()
                << " and tau: " << cdv->tau->size()
              << " and nc: " << cdv->lambda_c->size()
              << "\nwith each lambda of: " << (*cdv->lambda_c)[0].size()
               << std::endl;

    whole_body_roller::Constraint c(cdv, 2, whole_body_roller::constraint_type_t::INEQUALITY);
    c.set_qdd_constraints(Eigen::MatrixXd::Ones(2, 7));



    std::cout << c.get_constraint_matrix() << std::endl;
    std::cout << "checking if constraint is valid" << c.is_constraint_valid() << std::endl;

    return 0;
}