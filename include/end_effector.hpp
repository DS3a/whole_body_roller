#pragma once
#include <string>


namespace whole_body_roller {
    enum end_effector_state_t {
        IN_CONTACT,
        FLOATING,
    };

    enum end_effector_function_t {
        MANIPULATION,
        LOCOMOTION,
        IDLE,
    };

    class EndEffector {
    public:
        std::string frame;
        end_effector_state_t state;
        end_effector_function_t fn;

    };
}