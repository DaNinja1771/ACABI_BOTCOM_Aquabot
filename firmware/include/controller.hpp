#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "BOTCOM_parameters.hpp"
#include "util_functions.hpp"

enum class MovementCommand {
    STOP = 0,
    FORWARD,
    FORWARD_VARIABLE,
    LEFT,
    RIGHT,
    COMPASS_CALIBRATE,
    COMPASS_CALIBRATE_MAX
};// End ControllerCommand Enum Class

class Controller {

    public:
        /// @brief Default Constructor
        Controller();
        /// @brief Sets up object. Sets motors to stop 
        void setup();
        /// @brief Checks if planner state is started. 
        /// If NO motors will be set to stop. 
        /// If YES motos will be set in either forward or a turning state 
        /// depending on how far off the drone heading is from the target (heading error)
        /// @param planner_data This is a reference to the planner_data variable in the main function
        /// @param estimator_data This is a reference to the estimator_data in the main function
        void iteration(PlannerData &planner_data, EstimatorData &estimator_data);
        /// @brief Sets correct signal input to the motors by providing the 
        /// PWM signal via analogwrite (found in Arduino.h)
        /// @param command 
        void controller_command(MovementCommand command);
        
    private:
        void motor_set(int right_a1, int right_a2, int left_a1, int left_a2);
        uint8_t right_pwm;
        uint8_t left_pwm;
};// End Controller Class

#endif // CONTROLLER_HPP
