#include "controller.hpp"

Controller::Controller() {

}

void Controller::setup() {

    controller_command(MovementCommand::STOP);
}

void Controller::iteration(PlannerData &planner_data, EstimatorData &estimator_data) {

    // Check the state of planner_data
    if (planner_data.state != MissionState::Started) {

        // Stop the motors if not in 'Started' state
        controller_command(MovementCommand::STOP); 
        
        return;
    }

    // if right of zero go left
    // negative heading error = right of zero for heading error
    // positive heading error = left of zero for heading error
    
    if(estimator_data.compass_data.heading_error < BOUNDED_DEGREES && estimator_data.compass_data.heading_error > -BOUNDED_DEGREES){

        left_pwm = ((MOTOR_PWM_MAX - MOTOR_PWM_BASE) / (double)BOUNDED_DEGREES) * (estimator_data.compass_data.heading_error) + MOTOR_PWM_BASE;
        right_pwm = -((MOTOR_PWM_MAX - MOTOR_PWM_BASE) / (double)BOUNDED_DEGREES) * (estimator_data.compass_data.heading_error) + MOTOR_PWM_BASE;
        if (estimator_data.compass_data.heading_error > HEADING_TOLERANCE){
            left_pwm += MOTOR_PWM_BASE_CHANGE;   
        }
        else if (estimator_data.compass_data.heading_error < -HEADING_TOLERANCE){
            right_pwm += MOTOR_PWM_BASE_CHANGE;
        }
          
        controller_command(MovementCommand::FORWARD_VARIABLE);
    }
    else if (estimator_data.compass_data.heading_error >= BOUNDED_DEGREES){
        controller_command(MovementCommand::RIGHT);
    }
    else if(estimator_data.compass_data.heading_error <= -BOUNDED_DEGREES){
        controller_command(MovementCommand::LEFT);
    }
    else{
        controller_command(MovementCommand::FORWARD);
    }

}

void Controller::controller_command(MovementCommand command) {

    switch(command) {
        // Specials ///////////////////////////////////////////////////////////////////////////////////////////////////
        case MovementCommand::STOP: 
            motor_set(  MOTOR_PWM_OFF, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_OFF
                        ); 
                        debug_logln("s");
                        // delay(10);
            break;
        case MovementCommand::FORWARD: 
            motor_set(  MOTOR_PWM_OFF, 
                        MOTOR_PWM_BASE, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_BASE
                        ); 
                debug_logln("f");
            break;
        case MovementCommand::COMPASS_CALIBRATE: 
            motor_set(  MOTOR_PWM_CAL, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_CAL
                        ); 
            break;

        case MovementCommand::COMPASS_CALIBRATE_MAX: 
            motor_set(  MOTOR_PWM_MAX, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_MAX
                        ); 
            break;

        case MovementCommand::FORWARD_VARIABLE:
            motor_set(  MOTOR_PWM_OFF, 
                        right_pwm, 
                        MOTOR_PWM_OFF, 
                        left_pwm
                        );
            break;

        // Rights ///////////////////////////////////////////////////////////////////////////////////////////////////
        case MovementCommand::RIGHT: 
            motor_set(  MOTOR_TURN_PWM, 
                        MOTOR_PWM_OFF, 
                        MOTOR_PWM_OFF, 
                        MOTOR_TURN_PWM
                        ); 
            break;
        
        // Lefts ///////////////////////////////////////////////////////////////////////////////////////////////////
        case MovementCommand::LEFT: 
            motor_set(  MOTOR_PWM_OFF, 
                        MOTOR_TURN_PWM, 
                        MOTOR_TURN_PWM, 
                        MOTOR_PWM_OFF
                        ); 
            break;
       
        default:
            debug_logln("There is blood dripping from my nose that is dripping.....");
            // thats bad 
            break;
    }
}

void Controller::motor_set(int right_a1, int right_a2, int left_a1, int left_a2) {

    analogWrite(MOTOR_0_PIN_RIGHT_A1,   right_a1);
    analogWrite(MOTOR_1_PIN_RIGHT_A2,   right_a2);
    analogWrite(MOTOR_2_PIN_LEFT_A1,    left_a1);
    analogWrite(MOTOR_3_PIN_LEFT_A2,    left_a2);

}
