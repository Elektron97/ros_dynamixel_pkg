/********************************************************************
 * Re-implementation of a Current Control, using as actuation the   *
 * Extended Position Mode of the Dynamixel.                         *                                                   
 ********************************************************************/
#ifndef CUSTOM_CURRENT_CONTROL_H_
#define CUSTOM_CURRENT_CONTROL_H_

/* INCLUDES */
#include "ros_dynamixel_pkg/dynamixel_utils.h"
#include "pid/pid.h"

/* DEFINES */
#define KP 0.5      // Default 
#define KI 0.2      // Default
#define KD 0.01     // Default

/* INHERITING ExtPos_Dynamixel */
class Current_PID : public ExtPos_Dynamixel
{
    /* PRIVATE ATTRIBUTES */
    // Vectors
    std::vector<PID<float>> pid_controllers;
    std::vector<float> motor_currents;
    std::vector<float> pid_outputs;

    // Time variable
    float prev_time;

    /* PUBLIC ATTRIBUTES AND METHODS */
    public: 
        Current_PID(int n_dyna, float Kp, float Ki, float Kd);
        // Overload Constructor: import Default PID gains
        Current_PID(int n_dyna);
        bool set_currents(std::vector<float> cmd_currents, float currents_time);
        // Overload set_currents: it gives the actuators' internal currents
        bool set_currents(std::vector<float> cmd_currents, float currents_time, std::vector<float>& feedback_currents);

        /************************* Few Words about set_currents method: *********************************
         * set_currents uses the current feedback to command the motors.                                *
         * If the higher-level ROS node try to read the currents, there will be                         *
         * 2 reading of currents. This augments the probability of conflict in                          *
         * the accessing to the registers of Dynamixels.                                                *
         * To avoid it, set_currents overloaded method, store the feedback current                      *
         * in an internal variable, in such a way, it is possible to read the already extracted value   *
         * for debug purposes.                                                                          *
         ************************************************************************************************/
};

#endif /* CUSTOM_CURRENT_CONTROL_H_ */