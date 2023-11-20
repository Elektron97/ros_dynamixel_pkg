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
#define KP 0.5
#define KI 0.2
#define KD 0.01

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
        Current_PID(int n_dyna);
        bool set_currents(std::vector<float> cmd_currents, float currents_time);
};

#endif /* CUSTOM_CURRENT_CONTROL_H_ */