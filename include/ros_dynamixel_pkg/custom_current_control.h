/********************************************************************
 * Re-implementation of a Current Control, using as actuation the   *
 * Extended Position Mode of the Dynamixel.                         *                                                   
 ********************************************************************/
#ifndef CUSTOM_CURRENT_CONTROL_H_
#define CUSTOM_CURRENT_CONTROL_H_

/* INCLUDES */
#include "ros_dynamixel_pkg/dynamixel_utils.h"
#include "pid/pid.h"

/* INHERITING ExtPos_Dynamixel */
class Current_PID : public ExtPos_Dynamixel
{
    /* PRIVATE ATTRIBUTES */
    std::vector<PID> pid_controllers;

    /* PUBLIC ATTRIBUTES AND METHODS */
    public: 
        Current_PID(int n_dyna);
};

#endif /* CUSTOM_CURRENT_CONTROL_H_ */