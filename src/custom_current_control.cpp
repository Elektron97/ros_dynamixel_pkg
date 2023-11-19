/****************************************
 * Source code for Dynamixel Library    *
 ****************************************/
// Include
#include "ros_dynamixel_pkg/custom_current_control.h"

// Source code of methods and constructors
Current_PID::Current_PID(int n_dyna) : ExtPos_Dynamixel(n_dyna)
{
    /* After ExtPos_Dynamixel Constructor... */
    // Init PIDs
    for(i = 0; i < n_motors; i++)
    {
        // TO DO: Insert Tuned Gains
        pid_controllers.push_back(PID<float>(1.0, 1.0, 1.0));
    }

    // Init motor_currents
    motor_currents = std::vector<float>(n_motors);
    pid_outputs = std::vector<float>(n_motors);
}

bool Current_PID::set_currents(std::vector<float> cmd_currents)
{
    // 1. Read motor_currents
    if(get_currents(motor_currents))
    {
        // Compute PID outputs
        for(i = 0; i < n_motors; i++)
        {
            // Hardcoded 100 Hz for now
            pid_controllers[i].compute(0.01, cmd_currents[i] - motor_currents[i], pid_outputs[i]);
        }

        // Send to set_turns method of ExtPos_Dynamixel Super Class
        if(!set_turns(pid_outputs))
            return false;
        else
            return true;
    }
    else
    {
        ROS_ERROR("[PID Error]: Current Feedback not available.");
        return false;
    }
}