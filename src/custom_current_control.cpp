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
        pid_controllers.push_back(PID<float>(KP, KI, KD));
    }

    // Init motor_currents
    motor_currents = std::vector<float>(n_motors);
    pid_outputs = std::vector<float>(n_motors);

    // Init prev_time with current instant
    prev_time = ros::Time::now().toSec();
}

bool Current_PID::set_currents(std::vector<float> cmd_currents, float currents_time)
{
    // 1. Read motor_currents
    if(get_currents(motor_currents))
    {
        // Compute PID outputs
        for(i = 0; i < n_motors; i++)
        {
            // Variable Dt
            pid_controllers[i].compute(currents_time - prev_time, cmd_currents[i] - motor_currents[i], pid_outputs[i]);
        }

        // Update prev_time
        prev_time = currents_time;

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