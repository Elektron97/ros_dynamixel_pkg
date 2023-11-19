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
        pid_controllers.push_back(PID(1.0, 1.0, 1.0));
    }
}