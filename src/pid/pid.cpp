/**
 * PID Implementation in C++.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid/pid.h"

using namespace std;

PID::PID(double Kp, double Ki, double Kd)
{
    // Assign Gains to PID obj
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    // Init error & cumulative error
    prev_error = 0.0;
    integral = 0.0;
}

// Empty Arguments: Constructor Delegation
// PID::PID() : PID::PID(0.0, 0.0, 0.0) {}

bool PID::compute(double dt, double current_error, double& output)
{
    // Update integral contribute
    integral += current_error*dt;

    if(dt == 0)
        return false;
    else
    {
        output = Kp*current_error + Ki*integral + (current_error - prev_error)/dt;
        return true;
    }
}
#endif