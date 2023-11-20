/**
 * PID Implementation in C++.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include "pid/pid.h"

using namespace std;

template <typename T> PID<T>::PID(T Kp, T Ki, T Kd)
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

template <typename T> bool PID<T>::compute(T dt, T current_error, T& output)
{
    // Security Check on dt
    if(dt == 0)
        return false;
    else
    {
        // Update integral contribute
        integral += current_error*dt;

        // Compute Control Law
        output = Kp*current_error + Ki*integral + (current_error - prev_error)/dt;
        return true;
    }
}
#endif