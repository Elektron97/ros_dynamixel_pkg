/**
 * PID Implementation in C++.
 */

#ifndef _PID_H_
#define _PID_H_

#include <iostream>
#include <cmath>

template <typename T>
class PID
{
    /* Private Attributes */
    T Kp, Ki, Kd;
    T prev_error;
    T integral;

    public:
    /* Constructor */
    // PID();  // Kp = 0, Ki = 0, Kd = 0
    PID(T Kp, T Ki, T Kd);

    /* Methods */
    bool compute(T dt, T current_error, T& output);
};

/****************************************************************
 * The implementation of the methods inside the header file     *
 * is necessary in the case of a class with the template.       *
 * Unfortunately, I was forced to coding in this way.           *
 * I'm sorry.                                                   *
 ****************************************************************/

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

#endif /* _PID_H_ */