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

#endif /* _PID_H_ */