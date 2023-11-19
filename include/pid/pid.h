/**
 * PID Implementation in C++.
 */

#ifndef _PID_H_
#define _PID_H_

class PID
{
    /* Private Attributes */
    double Kp, Ki, Kd;
    double prev_error;
    double integral;

    public:
    /* Constructor */
    PID(double Kp, double Ki, double Kd);

    /* Methods */
    bool compute(double dt, double current_error, double& output);
};

#endif /* _PID_H_ */