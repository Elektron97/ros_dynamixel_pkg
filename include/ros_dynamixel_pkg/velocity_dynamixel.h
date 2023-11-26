#ifndef VELOCITY_DYNAMIXEL_H_
#define VELOCITY_DYNAMIXEL_H_

#include "ros_dynamixel_pkg/dynamixel_motors.h"

// Velocity Dynamixel with Current Feedback
class Velocity_Dynamixel: public Dynamixel_Motors<int32_t>
{
    // Init with POSITION address
    GroupSyncWrite  motors_syncWrite    = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, VELOCITY_BYTE);
    GroupSyncRead   position_syncRead   = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, POSITION_BYTE);
    GroupSyncRead   current_syncRead    = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, CURRENT_BYTE);

    // Calibrated Positions
    std::vector<int32_t> calibrated_positions;

    public:
        // --- Constructor --- //
       Velocity_Dynamixel(int n_dyna);

        // --- Methods --- //
        // Low Level Set: Register
        bool set2registers(int32_t registers[]);
        bool set2registers(std::vector<int32_t> registers);         // Overwrite (vector<T>)

        // Low Level Get: Register
        bool get_CurRegisters(std::vector<int16_t>& currents);
        bool get_PosRegisters(std::vector<int32_t>& positions);

        // Mid Level Get: Current
        bool get_currents(std::vector<float>& currents);

        // High Level Set: Velocities
        bool set_velocities(std::vector<float> velocities);

        // Power Off Functions
        bool set2Zeros();
};

#endif /* VELOCITY_DYNAMIXEL_H_ */