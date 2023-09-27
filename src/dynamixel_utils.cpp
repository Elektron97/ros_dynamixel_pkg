/****************************************
 * Source code for Dynamixel Library    *
 ****************************************/
// Include
#include "ros_dynamixel_pkg/dynamixel_utils.h"

// --- (Abstract) Dynamixel Class --- //
// Constructor 
template <typename T> Dynamixel_Motors<T>::Dynamixel_Motors()
{
    // Open Communication

    // Init of these variables in the declaration
    //uint8_t dxl_error = 0;
    //int dxl_comm_result = COMM_TX_FAIL;

    if(!portHandler->openPort()) 
        ROS_ERROR("Failed to open the port!");

    if(!portHandler->setBaudRate(BAUDRATE))
        ROS_ERROR("Failed to set the baudrate!");
}

// Deconstructor
template <typename T> Dynamixel_Motors<T>::~Dynamixel_Motors()
{
    ROS_WARN("Terminating Dynamixel object...");
    
    // Set Current to Zero in every motors
    if(!set2Zeros())
        ROS_ERROR("Failed to set all the torques to zero.");
    
    // Turn Off every motors
    powerOFF();
    
    ROS_WARN("Dynamixel Object terminated.");
}

template <typename T> void Dynamixel_Motors<T>::powerOFF()
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;

    // Turn Off LED and Disable Torque
    i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_OFF, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn off LED for Dynamixel ID %d", i+1);
            //break;
        }

        // Disable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to disable torque for Dynamixel ID %d", i+1);
            //break;
        }
    }
}
/**************************************************************************/
// --- Current Dynamixel Class --- //
Current_Dynamixel::Current_Dynamixel(int n_dyna)
{
    // Init n_motors
    n_motors = n_dyna;

    // Error Handling
    // Init done by Super Class Constructor
    //dxl_error = 0;
    //dxl_comm_result = COMM_TX_FAIL;

    // Turn On LED, Current Mode and Enable Torque
    i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn on LED for Dynamixel ID %d", i+1);
            break;
        }

        // Current Drive Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_OP_MODE, CURRENT_MODE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set Current Mode for Dynamixel ID %d", i+1);
            break;
        }

        // Enable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Set Current to Zero in every motors
    if(!set2Zeros())
        ROS_ERROR("Failed to set all the torques to zero.");
}

bool Current_Dynamixel::set2Zeros()
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_currents[i][0] = DXL_LOBYTE(0);
        param_goal_currents[i][1] = DXL_HIBYTE(0);

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_currents[i]); // da sistemare
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, 0); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Current_Dynamixel::set2registers(int16_t registers[])
{
    // Assert: check that dim(currents) = n_motors
    //assert(sizeof(currents)/sizeof(int16_t) == n_motors);

    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        if(!registerCur_saturation(registers[i]))
            ROS_WARN("Commanded Current are out of limits. Saturating...");


        param_goal_currents[i][0] = DXL_LOBYTE(registers[i]);
        param_goal_currents[i][1] = DXL_HIBYTE(registers[i]);

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_currents[i]); // da sistemare
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Current_Dynamixel::set2registers(std::vector<int16_t> registers)
{
    // Assert: check that dim(currents) = n_motors
    //assert(sizeof(currents)/sizeof(int16_t) == n_motors);

    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        if(!registerCur_saturation(registers[i]))
            ROS_WARN("Commanded Current are out of limits. Saturating...");

        param_goal_currents[i][0] = DXL_LOBYTE(registers[i]);
        param_goal_currents[i][1] = DXL_HIBYTE(registers[i]);

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_currents[i]); // da sistemare
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Current_Dynamixel::get_CurRegisters(std::vector<int16_t>& currents)
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add all motors' id
    i = 1;
    for(i; i <= n_motors; i++)
    {
        // Supposing that Motors ID are 1, 2, 3, 4, ..., n_motors
        dxl_addparam_result = current_syncRead.addParam((uint8_t) i);
        if (dxl_addparam_result != true) 
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", i);
            break;
        }
    }

    // Read all motors
    dxl_comm_result = current_syncRead.txRxPacket();
    if(dxl_comm_result == COMM_SUCCESS)
    {
        for(i = 1; i <= n_motors; i++)
        {
            currents.push_back(current_syncRead.getData((uint8_t) i, ADDR_PRESENT_CURRENT, CURRENT_BYTE));
        }

        current_syncRead.clearParam();
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get currents! Result: %d", dxl_comm_result);
        current_syncRead.clearParam();
        return false;       
    }
}

bool Current_Dynamixel::set_currents(float currents[])
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = current2Register(currents[i]);
    }

    return set2registers(registers);
}

// Overloading
bool Current_Dynamixel::set_currents(std::vector<float> currents)
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = current2Register(currents[i]);
    }

    return set2registers(registers);
}

bool Current_Dynamixel::set_torques(float torques[])
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = torque2Register(torques[i]);
    }

    return set2registers(registers);
}

bool Current_Dynamixel::get_currents(std::vector<float>& currents)
{
    // Local Variable useful to convert
    std::vector<int16_t> current_registers;
    
    if(get_CurRegisters(current_registers))
    {
        // Convert
        for(i = 0; i < n_motors; i++)
        {
            currents[i] = register2Current(current_registers[i]); 
        }

        return true;
    }
    else
    {
        // Stop and get false
        return false;
    }
}

// Overloading
bool Current_Dynamixel::set_torques(std::vector<float> torques)
{
    int16_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = torque2Register(torques[i]);
    }

    return set2registers(registers);
}

/**************************************************************************/
// --- Extended Position Dynamixel Class --- //
ExtPos_Dynamixel::ExtPos_Dynamixel(int n_dyna)
{
    // Init n_motors
    n_motors = n_dyna;

    // Error Handling
    // Init done by Super Class Constructor
    //dxl_error = 0;
    //dxl_comm_result = COMM_TX_FAIL;

    // Turn On LED | Current Mode | Enable Torque | Profile velocity | Profile Acceleration
    i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn on LED for Dynamixel ID %d", i+1);
            break;
        }

        // Current Drive Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_OP_MODE, EXTENDED_POSITION_MODE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set Current Mode for Dynamixel ID %d", i+1);
            break;
        }

        // Enable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
            break;
        }

        // Profile Velocity
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, ADDR_PROFILE_VEL, PROFILE_VEL_VALUE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set profile velocity for Dynamixel ID %d", i+1);
            break;
        }
        
        // Profile Acceleration
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i + 1, ADDR_PROFILE_ACC, PROFILE_ACC_VALUE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set profile acceleration for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Read Initial Position
    if(!get_PosRegisters(initial_positions))
        ROS_ERROR("Failed to read all initial positions of the motors.");
}

bool ExtPos_Dynamixel::get_PosRegisters(std::vector<int32_t>& positions)
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add all motors' id
    i = 1;
    for(i; i <= n_motors; i++)
    {
        // Supposing that Motors ID are 1, 2, 3, 4, ..., n_motors
        dxl_addparam_result = position_syncRead.addParam((uint8_t) i);
        if (dxl_addparam_result != true) 
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", i);
            break;
        }
    }

    // Read all motors
    dxl_comm_result = position_syncRead.txRxPacket();
    if(dxl_comm_result == COMM_SUCCESS)
    {
        for(i = 1; i <= n_motors; i++)
        {
            positions.push_back(position_syncRead.getData((uint8_t) i, ADDR_PRESENT_POSITION, POSITION_BYTE));
        }

        position_syncRead.clearParam();
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get positions! Result: %d", dxl_comm_result);
        position_syncRead.clearParam();
        return false;       
    }
}

bool ExtPos_Dynamixel::get_CurRegisters(std::vector<int16_t>& currents)
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add all motors' id
    i = 1;
    for(i; i <= n_motors; i++)
    {
        // Supposing that Motors ID are 1, 2, 3, 4, ..., n_motors
        dxl_addparam_result = current_syncRead.addParam((uint8_t) i);
        if (dxl_addparam_result != true) 
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", i);
            break;
        }
    }

    // Read all motors
    dxl_comm_result = current_syncRead.txRxPacket();
    if(dxl_comm_result == COMM_SUCCESS)
    {
        for(i = 1; i <= n_motors; i++)
        {
            currents.push_back(current_syncRead.getData((uint8_t) i, ADDR_PRESENT_CURRENT, CURRENT_BYTE));
        }

        current_syncRead.clearParam();
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get currents! Result: %d", dxl_comm_result);
        current_syncRead.clearParam();
        return false;       
    }
}

bool ExtPos_Dynamixel::get_currents(std::vector<float>& currents)
{
    // Local Variable useful to convert
    std::vector<int16_t> current_registers;
    
    if(get_CurRegisters(current_registers))
    {
        // Convert
        for(i = 0; i < n_motors; i++)
        {
            currents[i] = register2Current(current_registers[i]); 
        }

        return true;
    }
    else
    {
        // Stop and get false
        return false;
    }
}

bool ExtPos_Dynamixel::set2registers(int32_t registers[])
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_positions[n_motors][POSITION_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        //if(!registerTurns_saturation(registers[i]))
        //    ROS_WARN("Commanded Current are out of limits. Saturating...");

        param_goal_positions[i][0] = DXL_LOBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][1] = DXL_HIBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][2] = DXL_LOBYTE(DXL_HIWORD(registers[i]));
        param_goal_positions[i][3] = DXL_HIBYTE(DXL_HIWORD(registers[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_positions[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setPosition : [ID:%d] [POSITION (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool ExtPos_Dynamixel::set2registers(std::vector<int32_t> registers)
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_positions[n_motors][POSITION_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        //if(!registerTurns_saturation(registers[i]))
        //    ROS_WARN("Commanded Current are out of limits. Saturating...");

        param_goal_positions[i][0] = DXL_LOBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][1] = DXL_HIBYTE(DXL_LOWORD(registers[i]));
        param_goal_positions[i][2] = DXL_LOBYTE(DXL_HIWORD(registers[i]));
        param_goal_positions[i][3] = DXL_HIBYTE(DXL_HIWORD(registers[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_positions[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setPosition : [ID:%d] [POSITION (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool ExtPos_Dynamixel::set_turns(float turns[])
{
    // Convert in position register value
    int32_t registers[n_motors];

    // Start Conversion
    i = 0;
    for(i; i < n_motors; i++)
    {
        // Security Saturation on turns values
        if(!turns_saturation(turns[i]))
            ROS_WARN("Commanded Turns are out of limits. Saturating...");

        registers[i] = ((int32_t) (turns[i]*((float) ONE_TURN_REGISTER))) + initial_positions[i];
    }

    return set2registers(registers);
}

bool ExtPos_Dynamixel::set_turns(std::vector<float> turns)
{
    // Convert in position register value
    int32_t registers[n_motors];

    // Start Conversion
    i = 0;
    for(i; i < n_motors; i++)
    {
        // Security Saturation on turns values
        if(!turns_saturation(turns[i]))
            ROS_WARN("Commanded Turns are out of limits. Saturating...");

        registers[i] = ((int32_t) (turns[i]*((float) ONE_TURN_REGISTER))) + initial_positions[i];
    }

    return set2registers(registers);
}

bool ExtPos_Dynamixel::set2Zeros()
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_positions[n_motors][POSITION_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_positions[i][0] = DXL_LOBYTE(DXL_LOWORD(initial_positions[i]));
        param_goal_positions[i][1] = DXL_HIBYTE(DXL_LOWORD(initial_positions[i]));
        param_goal_positions[i][2] = DXL_LOBYTE(DXL_HIWORD(initial_positions[i]));
        param_goal_positions[i][3] = DXL_HIBYTE(DXL_HIWORD(initial_positions[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_positions[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setPosition : [ID:%d] [POSITION (register):%d]", i+1, 0); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

/**************************************************************************/
// --- Velocity Dynamixel Class --- //
Velocity_Dynamixel::Velocity_Dynamixel(int n_dyna)
{
    // Init n_motors
    n_motors = n_dyna;

    // Error Handling
    // Init done by Super Class Constructor
    //dxl_error = 0;
    //dxl_comm_result = COMM_TX_FAIL;

    // Turn On LED, Current Mode and Enable Torque
    i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn on LED for Dynamixel ID %d", i+1);
            break;
        }

        // Current Drive Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_OP_MODE, VELOCITY_MODE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set Current Mode for Dynamixel ID %d", i+1);
            break;
        }

        // Enable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
            break;
        }
    }
}

bool Velocity_Dynamixel::set2registers(int32_t registers[])
{
     // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_velocities[n_motors][VELOCITY_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_velocities[i][0] = DXL_LOBYTE(DXL_LOWORD(registers[i]));
        param_goal_velocities[i][1] = DXL_HIBYTE(DXL_LOWORD(registers[i]));
        param_goal_velocities[i][2] = DXL_LOBYTE(DXL_HIWORD(registers[i]));
        param_goal_velocities[i][3] = DXL_HIBYTE(DXL_HIWORD(registers[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_velocities[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setVelocity : [ID:%d] [VELOCITY (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Velocity_Dynamixel::set2registers(std::vector<int32_t> registers)
{
         // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_velocities[n_motors][VELOCITY_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_velocities[i][0] = DXL_LOBYTE(DXL_LOWORD(registers[i]));
        param_goal_velocities[i][1] = DXL_HIBYTE(DXL_LOWORD(registers[i]));
        param_goal_velocities[i][2] = DXL_LOBYTE(DXL_HIWORD(registers[i]));
        param_goal_velocities[i][3] = DXL_HIBYTE(DXL_HIWORD(registers[i]));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_velocities[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setPosition : [ID:%d] [VELOCITY (register):%d]", i+1, registers[i]); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Velocity_Dynamixel::get_CurRegisters(std::vector<int16_t>& currents)
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add all motors' id
    i = 1;
    for(i; i <= n_motors; i++)
    {
        // Supposing that Motors ID are 1, 2, 3, 4, ..., n_motors
        dxl_addparam_result = current_syncRead.addParam((uint8_t) i);
        if (dxl_addparam_result != true) 
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", i);
            break;
        }
    }

    // Read all motors
    dxl_comm_result = current_syncRead.txRxPacket();
    if(dxl_comm_result == COMM_SUCCESS)
    {
        for(i = 1; i <= n_motors; i++)
        {
            currents.push_back(current_syncRead.getData((uint8_t) i, ADDR_PRESENT_CURRENT, CURRENT_BYTE));
        }

        current_syncRead.clearParam();
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get currents! Result: %d", dxl_comm_result);
        current_syncRead.clearParam();
        return false;       
    }
}

bool Velocity_Dynamixel::get_currents(std::vector<float>& currents)
{
    // Local Variable useful to convert
    std::vector<int16_t> current_registers;
    
    if(get_CurRegisters(current_registers))
    {
        // Convert
        for(i = 0; i < n_motors; i++)
        {
            currents[i] = register2Current(current_registers[i]); 
        }

        return true;
    }
    else
    {
        // Stop and get false
        return false;
    }
}

bool Velocity_Dynamixel::get_PosRegisters(std::vector<int32_t>& positions)
{
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Add all motors' id
    i = 1;
    for(i; i <= n_motors; i++)
    {
        // Supposing that Motors ID are 1, 2, 3, 4, ..., n_motors
        dxl_addparam_result = position_syncRead.addParam((uint8_t) i);
        if (dxl_addparam_result != true) 
        {
            ROS_ERROR("Failed to addparam to groupSyncRead for Dynamixel ID %d", i);
            break;
        }
    }

    // Read all motors
    dxl_comm_result = position_syncRead.txRxPacket();
    if(dxl_comm_result == COMM_SUCCESS)
    {
        for(i = 1; i <= n_motors; i++)
        {
            positions.push_back(position_syncRead.getData((uint8_t) i, ADDR_PRESENT_POSITION, POSITION_BYTE));
        }

        position_syncRead.clearParam();
        return true;
    }
    else
    {
        ROS_ERROR("Failed to get positions! Result: %d", dxl_comm_result);
        position_syncRead.clearParam();
        return false;       
    }
}

bool Velocity_Dynamixel::set2Zeros()
{
    // Error Handling
    dxl_error = 0;
    dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_velocities[n_motors][POSITION_BYTE];

    // Add parameters to sync_write obj
    i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_velocities[i][0] = DXL_LOBYTE(DXL_LOWORD(0));
        param_goal_velocities[i][1] = DXL_HIBYTE(DXL_LOWORD(0));
        param_goal_velocities[i][2] = DXL_LOBYTE(DXL_HIWORD(0));
        param_goal_velocities[i][3] = DXL_HIBYTE(DXL_HIWORD(0));

        dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_velocities[i]);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
            break;
        }
    }

    // Send all data
    dxl_comm_result = motors_syncWrite.txPacket();
    if (dxl_comm_result == COMM_SUCCESS) 
    {
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setVelocity : [ID:%d] [Velocity (register):%d]", i+1, 0); 
        }
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return true;
    } 
    else 
    {
        ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
        
        // Clear Parameters
        motors_syncWrite.clearParam();
        return false;
    }
}

bool Velocity_Dynamixel::set_velocities(std::vector<float> velocities)
{
    int32_t registers[n_motors];

    // Convert in Register Values
    i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = velocity2Register(velocities[i]);
    }

    return set2registers(registers);
}

/**************************************************************************/
// --- Functions --- //
int16_t current2Register(float current_value)
{
    return MAX_CURRENT_REGISTER*((int16_t) (current_value/MAX_CURRENT));
}

float register2Current(int16_t register_value)
{
    return MAX_CURRENT*(((float) register_value)/ ((float) MAX_CURRENT_REGISTER));
}

bool registerCur_saturation(int16_t &register_value) 
{
    // No need of sign function, because is only positive values
    if(abs(register_value) > MAX_CURRENT_REGISTER)
    {
        register_value = ((int16_t) sign(register_value))*MAX_CURRENT_REGISTER;
        return false;
    }
    else
    {
        return true;
    }
}

bool turns_saturation(float &turn) 
{
    // No need of sign function, because is only positive values
    if(turn > MAX_TURNS)
    {
        turn = MAX_TURNS;
        return false;
    }
    // No negative turns
    else if (turn < 0)
    {
        turn = 0.0;
        return false;
    }
    else
    {
        return true;
    }
}

float sign(float x)
{
    /*SIGN FUNCTION:*/
    if(x > 0)
        return 1.0;
    if(x < 0)
        return -1.0;
    else
        return 0.0;
}

float torque2Current(float torque)
{
    return COEFF_2*torque*torque + COEFF_1*torque + COEFF_0;
}

int16_t torque2Register(float torque)
{
    return current2Register(torque2Current(torque));
}

int32_t velocity2Register(float velocity_value)
{
    return MAX_VELOCITY_REGISTER*((int32_t) (velocity_value/MAX_VELOCITY));
}