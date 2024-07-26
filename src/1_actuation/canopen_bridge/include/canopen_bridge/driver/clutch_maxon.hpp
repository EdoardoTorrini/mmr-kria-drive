#pragma once

#include <canopen_bridge/driver/lib/motor_maxon.hpp>

class MaxonClutch : public MaxonMotor
{
    private:

    public:

        MaxonClutch(int nSocket, int nNodeId)
            : MaxonMotor(nSocket, nNodeId, m_nModeOfOp, nTimeOutMsg) 
        {
        
        }
        
        ~MaxonClutch(){ this->disable(); }
}