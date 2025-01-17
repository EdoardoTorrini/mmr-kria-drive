#pragma once

#include <canopen_bridge/driver/lib/motor_maxon.hpp>

class MaxonSteer : public MaxonMotor
{
    
    private:
    
        const int m_nModeOfOp = MOTOR::PPM;
        int m_nVelocity, m_fMaxTarget;

        void initSteer() {
            /* set modes of operation */
            this->download<uint8_t>(0x6060, 0x00, this->m_nModeOfOp);

            /* set parameter */
            this->download<int>(0x6081, 0x00, this->m_nVelocity);

            /* enable device */
            this->init();
        }

    public:

        MaxonSteer(int nSocket, int nNodeId, int nTimeOutMsg, float fMaxTarget, int nVelocity) 
            : MaxonMotor(nSocket, nNodeId, m_nModeOfOp, nTimeOutMsg) 
        {
            this->m_nVelocity = nVelocity;
            this->m_fMaxTarget = fMaxTarget;

            for (int i = 0; i < 10; i++)
                this->initSteer();
        };

        MaxonSteer(int nSocket, int nNodeId, int nHomingMethod=0x25)
            : MaxonMotor(nSocket, nNodeId, MOTOR::HMM)
        {
            /* set modes of operation */
            this->download<uint8_t>(0x6060, 0x00, MOTOR::HMM);

            /* 0x25 -> set homing method */
            this->download<uint8_t>(0x6098, 0x00, nHomingMethod);

            /* enable device */
            this->init();

            /* start homing */
            this->download<uint16_t>(0x6040, 0x00, 0x001F);

            this->disable();
        }

        ~MaxonSteer() { this->disable(); };

        void writeTargetPos(int nTargetPos) {
            if ((nTargetPos < -this->m_fMaxTarget) || (nTargetPos > this->m_fMaxTarget))
                nTargetPos = this->m_fMaxTarget * std::copysign(1, nTargetPos);
            
            /* set target position */
            this->download<int>(0x607A, 0x00, nTargetPos);

            /* start postioning & toggle `new position` bit */
            this->toggle_new_pos(MOTOR::IDX_TOGGLE_NEW_POS::IDX_WRITE_ABS_POS);
        }

};