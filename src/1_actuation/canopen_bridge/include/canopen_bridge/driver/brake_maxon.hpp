#pragma once

#include <canopen_bridge/driver/lib/motor_maxon.hpp>

class MaxonBrake : public MaxonMotor
{
    private:

        const int m_nModeOfOp = MOTOR::CST;
        int m_nMaxTorque, m_nReturnPedalTorque;

    public:

        MaxonBrake(int nSocket, int nNodeId, int nTimeOutMsg, int nMaxTorque, int nReturnPedalTorque)
            : MaxonMotor(nSocket, nNodeId, m_nModeOfOp, nTimeOutMsg) 
        {
            this->m_nMaxTorque = nMaxTorque;
            this->m_nReturnPedalTorque = nReturnPedalTorque;

            for (int i = 0; i < 10; i ++)
                this->initBrake();
        }

        ~MaxonBrake() { this->disable(); };

        void initBrake() {
            /* set modes of operation */
            this->download<uint8_t>(0x6060, 0x00, this->m_nModeOfOp);

            /* enable device */
            this->init();
        }

        void writeTargetTorque(int nTargetTorque) {
            uint16_t nTarget = nTargetTorque > 0 ? static_cast<uint16_t>(nTargetTorque) : 0;
            nTarget = nTarget < this->m_nMaxTorque ? nTarget : this->m_nMaxTorque;

            /* set velocity */
            this->download<uint16_t>(0x60B2, 0x00, nTargetTorque);
        }

        void returnToZero() {
            uint16_t nButton = this->upload<uint16_t>(0x3141, 0x01);

            if (nButton)
                this->writeTargetTorque(0);
            else
                this->writeTargetTorque(this->m_nReturnPedalTorque);
        }
};