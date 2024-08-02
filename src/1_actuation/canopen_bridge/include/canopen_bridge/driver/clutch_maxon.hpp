#pragma once

#include <canopen_bridge/driver/lib/motor_maxon.hpp>
#include <vector>

class MaxonClutch : private MaxonMotor
{
    private:

        int m_nVelocity;
        
        std::vector<long int> m_aMotorSteps;
        std::vector<double> m_aPotVal;
        
        const int m_nModeOfOp = MOTOR::PPM;

        void initClutch() {
            /* set modes of operation */
            this->download<uint8_t>(0x6060, 0x00, this->m_nModeOfOp);

            /* set parameter */
            this->download<int>(0x6081, 0x00, this->m_nVelocity);

            /* enable device */
            this->init();
        }

    public:

        MaxonClutch (int nSocket, int nNodeId, int nTimeOutMsg, int nVelocity, 
                std::vector<long int> m_aMotorSteps, std::vector<double> m_aPotVal)
            : MaxonMotor(nSocket, nNodeId, m_nModeOfOp, nTimeOutMsg) 
        {
            this->m_nVelocity   = nVelocity;
            this->m_aMotorSteps = m_aMotorSteps;
            this->m_aPotVal     = m_aPotVal;

            for (int i = 0; i < 10; i++)
                this->initClutch();
        }

        MOTOR::ACTUATOR_STATUS disengage (float fClutchPot) {
            if (fClutchPot < this->m_aPotVal[MOTOR::INDEX_CLUTCH::CLUTCH_SET_DISENGAGED]) {
                this->download<int>(0x607A, 0x00, this->m_aMotorSteps[MOTOR::INDEX_CLUTCH::CLUTCH_SET_DISENGAGED]);
                this->toggle_new_pos(MOTOR::IDX_TOGGLE_NEW_POS::IDX_WRITE_REL_POS);
                return MOTOR::ACTUATOR_STATUS::ENGAGE;
            }
            else return MOTOR::ACTUATOR_STATUS::DISENGAGE;
        }

        void engage(int nSteps) {
            this->download<int>(0x607A, 0x00, nSteps);
            this->toggle_new_pos(MOTOR::IDX_TOGGLE_NEW_POS::IDX_WRITE_REL_POS);
        }

        MOTOR::ACTUATOR_STATUS engage (float fClutchPot) {

            for (int i = MOTOR::INDEX_CLUTCH::CLUTCH_SET_ENGAGED_1; i <= MOTOR::INDEX_CLUTCH::CLUTCH_SET_ENGAGED_4; i++) {
                if (fClutchPot > this->m_aPotVal[i]) {
                    this->download<int>(0x607A, 0x00, this->m_aMotorSteps[i]);
                    this->toggle_new_pos(MOTOR::IDX_TOGGLE_NEW_POS::IDX_WRITE_REL_POS);
                    return MOTOR::ACTUATOR_STATUS::DISENGAGE; 
                }
            }
            return MOTOR::ACTUATOR_STATUS::ENGAGE;
        }

        ~MaxonClutch () { this->disable(); }
};