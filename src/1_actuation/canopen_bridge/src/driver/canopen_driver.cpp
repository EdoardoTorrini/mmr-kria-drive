#include <canopen_bridge/driver/canopen_driver.hpp>

MaxonMotor::MaxonMotor(int nSocket, int nNodeID, int nTimeOutMsg, int nModeOfOp)
{
    this->m_nSocket = nSocket;
    this->m_nNodeID = nNodeID;
    this->m_nTimeOutMsg = nTimeOutMsg;
    this->m_nModeOfOp = nModeOfOp;
}

void MaxonMotor::initMotor()
{
    /* set modes of operation */
    this->download<uint8_t>(0x6060, 0x00, this->m_nModeOfOp);

    if (this->m_nModeOfOp == MOTOR::HMM) 
        /* 0x25 -> absolute zero set to current position */
        this->download<uint8_t>(0x6098, 0x00, 0x25);

    /* shutdown */
    this->download<uint16_t>(0x6040, 0x00, 0x0006);

    /* switch on & enable */
    this->download<uint16_t>(0x6040, 0x00, 0x000F);
}

int MaxonMotor::sendMsgOverCANBus(CANOpen::canopen_frame cof, CANOpen::canopen_frame* rcv, int len)
{
    can_frame tx = cof.copy(len), rx;
    int nMsgRead = 0;

    if (write(this->m_nSocket, &tx, sizeof(struct can_frame)) != sizeof(struct can_frame))
        return -1;

    while (nMsgRead++ < this->m_nTimeOutMsg) {
        
        if (read(this->m_nSocket, &rx, sizeof(struct can_frame)) < 0)
            return -1;
        
        *rcv = rx;

        if ((cof.index == rcv->index) && (cof.subindex == rcv->subindex)) return 0;

    }

    return -1;
}

void MaxonMotor::disableMotor()
{
    this->download<uint16_t>(0x6040, 0x00, 0x000B);
    this->download<uint16_t>(0x6040, 0x00, 0x0000);
}

int MaxonMotor::writeTargetPos(int targetPos)
{
    if (this->m_nModeOfOp != MOTOR::PPM )
        return -1;
    /*
    if (targetPos > this->m_nMaxSteerTarget || targetPos < -this->m_nMaxSteerTarget)
        targetPos = this->m_nMaxSteerTarget * int(std::copysign(1,targetPos));
    */

    this->download<int>(0x607A, 0x00, targetPos);
    this->download<uint16_t>(0x6040, 0x00, 0x000F);
    this->download<uint16_t>(0x6040, 0x00, 0x003F);

    return 0;

}

int MaxonMotor::writeTargetTorque(int targetTorque)
{
    if (this->m_nModeOfOp != MOTOR::CST)
        return -1;

    /*
    targetTorque = targetTorque > 0 ? targetTorque : 0;
    targetTorque = targetTorque < this->m_nMaxBrakeTarget ? targetTorque : this->m_nMaxBrakeTarget; 
    */

    this->download<uint16_t>(0x60B2, 0x00, targetTorque);
}
