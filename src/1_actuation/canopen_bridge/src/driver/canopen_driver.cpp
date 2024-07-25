#include <canopen_bridge/driver/canopen_driver.hpp>

MaxonMotor::MaxonMotor(int nSocket, int nNodeID, int nTimeOutMsg)
{
    this->m_nSocket = nSocket;
    this->m_nNodeID = nNodeID;
    this->m_nTimeOutMsg = nTimeOutMsg;
}

int MaxonMotor::sendMsgOverCANBus(CANOpen::canopen_frame cof, CANOpen::canopen_frame* rcv)
{
    can_frame tx = cof.copy(), rx;
    int nMsgRead = 0;

    if (write(this->m_nSocket, &tx, sizeof(struct can_frame)) != sizeof(struct can_frame))
        return -1;

    while (nMsgRead < this->m_nTimeOutMsg) {
        
        if (read(this->m_nSocket, &rx, sizeof(struct can_frame)) < 0)
            return -1;
        
        *rcv = rx;

        if ((cof.index == rcv->index) && (cof.subindex == rcv->subindex)) return 0;

    }

    return -1;
}