#include <canopen_bridge/driver/lib/motor_maxon.hpp>

MaxonMotor::MaxonMotor(int socket, int node_id, int mode_of_op, int timeout_msg_count)
{
    this->socket = socket;
    this->node_id = node_id;
    this->mode_of_op = mode_of_op;

    this->timeout_msg_count = timeout_msg_count;
}

void MaxonMotor::init()
{       
    /* shutdown */
    this->download<uint16_t>(0x6040, 0x00, 0x0006);

    /* switch on & enable */
    this->download<uint16_t>(0x6040, 0x00, 0x000F);
}

int MaxonMotor::send_msg_on_canbus(CANOpen::canopen_frame cof, CANOpen::canopen_frame* rcv, int len)
{
    can_frame tx = cof.copy(len), rx;
    int msg_read_count = 0;

    if (write(this->socket, &tx, sizeof(struct can_frame)) != sizeof(struct can_frame))
        return -1;

    while (msg_read_count++ < this->timeout_msg_count) {
        
        if (read(this->socket, &rx, sizeof(struct can_frame)) < 0)
            return -1;
        
        *rcv = rx;

        if ((cof.index == rcv->index) && (cof.subindex == rcv->subindex)) return 0;

    }

    return -1;
}

void MaxonMotor::disable()
{
    this->download<uint16_t>(0x6040, 0x00, 0x000B);
    this->download<uint16_t>(0x6040, 0x00, 0x0000);
}

void MaxonMotor::toggle_new_pos()
{
    this->download<uint16_t>(0x6040, 0x00, 0x000F);
    this->download<uint16_t>(0x6040, 0x00, 0x003F);
}
