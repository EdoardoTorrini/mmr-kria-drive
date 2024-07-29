#include <canbus_bridge/canbus_bridge.hpp>

CANBusBridge::CANBusBridge() : EDFNode("canbus_bridge_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
    
    RCLCPP_INFO(
        this->get_logger(),
        "[ INTERFACE ]: %s, [ BITRATE ]: %d, [ DEBUG ]: %d",
        this->m_sInterface.c_str(), this->m_nBitrate, this->m_bDebug
    );

    RCLCPP_INFO(
        this->get_logger(),
        "[ TX Topic ]: %s, [ RX Topic ]: %s",
        this->m_sTopicTx.c_str(), this->m_sTopicRx.c_str()
    );

    this->connectCANBus();

    this->m_subCANRx = this->create_subscription<can_msgs::msg::Frame>(
        this->m_sTopicRx, 1, std::bind(&CANBusBridge::msgCANBusRxCallback, this, std::placeholders::_1));

    this->m_pubCANBusTx = this->create_publisher<can_msgs::msg::Frame>(this->m_sTopicTx, 1);
    this->m_pubEcuStatus = this->create_publisher<mmr_kria_base::msg::EcuStatus>(this->m_sEcuStatusTopic, 1);
    this->m_pubResStatus = this->create_publisher<mmr_kria_base::msg::ResStatus>(this->m_sResStatusTopic, 1);
}

void CANBusBridge::loadParameters()
{
    declare_parameter("generic.interface", "");
    declare_parameter("generic.bitrate", 5000000);
	declare_parameter("generic.WCET", 5000000);
	declare_parameter("generic.period", 10000000);
	declare_parameter("generic.deadline", 10000000);
    declare_parameter("generic.max_msgs", 5);
    declare_parameter("generic.debug", false);

    declare_parameter("topic.canTxTopic", "");
    declare_parameter("topic.canRxTopic", "");
    declare_parameter("topic.ecuStatusTopic", "");
    declare_parameter("topic.resStatusTopic", "");
    
    get_parameter("generic.interface", this->m_sInterface);
    get_parameter("generic.bitrate", this->m_nBitrate);
	get_parameter("generic.WCET", this->m_nWCET);
	get_parameter("generic.period", this->m_nPeriod);
	get_parameter("generic.deadline", this->m_nDeadline);
    get_parameter("generic.max_msgs", this->m_nMaxMsgs);
    get_parameter("generic.debug", this->m_bDebug);

    get_parameter("topic.canTxTopic", this->m_sTopicTx);
    get_parameter("topic.canRxTopic", this->m_sTopicRx);
    get_parameter("topic.ecuStatusTopic", this->m_sEcuStatusTopic);
    get_parameter("topic.resStatusTopic", this->m_sResStatusTopic);

}

void CANBusBridge::msgCANBusRxCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if (this->m_bDebug)
        RCLCPP_INFO(
            this->get_logger(),
            "[ RECV new MSG ] -> [ ID ]: %d, [ DLC ]: %d",
            msg->id, msg->dlc
        );

    struct can_frame frame = {
        .can_id = msg->id,
        .len = msg->dlc,
    };

    memcpy(frame.data, &msg->data, CAN_MAX_DLEN);

    if (write(this->m_nSocket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        RCLCPP_ERROR(this->get_logger(), "Error on write data to socket");
}

void CANBusBridge::connectCANBus()
{
    this->m_nSocket = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (this->m_nSocket < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error on socket define");
        throw 1;
    }

    strcpy(this->m_ifr.ifr_name, this->m_sInterface.c_str());
    ioctl(this->m_nSocket, SIOCGIFINDEX, &this->m_ifr);

    memset(&this->m_addr, 0, sizeof(this->m_addr));
    this->m_addr.can_family = AF_CAN;
    this->m_addr.can_ifindex = this->m_ifr.ifr_ifindex;

    if (bind(this->m_nSocket, (struct sockaddr *)&this->m_addr, sizeof(this->m_addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error on socker association");
        throw 1;        
    }
}

void CANBusBridge::readMsgFromCANBus()
{
    struct can_frame frame;
    int nMsgRead = 0;
    can_msgs::msg::Frame *txMsg;

    while (nMsgRead < this->m_nMaxMsgs) {
        
        if (read(this->m_nSocket, &frame, sizeof(struct can_frame)) <= 0)
            break;

        txMsg->id = frame.can_id;
        txMsg->dlc = frame.len;
        memcpy(&txMsg->data, frame.data, CAN_MAX_DLEN);

        this->m_pubCANBusTx->publish(*txMsg);

        if ((frame.can_id & ECU::MMR_ECU_MASK) == ECU::MMR_ECU_MASK)
            this->readEcuStatus(frame);

        nMsgRead ++;
    }
}

void CANBusBridge::readEcuStatus(can_frame frame)
{
    switch (frame.can_id)
    {
        case ECU::MMR_ECU_PEDAL_THROTTLE:
            memcpy(&this->m_msgEcuStatus.pot_pedal_a, &frame.data, 2);
            this->m_msgEcuStatus.pot_pedal_a /= 1000;

            memcpy(&this->m_msgEcuStatus.pot_pedal_b, &frame.data[2], 2);
            this->m_msgEcuStatus.pot_pedal_b /= 1000;
            memcpy(&this->m_msgEcuStatus.pot_throttle_valve_a, &frame.data[4], 2);
            this->m_msgEcuStatus.pot_throttle_valve_a /= 1000;

            memcpy(&this->m_msgEcuStatus.pot_throttle_valve_b, &frame.data[6], 2);
            this->m_msgEcuStatus.pot_throttle_valve_b /= 1000;

            break;

        case ECU::MMR_ECU_TEMPERATURES:
            break;

        case ECU::MMR_ECU_ENGINE_FN1:
            memcpy(&this->m_msgEcuStatus.nmot, &frame.data, 2);

            memcpy(&this->m_msgEcuStatus.vehicle_speed, &frame.data[2], 2);
            this->m_msgEcuStatus.vehicle_speed /= 100;

            memcpy(&this->m_msgEcuStatus.gear, &frame.data[4], 2);

            memcpy(&this->m_msgEcuStatus.throttle, &frame.data[6], 2);
            this->m_msgEcuStatus.throttle /= 100;
        
        default:
            break;
    }
}