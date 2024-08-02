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
        "[ TX Topic ]: %s, [ RX Topic ]: %s, [ ECU TOPIC ], %s, [ RES TOPIC ]: %s",
        this->m_sTopicTx.c_str(), this->m_sTopicRx.c_str(), this->m_sEcuStatusTopic.c_str(), this->m_sResStatusTopic.c_str()
    );

    this->connectCANBus();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    this->m_subCANRx = this->create_subscription<can_msgs::msg::Frame>(
        this->m_sTopicRx, 1, std::bind(&CANBusBridge::msgCANBusRxCallback, this, std::placeholders::_1));

    this->m_pubCANBusTx = this->create_publisher<can_msgs::msg::Frame>(this->m_sTopicTx, 1);
    this->m_pubEcuStatus = this->create_publisher<mmr_kria_base::msg::EcuStatus>(this->m_sEcuStatusTopic, qos);
    this->m_pubResStatus = this->create_publisher<mmr_kria_base::msg::ResStatus>(this->m_sResStatusTopic, qos);
    this->m_pubMissionSelect = this->create_publisher<std_msgs::msg::Int8>(this->m_sMissionSelectTopic, 1);
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
    declare_parameter("topic.missionSelectTopic", "");
    
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
    get_parameter("topic.missionSelectTopic", this->m_sMissionSelectTopic);

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
    can_msgs::msg::Frame txMsg;

    while (nMsgRead < this->m_nMaxMsgs) {
        
        if (read(this->m_nSocket, &frame, sizeof(struct can_frame)) <= 0)
            break;

        txMsg.id = frame.can_id;
        txMsg.dlc = frame.can_dlc;
        memcpy(&txMsg.data, &frame.data, CAN_MAX_DLEN);

        this->m_pubCANBusTx->publish(txMsg);

        if ((frame.can_id & ECU::MMR_ECU_MASK) == ECU::MMR_ECU_MASK)
            this->readEcuStatus(frame);
        
        if (frame.can_id == RES::MMR_RES_STATUS)
            this->readResStatus(frame);

        if (frame.can_id == COCKPIT::MMR_MISSION_SELECTED) {
            std_msgs::msg::Int8 msgMission;
            msgMission.data = frame.data[0];

            this->m_pubMissionSelect->publish(msgMission);
        }

        nMsgRead ++;
    }
}

void CANBusBridge::sendStatus()
{
    if (this->m_pubEcuStatus != nullptr) {
        this->m_msgEcuStatus.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(timing::Clock::get_time()).count();
        this->m_msgEcuStatus.header.frame_id = "ECU_STATE";

        this->m_pubEcuStatus->publish(this->m_msgEcuStatus);
    }

    if (this->m_pubResStatus != nullptr) {
        this->m_msgResStatus.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(timing::Clock::get_time()).count();
        this->m_msgResStatus.header.frame_id = "RES_STATE";

        this->m_pubResStatus->publish(this->m_msgResStatus);
    }
}

void CANBusBridge::readResStatus(can_frame frame)
{
    this->m_msgResStatus.emergency = frame.data[0] & RES::RES_SIGNAL_EMERGENCY;
    this->m_msgResStatus.go_signal = frame.data[0] & RES::RES_SIGNAL_GO;
    this->m_msgResStatus.bag = frame.data[0] & RES::RES_SIGNAL_BAG;
}

void CANBusBridge::readEcuStatus(can_frame frame)
{
    switch (frame.can_id)
    {
        case ECU::MMR_ECU_PEDAL_THROTTLE:
            this->m_msgEcuStatus.pot_pedal_a = (float)this->endian_cast<uint16_t>(frame.data) / 1000;
            this->m_msgEcuStatus.pot_pedal_b = (float)this->endian_cast<uint16_t>(frame.data + 2) / 1000;
            this->m_msgEcuStatus.pot_throttle_valve_a = (float)this->endian_cast<uint16_t>(frame.data + 4) / 100;
            this->m_msgEcuStatus.pot_throttle_valve_b = (float)this->endian_cast<uint16_t>(frame.data + 6) / 100;
            break;

        case ECU::MMR_ECU_TEMPERATURES:
            this->m_msgEcuStatus.temp_oil = this->endian_cast<uint16_t>(frame.data) - 40;
            this->m_msgEcuStatus.temp_engine = this->endian_cast<uint16_t>(frame.data + 2) - 40;
            this->m_msgEcuStatus.temp_intake = this->endian_cast<uint16_t>(frame.data + 4) - 40;
            this->m_msgEcuStatus.temp_ambient = this->endian_cast<uint16_t>(frame.data + 6) - 40;
            break;

        case ECU::MMR_ECU_ENGINE_FN1:
            this->m_msgEcuStatus.nmot = this->endian_cast<uint16_t>(frame.data);
            this->m_msgEcuStatus.vehicle_speed = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.gear = this->endian_cast<uint16_t>(frame.data + 4);
            this->m_msgEcuStatus.throttle = (float)this->endian_cast<uint16_t>(frame.data+6) / 100;
            break;
        
        case ECU::MMR_ECU_PRESSURES:
            this->m_msgEcuStatus.p_oil = (float)this->endian_cast<uint16_t>(frame.data) / 20;
            this->m_msgEcuStatus.p_fuel = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.p_intake = (float)this->endian_cast<uint16_t>(frame.data + 4) / 10;
            this->m_msgEcuStatus.p_ambient = (float)this->endian_cast<uint16_t>(frame.data + 6) / 10;
            break;

        case ECU::MMR_ECU_ENGINE_FN2:
            this->m_msgEcuStatus.battery_voltage = (float)this->endian_cast<uint16_t>(frame.data) / 1000;
            this->m_msgEcuStatus.accelerator_pedal = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.fan_control = (float)this->endian_cast<uint16_t>(frame.data + 4) / 100;
            break;

        case ECU::MMR_ECU_CLUTCH_STEER:
            this->m_msgEcuStatus.clutch_percentage = this->endian_cast<float>(frame.data);
            this->m_msgEcuStatus.steering_angle = (float)this->endian_cast<uint16_t>(frame.data + 4) / 10;
            this->m_msgEcuStatus.wheel_angle = (float)this->endian_cast<uint16_t>(frame.data + 6) / 10;
            break;

        case ECU::MMR_ECU_WHEEL_SPEEDS:
            this->m_msgEcuStatus.wheel_speed_front_left = (float)this->endian_cast<uint16_t>(frame.data) / 100;
            this->m_msgEcuStatus.wheel_speed_front_right = (float)this->endian_cast<uint16_t>(frame.data + 2) / 100;
            this->m_msgEcuStatus.wheel_speed_rear_left = (float)this->endian_cast<uint16_t>(frame.data + 4) / 100;
            this->m_msgEcuStatus.wheel_speed_rear_right = (float)this->endian_cast<uint16_t>(frame.data + 6) / 100;
            break;

        case ECU::MMR_ECU_SAFETY_CHECK:
            this->m_msgEcuStatus.p_brake_rear = (float)this->endian_cast<uint16_t>(frame.data) / 200;
            this->m_msgEcuStatus.p_brake_front = (float)this->endian_cast<uint16_t>(frame.data + 2) / 200;
            this->m_msgEcuStatus.error_throttle = (float)this->endian_cast<uint16_t>(frame.data + 4);
            this->m_msgEcuStatus.error_pedal = (float)this->endian_cast<uint16_t>(frame.data + 6);
            break;

        case ECU::MMR_ECU_EBS_PRESSURE:
            this->m_msgEcuStatus.p_ebs_1 = this->endian_cast<float>(frame.data);
            this->m_msgEcuStatus.p_ebs_2 = this->endian_cast<float>(frame.data + 4);
            break;
        
        case ECU::MMR_ECU_SET_LAUNCH_CONTROL:
            this->m_msgEcuStatus.bool_ack_ideal_launch_control = this->endian_cast<uint8_t>(frame.data);
            this->m_msgEcuStatus.bool_ack_real_launch_control = this->endian_cast<uint8_t>(frame.data + 1);
            break;
        
    }
}