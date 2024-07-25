#include <canopen_bridge/canopen_bridge.hpp>

CANOpenBridge::CANOpenBridge() : EDFNode("canopen_bridge_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
    this->connectCANBus();

    this->m_mSteer = new MaxonMotor(this->m_nSocket, this->m_nSteerID, this->m_nTimeoutMsg);
    this->m_mSteer->download<uint8_t>(0x6060, 0x00, 0x01);
    this->m_mSteer->download<uint16_t>(0x6040, 0x00, 0x0600);
    this->m_mSteer->download<uint16_t>(0x6040, 0x00, 0x0F00);
    
}

void CANOpenBridge::loadParameters()
{
    declare_parameter("generic.interface", "");
    declare_parameter("generic.bitrate", 5000000);
	declare_parameter("generic.WCET", 5000000);
	declare_parameter("generic.period", 10000000);
	declare_parameter("generic.deadline", 10000000);
    declare_parameter("generic.timeout_msgs", 5);
    declare_parameter("generic.debug", false);

    declare_parameter("topic.steerTopic", "");
    declare_parameter("topic.brakeTopic", "");
    declare_parameter("topic.clutchTopic", "");
    declare_parameter("topic.statusActuatorTopic", "");

    declare_parameter("steer.node_id", 18);
    declare_parameter("steer.wheel_rate", 6.4286);
    declare_parameter("steer.inc_per_degree", 179.7224);
    declare_parameter("steer.max_target", 24000.0);
    declare_parameter("steer.velocity", 2750);
    
    get_parameter("generic.interface", this->m_sInterface);
    get_parameter("generic.bitrate", this->m_nBitrate);
	get_parameter("generic.WCET", this->m_nWCET);
	get_parameter("generic.period", this->m_nPeriod);
	get_parameter("generic.deadline", this->m_nDeadline);
    get_parameter("generic.timeout_msgs", this->m_nTimeoutMsg);
    get_parameter("generic.debug", this->m_bDebug);

    get_parameter("topic.steerTopic", this->m_sSteerTopic);
    get_parameter("topic.brakeTopic", this->m_sBrakeTopic);
    get_parameter("topic.clutchTopic", this->m_sClucthTopic);
    get_parameter("topic.statusActuatorTopic", this->m_sStatusActuatorTopic);

    get_parameter("steer.node_id", this->m_nSteerID);
    get_parameter("steer.wheel_rate", this->m_fWheelRate);
    get_parameter("steer.inc_per_degree", this->m_fIncPerDegree);
    get_parameter("steer.max_target", this->m_fMaxTarget);
    get_parameter("steer.velocity", this->m_nVelocity);
}

void CANOpenBridge::connectCANBus()
{
    this->m_nSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
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

    /* send OPERATIONAL MODE on CanOpen Network */
    can_frame frame = {
        .can_id = 0x00,
        .can_dlc = 2,
        .data = {0x01, 0x12}
    };

    if (write(this->m_nSocket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        return;
}

void CANOpenBridge::msgCmdSteerCallback(mmr_kria_base::msg::CmdMotor::SharedPtr msg)
{
    
}