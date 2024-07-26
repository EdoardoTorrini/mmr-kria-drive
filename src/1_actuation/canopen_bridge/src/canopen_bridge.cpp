#include <canopen_bridge/canopen_bridge.hpp>

CANOpenBridge::CANOpenBridge() : EDFNode("canopen_bridge_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
    this->connectCANBus();

    this->m_subCmdSteer = this->create_subscription<mmr_kria_base::msg::CmdMotor>(
        this->m_sSteerTopic, 1, std::bind(&CANOpenBridge::msgCmdSteerCallback, this, std::placeholders::_1));

    this->m_subCmdBrake = this->create_subscription<mmr_kria_base::msg::CmdMotor>(
        this->m_sSteerTopic, 1, std::bind(&CANOpenBridge::msgCmdBrakeCallback, this, std::placeholders::_1));

    
    
    
    /* Enables the steer motor in HMM */
    // this->m_mSteer = new MaxonSteer(this->m_nSocket, this->m_nSteerID);
    // delete this->m_nSteer;

    /**
     * Example for the use
     */
    for (int i = 0; i < 10; i++)
        this->m_mSteer->initSteer();   
    
    sleep(10);

    unsigned short usSteerVoltage = this->m_mSteer->upload<unsigned short>(0x2200, 0x01);
    RCLCPP_INFO(this->get_logger(), "[ ACTUAL STEER VOLTAGE ]: %d", usSteerVoltage);
    // this->m_mSteer->writeTargetTorque(80);
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

    declare_parameter("brake.node_id", 18);
    declare_parameter("brake.max_torque", 6.4286);
    declare_parameter("brake.return_pedal_torque", 1500);
    declare_parameter("steer.max_target", -20);
    
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

    get_parameter("brake.node_id", this->m_nBrakeId);
    get_parameter("brake.max_torque", this->m_nMaxTorque);
    get_parameter("brake.return_pedal_torque", this->m_nReturnPedalTorque);
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
    if (msg->enable) {
        /* Enables the steer motor in PPM */
        this->m_mSteer = new MaxonSteer(
            this->m_nSocket, this->m_nSteerID, this->m_nTimeoutMsg,
            this->m_fMaxTarget, this->m_nVelocity
        );
        return;
    }

    if (msg->homing) {
        this->m_mSteer = new MaxonSteer(this->m_nSocket, m_nSteerID);
        return;
    }
    
    /* convert radiant into degrees */
    msg->wheel_angle *= 180 / M_PI;

    /* Compute the incremets to do */
    int nIncrements = std::round(msg->wheel_angle * this->m_fWheelRate * this->m_fIncPerDegree);

    if (this->m_mSteer != nullptr)  
        this->m_mSteer->writeTargetPos(nIncrements);
}

void CANOpenBridge::msgCmdBrakeCallback(mmr_kria_base::msg::CmdMotor::SharedPtr msg)
{
    if (msg->enable) {
        /* Enables the brake motor in CST */
        this->m_mBrake = new MaxonBrake(
            this->m_nSocket, this->m_nSteerID, this->m_nTimeoutMsg,
            this->m_nMaxTorque, m_nReturnPedalTorque
        );
        return;
    }

    msg->brake_torque *= 1000;
    int nTorque = std::abs(std::round(msg->brake_torque));
    
    if (this->m_mBrake != nullptr)
        this->m_mBrake->writeTargetTorque(nTorque);
}