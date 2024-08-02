#include <canopen_bridge/canopen_bridge.hpp>

CANOpenBridge::CANOpenBridge() : EDFNode("canopen_bridge_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
    this->connectCANBus();
    
    RCLCPP_INFO(this->get_logger(), "[ INFO ] CAN INTERFACE: %s", this->m_sInterface);
    RCLCPP_INFO(this->get_logger(), "[ INFO ] CAN BITRATE: %d", this->m_nBitrate);

    this->m_subCmdSteer = this->create_subscription<mmr_kria_base::msg::CmdMotor>(
        this->m_sSteerTopic, 5, std::bind(&CANOpenBridge::msgCmdSteerCallback, this, std::placeholders::_1));

    this->m_subCmdBrake = this->create_subscription<mmr_kria_base::msg::CmdMotor>(
        this->m_sBrakeTopic, 5, std::bind(&CANOpenBridge::msgCmdBrakeCallback, this, std::placeholders::_1));

    this->m_subCmdClutch = this->create_subscription<mmr_kria_base::msg::CmdMotor>(
        this->m_sClucthTopic, 5, std::bind(&CANOpenBridge::msgCmdClutchCallback, this, std::placeholders::_1));

    this->m_subEcuStatus = this->create_subscription<mmr_kria_base::msg::EcuStatus>(
        this->m_sEcuStatusTopic, 1, std::bind(&CANOpenBridge::msgSelectorCallback, this, std::placeholders::_1));

    this->m_pubActuatorStatus = this->create_publisher<mmr_kria_base::msg::ActuatorStatus>(m_sStatusActuatorTopic, 1);

    this->m_msgActuatorStatus.brake_status  = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    this->m_msgActuatorStatus.steer_status  = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    
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
    declare_parameter("topic.ecuStatusTopic", "");

    declare_parameter("steer.node_id", 18);
    declare_parameter("steer.wheel_rate", 6.4286);
    declare_parameter("steer.inc_per_degree", 179.7224);
    declare_parameter("steer.max_target", 24000.0);
    declare_parameter("steer.velocity", 2750);

    declare_parameter("brake.node_id", 18);
    declare_parameter("brake.max_torque", 1500);
    declare_parameter("brake.return_pedal_torque", -20);

    declare_parameter("clutch.node_id", 16);
    declare_parameter("clutch.velocity", 3500);
    declare_parameter<std::vector<long int>>("clutch.step_maxon", std::vector<long int>());
    declare_parameter<std::vector<double>>("clutch.pot_val", std::vector<double>());
    
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
    get_parameter("topic.ecuStatusTopic", this->m_sEcuStatusTopic);

    get_parameter("steer.node_id", this->m_nSteerID);
    get_parameter("steer.wheel_rate", this->m_fWheelRate);
    get_parameter("steer.inc_per_degree", this->m_fIncPerDegree);
    get_parameter("steer.max_target", this->m_fMaxTarget);
    get_parameter("steer.velocity", this->m_nVelocity);

    get_parameter("brake.node_id", this->m_nBrakeId);
    get_parameter("brake.max_torque", this->m_nMaxTorque);
    get_parameter("brake.return_pedal_torque", this->m_nReturnPedalTorque);

    get_parameter("clutch.node_id", this->m_nClutchId);
    get_parameter("clutch.velocity", this->m_nVelocityClutch);
    get_parameter("clutch.step_maxon", this->m_aMotorSteps);
    get_parameter("clutch.pot_val", this->m_aPotVal);
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
    if (msg->enable && (this->m_mSteer == nullptr)) {
        /* Enables the steer motor in PPM */
        this->m_mSteer = new MaxonSteer(
            this->m_nSocket, this->m_nSteerID, this->m_nTimeoutMsg,
            this->m_fMaxTarget, this->m_nVelocity
        );
        this->m_msgActuatorStatus.brake_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::POSITION_MODE);
        RCLCPP_INFO(this->get_logger(), "[ INFO ] ENABLE RECEIVED FOR STEER");
        return;
    }

    if (msg->disable && (this->m_mSteer != nullptr)) {
        delete this->m_mSteer;
        this->m_msgActuatorStatus.steer_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
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
    if (msg->enable && (this->m_mBrake == nullptr)) {
        /* Enables the brake motor in CST */
        this->m_mBrake = new MaxonBrake(
            this->m_nSocket, this->m_nBrakeId, this->m_nTimeoutMsg,
            this->m_nMaxTorque, m_nReturnPedalTorque
        );

        RCLCPP_INFO(this->get_logger(), "[ INFO ] ENABLE RECEIVED FOR BRAKE");
        this->m_msgActuatorStatus.brake_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::TORQUE_MODE);
        return;
    }

    if (msg->disable && (this->m_mBrake != nullptr)) {
        delete this->m_mBrake;
        this->m_msgActuatorStatus.brake_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    }

    msg->brake_torque *= 1000;
    int nTorque = std::abs(std::round(msg->brake_torque));
    
    if (this->m_mBrake != nullptr)
        this->m_mBrake->writeTargetTorque(nTorque);
}

void CANOpenBridge::msgCmdClutchCallback(mmr_kria_base::msg::CmdMotor::SharedPtr msg)
{
    if (msg->enable && (this->m_mClutch == nullptr)) {
        this->m_mClutch = new MaxonClutch(
            this->m_nSocket, this->m_nClutchId, 
            this->m_nTimeoutMsg, this->m_nVelocityClutch,
            this->m_aMotorSteps, this->m_aPotVal
        );
        RCLCPP_INFO(this->get_logger(), "[ INFO ] ENABLE RECEIVED FOR CLUTCH");
        return;
    }

    if (msg->disable && (this->m_mClutch != nullptr)) { 
        delete this->m_mClutch;
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISABLE);
    }

    this->m_bToDisangaged = msg->disengaged;
}

void CANOpenBridge::msgSelectorCallback(mmr_kria_base::msg::EcuStatus::SharedPtr msg)
{
    if (this->m_fClutchPot.has_value())
        this->msgEcuStatusCallback(msg);
    else this->msgEngageInitClutch(msg);
}

void CANOpenBridge::msgEngageInitClutch(mmr_kria_base::msg::EcuStatus::SharedPtr msg)
{
    float fClutchPot = msg->clutch_percentage;
    if (this->m_mClutch == nullptr)
        return;
    
    if (fClutchPot > this->m_aPotVal[MOTOR::CLUTCH_SET_INIT]){
        this->m_mClutch->engage(static_cast<int>(this->m_aMotorSteps[MOTOR::CLUTCH_SET_INIT]));
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::DISENGAGE);
    }
    else{
        this->m_fClutchPot = fClutchPot;
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned char>(MOTOR::ACTUATOR_STATUS::ENGAGE);
    }
}

void CANOpenBridge::msgEcuStatusCallback(mmr_kria_base::msg::EcuStatus::SharedPtr msg) 
{ 
    this->m_fClutchPot = msg->clutch_percentage; 
}

void CANOpenBridge::monitoringClutch()
{
    if (!this->m_fClutchPot.has_value())
        return;
    
    float fClutchPot = this->m_fClutchPot.value();

    if (
        ((fClutchPot < this->m_aPotVal[MOTOR::CLUTCH_SET_ENGAGED_4]) && (!this->m_bToDisangaged)) || 
        ((fClutchPot > this->m_aPotVal[MOTOR::CLUTCH_SET_DISENGAGED]) && (this->m_bToDisangaged))
    ) return;
    

    if (!this->m_bToDisangaged) 
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned int>(this->m_mClutch->engage(fClutchPot));
    else 
        this->m_msgActuatorStatus.clutch_status = static_cast<unsigned int>(this->m_mClutch->disengage(fClutchPot));

}