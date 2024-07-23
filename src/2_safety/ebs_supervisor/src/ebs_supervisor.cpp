#include <ebs_supervisor/ebs_supervisor.hpp>

EBSSupervisor::EBSSupervisor() : EDFNode("ebs_supervisor_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);


    this->m_subCANBusMsg = this->create_subscription<can_msgs::msg::Frame>(
        this->m_sTopicCANTx, 1, std::bind(&EBSSupervisor::msgCANBusCallback, this, std::placeholders::_1));

    this->m_pubEBSStatus = this->create_publisher<std_msgs::msg::Int8>(this->m_sTopicEBS, 1);
}

void EBSSupervisor::loadParameters()
{
	declare_parameter("generic.WCET", 5000000);
	declare_parameter("generic.period", 10000000);
	declare_parameter("generic.deadline", 10000000);

    declare_parameter("topic.canTxTopic", "");
    declare_parameter("topic.ebsStatusTopic", "");
    
	get_parameter("generic.WCET", this->m_nWCET);
	get_parameter("generic.period", this->m_nPeriod);
	get_parameter("generic.deadline", this->m_nDeadline);

    get_parameter("topic.canTxTopic", this->m_sTopicCANTx);
    get_parameter("topic.ebsStatusTopic", this->m_sTopicEBS);
}

void EBSSupervisor::msgCANBusCallback(const can_msgs::msg::Frame::SharedPtr canbus_msg)
{
    switch (canbus_msg->id)
    {
        case RES::MMR_RES_STATUS:
            this->m_bEmergency = canbus_msg->data[0] & RES::RES_SIGNAL_EMERGENCY;
            this->m_bGoSignal = canbus_msg->data[0] & RES::RES_SIGNAL_GO;
            this->m_bBagSignal = canbus_msg->data[0] & RES::RES_SIGNAL_BAG;
            break;

        case ECU::MMR_ECU_SAFETY_CHECK:
            memcpy(&this->m_fPressionBrakeFront, canbus_msg->data.begin(), 2);
            memcpy(&this->m_fPressionBrakeFront, canbus_msg->data.begin() + 2, 2);
            break;
        
        case ECU::MMR_ECU_EBS_PRESSURE:
            memcpy(&this->m_fEBSPression1, canbus_msg->data.begin(), 4);
            memcpy(&this->m_fEBSPression2, canbus_msg->data.begin() + 4, 4);
            break;

        case COCKPIT::MMR_MISSION_SELECTED:
            this->m_bIsAutonomousMission = (int)canbus_msg->data[0] != COCKPIT::MMR_MISSION_MANUAL ? true : false;
            break;
    }
}