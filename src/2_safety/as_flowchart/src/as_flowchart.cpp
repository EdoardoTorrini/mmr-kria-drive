#include "as_flowchart/as_flowchart.hpp"

ASFlowchart::ASFlowchart() : EDFNode("as_flowchart_node")
{
    this->loadParameters();
    this->configureEDFScheduler(this->m_nPeriod, this->m_nWCET, this->m_nDeadline);
}

void ASFlowchart::loadParameters() 
{
	declare_parameter("generic.WCET", 5000000);
	declare_parameter("generic.period", 10000000);
	declare_parameter("generic.deadline", 10000000);

	declare_parameter("topic.ebsTopic", "");
	declare_parameter("topic.canTopic", "");
	
	get_parameter("generic.WCET", this->m_nWCET);
	get_parameter("generic.period", this->m_nPeriod);
	get_parameter("generic.deadline", this->m_nDeadline);

	get_parameter("topic.ebsTopic", this->m_sTopicEBS);
	get_parameter("topic.canTopic", this->m_sTopicCANBus);
}

void ASFlowchart::ebsStatusCallback(const std_msgs::msg::Int8::SharedPtr status)
{
    switch (status->data)
    {
        case READY:
            break;
        
        case EMERGENCY:
            break;
    }
}

void ASFlowchart::msgCANBusCallback(const can_msgs::msg::Frame::SharedPtr canbus_msg)
{
    switch (canbus_msg->id)
    {
        case 0:
            break;
    }
}

void ASFlowchart::finiteStateMachine() { /* case per check STATE */ }