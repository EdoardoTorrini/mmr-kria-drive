#include "as_flowchart/as_flowchart.hpp"

ASFlowchart::ASFlowchart() : Node("as_flowchart_node")
{
    this->loadParameters();
    this->configureEDFScheduler();
}

void ASFlowchart::loadParameters() 
{
	declare_parameter("generic.period", 10000000);
	declare_parameter("topic.ebsTopic", "");
	declare_parameter("topic.canTopic", "");
	
	get_parameter("generic.period", this->m_nPeriod);
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