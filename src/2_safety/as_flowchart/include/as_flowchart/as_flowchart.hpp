#pragma once

#include <mmr_kria_base/edf_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <std_msgs/msg/int8.hpp>
#include <can_msgs/msg/frame.hpp>

#include <sys/syscall.h>
#include <linux/sched.h>
#include <sched.h>

#define READY 0
#define EMERGENCY 1

class ASFlowchart : public EDFNode
{
    private:

        std::string m_sTopicEBS;
        std::string m_sTopicCANTx, m_sTopicCANRx;

        int m_nSTATE;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subEBSupervisor;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subCANBusMsg;

        void msgCANBusCallback(const can_msgs::msg::Frame::SharedPtr canbus_msg);
        void ebsStatusCallback(const std_msgs::msg::Int8::SharedPtr status);

        void loadParameters();

    public:

        ASFlowchart();
        ~ASFlowchart() { RCLCPP_INFO(this->get_logger(), "CLOSING!"); };

        void finiteStateMachine();
        
};