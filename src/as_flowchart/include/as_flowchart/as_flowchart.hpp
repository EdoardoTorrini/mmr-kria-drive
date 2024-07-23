#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <std_msgs/msg/int8.hpp>
#include <can_msgs/msg/frame.hpp>

#include <sys/syscall.h>
#include <linux/sched.h>
#include <sched.h>

#define READY 0
#define EMERGENCY 1

class ASFlowchart : public rclcpp::Node
{
    typedef struct {
        uint32_t size;
        uint32_t sched_policy;
        uint64_t sched_flags;
        int32_t sched_nice;
        uint32_t sched_priority;
        uint64_t sched_runtime;
        uint64_t sched_deadline;
        uint64_t sched_period;
    } sched_attr;

    private:

        std::string m_sTopicEBS;
        std::string m_sTopicCANBus;
        int m_nPeriod;

        int m_nSTATE;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subEBSupervisor;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subCANBusMsg;

        void msgCANBusCallback(const can_msgs::msg::Frame::SharedPtr canbus_msg);
        void ebsStatusCallback(const std_msgs::msg::Int8::SharedPtr status);

        void loadParameters();
        void configureEDFScheduler();

    public:

        ASFlowchart();
        ~ASFlowchart() { RCLCPP_INFO(this->get_logger(), "CLOSING!"); };

        void finiteStateMachine();
        
};