#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <std_msgs/msg/int8.hpp>

#include <sys/syscall.h>
#include <linux/sched.h>
#include <sched.h>

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
        int m_nPeriod;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subEBSupervisor;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subCANBusMsg;
        
};