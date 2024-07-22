#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <std_msgs/msg/int8.hpp>

#include <thread>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include <sys/syscall.h>
#include <linux/sched.h>
#include <sched.h>

#define gettid() syscall(SYS_gettid)

class AppsActuator : public rclcpp::Node
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

        /* ROS2 Parameters */
        std::string m_sSpiInterface;
        std::string m_sTopic;
        uint32_t m_nSpiSpeed;

        int m_nIODevice, m_nPeriod;
        uint8_t m_nMode = 0, m_nBits = 8;
        uint16_t m_nDelay = 0;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subAppsActuator;
        
        void loadParameters();
        void openSpiDevice();
        void configureEDFScheduler();

        void appsActuatorPercentageCallback(const std_msgs::msg::Int8::SharedPtr appsTarget);

    public:

        AppsActuator();
        ~AppsActuator();

        void timerCallback();
};