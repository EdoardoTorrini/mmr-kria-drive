#pragma once

#include <mmr_kria_base/edf_node.hpp>
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

class AppsActuator : public EDFNode
{

    private:

        /* ROS2 Parameters */
        std::string m_sSpiInterface;
        std::string m_sTopic;
        uint32_t m_nSpiSpeed;

        int m_nIODevice, m_nPeriod, m_nWCET, m_nDeadline;
        uint8_t m_nMode = 0, m_nBits = 8;
        uint16_t m_nDelay = 0;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr m_subAppsActuator;
        
        void loadParameters();
        void openSpiDevice();

        void appsActuatorPercentageCallback(const std_msgs::msg::Int8::SharedPtr appsTarget);

    public:

        AppsActuator();
        ~AppsActuator();

        void timerCallback();
};