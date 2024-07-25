#pragma once

#include <mmr_kria_base/edf_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <can_msgs/msg/frame.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>

#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>

class CANBusBridge : public EDFNode
{

    private:

        std::string m_sInterface, m_sTopicTx, m_sTopicRx;
        int m_nBitrate, m_nMaxMsgs;
        bool m_bDebug;

        void loadParameters();

        /* Subscriber for CANBus Msg */
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subCANRx;
        void msgCANBusRxCallback(const can_msgs::msg::Frame::SharedPtr msg);

        /* Publisher for CANBus Msg */
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_pubCANBusTx;

        int m_nSocket;
        struct ifreq m_ifr;
        struct sockaddr_can m_addr;

        void connectCANBus();

    public:

        CANBusBridge();
        ~CANBusBridge() { close(this->m_nSocket); };

        void readMsgFromCANBus();

};