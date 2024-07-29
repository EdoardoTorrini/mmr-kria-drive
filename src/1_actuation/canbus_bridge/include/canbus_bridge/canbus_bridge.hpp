#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <can_msgs/msg/frame.hpp>

#include <mmr_edf/edf_node.hpp>
#include <mmr_kria_base/msg/ecu_status.hpp>
#include <mmr_kria_base/msg/res_status.hpp>

#include <mmr_kria_base/configuration.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>

#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include <bit>
#include <algorithm>

class CANBusBridge : public EDFNode
{

    private:

        std::string m_sInterface, m_sTopicTx, m_sTopicRx, m_sEcuStatusTopic, m_sResStatusTopic;
        int m_nBitrate, m_nMaxMsgs;
        bool m_bDebug;

        void loadParameters();

        /* Subscriber for CANBus Msg */
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subCANRx;
        void msgCANBusRxCallback(const can_msgs::msg::Frame::SharedPtr msg);

        /* Publisher for CANBus Msg */
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr m_pubCANBusTx;
        rclcpp::Publisher<mmr_kria_base::msg::EcuStatus>::SharedPtr m_pubEcuStatus;
        rclcpp::Publisher<mmr_kria_base::msg::ResStatus>::SharedPtr m_pubResStatus;

        /* message for the pub */
        mmr_kria_base::msg::EcuStatus m_msgEcuStatus;
        mmr_kria_base::msg::ResStatus m_msgResStatus;

        int m_nSocket;
        struct ifreq m_ifr;
        struct sockaddr_can m_addr;

        void connectCANBus();
        void readEcuStatus(can_frame frame);
        void readResStatus(can_frame frame);

    public:

        CANBusBridge();
        ~CANBusBridge() { close(this->m_nSocket); };

        void readMsgFromCANBus();
        void sendStatusEcu() { if (this->m_pubEcuStatus != nullptr) this->m_pubEcuStatus->publish(this->m_msgEcuStatus); };
        void sendStatusRes() { if (this->m_pubResStatus != nullptr) this->m_pubResStatus->publish(this->m_msgResStatus); };
};