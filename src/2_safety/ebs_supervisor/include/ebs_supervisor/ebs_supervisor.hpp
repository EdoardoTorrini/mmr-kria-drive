#pragma once

#include <mmr_kria_base/edf_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <std_msgs/msg/int8.hpp>
#include <can_msgs/msg/frame.hpp>

#include <sys/syscall.h>
#include <linux/sched.h>
#include <sched.h>

class EBSSupervisor : public EDFNode
{
    
    private:

        std::string m_sTopicEBS, m_sTopicCANTx;
        int m_nPeriod, m_nWCET, m_nDeadline;
        bool m_bDebug;
    
        /* value from ECU */
        float m_fPressionBrakeRear, m_fPressionBrakeFront, m_fEBSPression1, m_fEBSPression2;

        /* status of the RES */
        bool m_bEmergency, m_bGoSignal, m_bBagSignal;

        /* status of autonomous mission */
        uint8_t m_nMission;

        void loadParameters();

        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subCANBusMsg;
        void msgCANBusCallback(const can_msgs::msg::Frame::SharedPtr canbus_msg);

        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr m_pubEBSStatus;

    public:

        EBSSupervisor();
        ~EBSSupervisor() { RCLCPP_INFO(this->get_logger(), "CLOSING!"); };

        void debugCheckValueFromCAN();
};