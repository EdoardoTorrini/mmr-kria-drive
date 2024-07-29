#pragma once

#include <canopen_bridge/driver/steer_maxon.hpp>
#include <canopen_bridge/driver/brake_maxon.hpp>

#include <mmr_edf/edf_node.hpp>
#include <mmr_kria_base/configuration.hpp>
#include <mmr_kria_base/msg/cmd_motor.hpp>
#include <mmr_kria_base/msg/actuator_status.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <can_msgs/msg/frame.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <sys/socket.h>

#include <string.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <math.h>

class CANOpenBridge : public EDFNode
{
    private:

        std::string m_sInterface;
        int m_nBitrate, m_nTimeoutMsg;
        bool m_bDebug;
        

        std::string m_sSteerTopic, m_sBrakeTopic, m_sClucthTopic, m_sStatusActuatorTopic, m_sCanTopic;

        /* Steer parameters */
        int m_nSteerID, m_nVelocity;
        float m_fWheelRate, m_fIncPerDegree, m_fMaxTarget;

        /* Brake parameters */
        int m_nBrakeId, m_nMaxTorque, m_nReturnPedalTorque;

        /* Clutch parameters */
        int m_nClutchId, m_nVelocityClutch;
        float m_fClutchPot;

        /* Subscriber for CANOpen Command Msg */
        rclcpp::Subscription<mmr_kria_base::msg::CmdMotor>::SharedPtr m_subCmdSteer;
        void msgCmdSteerCallback(mmr_kria_base::msg::CmdMotor::SharedPtr msg);

        rclcpp::Subscription<mmr_kria_base::msg::CmdMotor>::SharedPtr m_subCmdBrake;
        void msgCmdBrakeCallback(mmr_kria_base::msg::CmdMotor::SharedPtr msg);

        rclcpp::Subscription<mmr_kria_base::msg::CmdMotor>::SharedPtr m_subCmdClutch;
        void msgCmdClutchCallback(mmr_kria_base::msg::CmdMotor::SharedPtr msg);

        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr m_subCanCallback;
        void msgCanCallback(can_msgs::msg::Frame::SharedPtr msg);

        mmr_kria_base::msg::ActuatorStatus m_asActuatorStatus;
        rclcpp::Publisher<mmr_kria_base::msg::ActuatorStatus>::SharedPtr m_pubCANBusTx;

        int m_nSocket;
        struct ifreq m_ifr;
        struct sockaddr_can m_addr;

        void connectCANBus();
        void loadParameters();

        MaxonSteer *m_mSteer = nullptr;
        MaxonBrake *m_mBrake = nullptr;

    public:

        CANOpenBridge();
        ~CANOpenBridge() { close(this->m_nSocket); };

};