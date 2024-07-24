#pragma once

#include <rclcpp/rclcpp.hpp>
#include "mmr_kria_base/edf_setup.hpp"

#include <fstream>
#include <string>

class EDFNode : public rclcpp::Node
{

    enum class GPIO_VALUE {
        DOWN = 0,
        UP,
    };

    protected:

        int m_nWCET, m_nPeriod, m_nDeadline;
        
        /* function to configure the node to use EDF - SCHED_DEALINE */
        void configureEDFScheduler(int period_ns, int runtime_ns, int deadline_ns);

        /* driver for write on GPIO using file descriptor */
        bool readGPIOValueFromFile(std::string sFile);
        void writeGPIOValueOnFile(std::string sFile, GPIO_VALUE nValue);

        EDFNode(std::string name);
        ~EDFNode() {};
};
