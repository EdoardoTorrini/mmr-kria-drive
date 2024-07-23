#pragma once

#include <rclcpp/rclcpp.hpp>
#include "mmr_kria_base/edf_setup.hpp"

class EDFNode : public rclcpp::Node
{
    protected:
        void configureEDFScheduler(int period_ns, int runtime_ns, int deadline_ns);

        EDFNode(std::string name);
        ~EDFNode() {};
};
