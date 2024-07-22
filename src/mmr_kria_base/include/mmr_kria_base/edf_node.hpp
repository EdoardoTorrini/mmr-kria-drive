#pragma once

#include <rclcpp/rclcpp.hpp>
#include "mmr_kria_base/edf_setup.hpp"

class EDFNode : public rclcpp::Node
{
private:
    int period_ns;
    int runtime_ns;
    int deadline_ns;
public:
    int configureEDFScheduler(int period_ns, int runtime_ns, int deadline_ns);
    EDFNode(std::string name);
    ~EDFNode();
};
