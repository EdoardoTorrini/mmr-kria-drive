#include "mmr_edf/mmr_edf.hpp"

void EDFNode::configureEDFScheduler(int period_ns, int runtime_ns, int deadline_ns) {

    // Set the scheduling policy to SCHED_DEADLINE
    struct sched_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime = runtime_ns;
    attr.sched_deadline = deadline_ns;
    attr.sched_period = period_ns;

    if (syscall(SYS_sched_setattr, gettid(), &attr, 0) != 0) {
        RCLCPP_ERROR(this->get_logger(), "[ EDF SCHEDULER ]: %s", strerror(errno));
        throw 1;
    }
}

bool EDFNode::readGPIOValueFromFile(std::string sFile)
{
    std::ifstream file(sFile);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED to OPEN FILE ]: %s", sFile.c_str());
        throw 1;
    }

    int nVal;
    file >> nVal;
    return nVal;
}

void EDFNode::writeGPIOValueOnFile(std::string sFile, GPIO_VALUE nValue)
{
    std::ofstream file(sFile);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED to OPEN FILE ]: %s", sFile.c_str());
        throw 1;
    }

    file << std::to_string((int)nValue);
    if (!file.good()) {
        RCLCPP_ERROR(this->get_logger(), "[ FAILED to OPEN FILE ]: %s", sFile.c_str());
        throw 1;
    }
}