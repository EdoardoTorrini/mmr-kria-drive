#include "mmr_kria_base/edf_node.hpp"

EDFNode::EDFNode(std::string name) : Node(name) {}

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