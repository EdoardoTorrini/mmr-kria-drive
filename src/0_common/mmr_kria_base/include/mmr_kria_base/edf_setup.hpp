#pragma once

#include <linux/sched.h>
#include <linux/types.h>
#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <string.h>
#include <cstdint>

#define gettid() syscall(SYS_gettid)

// Necessary for EDF scheduling
struct sched_attr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;
    int32_t sched_nice;
    uint32_t sched_priority;
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};

namespace RES {

    enum MMR_CAN_MSG_ID {
        MMR_RES_OPERATIONAL = 0x0,
        MMR_RES_STATUS = 0x191,
    };

};

namespace ECU {

    enum MMR_CAN_MSG_ID {

    };

};