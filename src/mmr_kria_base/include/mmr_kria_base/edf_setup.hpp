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

enum canBusStateMachine{
    R2D,
    TS,
    FINISHED,
    EMERGENCY,
    READY,
    DRIVING,
    OFF,
    ASB_CHECK,
    RES_EMERGENCY,
    EBS_FLAG,
    ASB_ENGAGED,
    VEHICLE_STANDSTILL,
    STATE,
    ACTUATORS_OK
};

enum mmrAsState {
    MMR_AS_OFF,
    MMR_AS_READY,
    MMR_AS_DRIVING,
    MMR_AS_EMERGENCY,
    MMR_AS_FINISHED
};

enum MissionState {
    MISSION_SELECTE,
    MISSION_READY,
    MISSION_FINISHED
};

// Mission
enum MmrMission {
    MMR_MISSION_IDLE = 0,
    MMR_MISSION_ACCELERATION,
    MMR_MISSION_SKIDPAD,
    MMR_MISSION_AUTOCROSS,
    MMR_MISSION_TRACKDRIVE,
    MMR_MISSION_EBS_TEST,
    MMR_MISSION_INSPECTION,
    MMR_MISSION_MANUAL,
    MMR_MISSION_DEBUG
};

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