#pragma once

namespace RES {

    enum MMR_CAN_MSG_ID {
        /* enable the RES in OpMode using payload = [01, 00, 00, 00, 00, 00, 00, 00] */
        MMR_RES_OPERATIONAL = 0x0,

        /* check the RES Status - [ bit 0 ]: EMERGENCY, [ bit 1 ]: GO_SIGNAL, [ bit 3 ]: BAG [ not used ] */
        MMR_RES_STATUS = 0x191,
    };

    enum MMR_RES_STATUS_MASK {
        RES_SIGNAL_EMERGENCY = 1,
        RES_SIGNAL_GO = 2,
        RES_SIGNAL_BAG = 4
    };

};

namespace ECU {

    enum MMR_CAN_MASK {
        MMR_ECU_MASK = 0x700,
    };

    enum MMR_CAN_MSG_ID {
        
        MMR_STEERING_ANGLE = 0x8A,
        MMR_BRAKING_PERCENTAGE,
        MMR_ACCELERATOR_PERCENTAGE = 0x8c,

        MMR_CLUTCH_PULL_OK = 0xE1,
        MMR_CLUTCH_RELEASE_OK = 0xE3,

        MMR_ECU_PEDAL_THROTTLE = 0x700,
        MMR_ECU_TEMPERATURES,
        MMR_ECU_ENGINE_FN1,
        MMR_ECU_PRESSURES,
        MMR_ECU_ENGINE_FN2,
        MMR_ECU_CLUTCH_STEER,
        MMR_ECU_WHEEL_SPEEDS,
        MMR_ECU_SAFETY_CHECK,
        MMR_ECU_EBS_PRESSURE,
        MMR_ECU_SET_LAUNCH_CONTROL,

        MMR_ECU_GEAR_CONTROL = 0x610,
        MMR_ECU_SET_PIT_LAUNCH = 0x628

    };

};

namespace COCKPIT {

    enum MMR_CAN_MSG_ID {
        MMR_MISSION_SELECTED = 0x40,
    };

    enum MMR_MISSION_VALUE {
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

};

namespace MOTOR {

    enum MMR_CAN_ID_BASE {
        REQUEST_SDO = 0x600,
        RESPONSE_SDO = 580,
    };

    enum ACTUATOR_STATUS {
        DISABLE = 0,
        ENABLE,
        POSITION_MODE,
        TORQUE_MODE,
        ERROR,
    };

    enum MODE_OF_OPERATION {
        HMM = 0x06,
        PPM = 0x01,
        PVM = 0x03,
        CSP = 0x08,
        CSV = 0x09,
        CST = 0x0A,
    };

};