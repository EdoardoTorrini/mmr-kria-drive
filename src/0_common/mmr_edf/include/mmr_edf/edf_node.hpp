#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sys/syscall.h>
#include <linux/sched.h>
#include <sched.h>

#include <linux/types.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <cstdint>

#include <bit>
#include <algorithm>
#include <array>

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

        EDFNode(std::string sName) : rclcpp::Node(sName) {};
    
    public:
        
        template <typename T, std::endian Endianness = std::endian::little>
        static T endian_cast(const uint8_t* bytes) {
            if (std::endian::native == Endianness)
                return *(const T*)bytes;
            else {
                std::array<uint8_t, sizeof(T)> buf;
                std::copy(bytes, bytes + sizeof(T), buf.rbegin());
                return *(const T*)buf.data();
            }
        }
};
