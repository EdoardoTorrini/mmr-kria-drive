#pragma once

#include <stdint.h>
#include <cstring>

#include <linux/can.h>
#include <algorithm>
#include <type_traits>

namespace CANOpen {

    const int header_size = 8, max_data_len = 4;
    const int sdo_upload_header = 64, sdo_download_header[max_data_len] = {47, 43, 39, 38};

    template<typename Base, typename T>
    static inline bool instanceof(const T) { return std::is_base_of<Base, T>::value; }

    struct canopen_frame {
        
        uint32_t id;
        uint8_t header;
        uint16_t index;
        uint8_t subindex;
        uint8_t data[max_data_len];

        canopen_frame& operator=(const canopen_frame& other) {
            if (this == &other)
                return *this;

            id = other.id;
            header = other.header;
            index = other.index;
            subindex = other.subindex;
            memcpy(&data, other.data, max_data_len * sizeof(uint8_t));

            return *this;
        }

        canopen_frame& operator=(const can_frame& other) {
            id = other.can_id;
            header = other.data[0];
            index = other.data[2] << 8 | other.data[1];
            subindex = other.data[3];
            memcpy(&data, &other.data[4], max_data_len * sizeof(uint8_t));

            return *this;
        }

        can_frame copy(int len) {
            can_frame frame;

            frame.can_id = this->id;
            frame.len = CAN_MAX_DLEN;

            memset(frame.data, '\0', CAN_MAX_DLEN);
            frame.data[0] = this->header;
            frame.data[1] = (uint8_t)this->index;
            frame.data[2] = (uint8_t)(this->index >> 8);
            frame.data[3] = this->subindex;

            memcpy(frame.data+4, this->data, max_data_len * sizeof(uint8_t));
            std::reverse_copy(this->data, this->data+len, frame.data+4);
            return frame;
        }

    };

    static uint8_t create_sdo_download_header(int payload_len) {
        if ((payload_len < 1) || (payload_len > 4)) return -1;
        return sdo_download_header[payload_len - 1]; 
    }

};