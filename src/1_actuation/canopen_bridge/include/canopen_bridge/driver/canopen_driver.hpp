#include <canopen_bridge/driver/canopen.hpp>
#include "mmr_kria_base/edf_setup.hpp"

class MaxonMotor
{
    private:

        int m_nSocket, m_nNodeID, m_nTimeOutMsg;
        int m_nState;

    public:

        MaxonMotor(int nSocket, int nNodeID, int nTimeOutMsg);
        ~MaxonMotor() {};

        int getState() { return this->m_nState; }
        
        int sendMsgOverCANBus(CANOpen::canopen_frame cof, CANOpen::canopen_frame* rcv, int len);

        template<typename T>
        int download(uint16_t nIndex, uint8_t nSubIndex, T value) {
            CANOpen::canopen_frame tx, rx;
            if ((CANOpen::instanceof<double>(value)) || (CANOpen::instanceof<long long int>(value)))
                return -1;
            
            tx.id = MOTOR::REQUEST_SDO + this->m_nNodeID;
            tx.header = CANOpen::create_sdo_download_header(sizeof(T));
            tx.index = nIndex;
            tx.subindex = nSubIndex;

            memset(tx.data, 0, CANOpen::max_data_len * sizeof(uint8_t));
            memcpy(tx.data, &value, sizeof(T));

            if (sendMsgOverCANBus(tx, &rx, sizeof(T)) < 0) return -1;

            // TODO: gestire nel caso in cui (rx->header >> (header_size - 1)) != 0
            //       è un errore -> cambiare stato del motore dello sterzo

            return 0;
        }

        template<typename T>
        T upload(uint16_t nIndex, uint8_t nSubIndex) {
            T res;
            CANOpen::canopen_frame tx, rx;

            tx.id = MOTOR::REQUEST_SDO + this->m_nNodeID;
            tx.header = CANOpen::sdo_upload_header;
            tx.index = nIndex;
            tx.subindex = nSubIndex;

            if (sendMsgOverCANBus(tx, &rx, 0) < 0) return -1;

            // TODO: gestire nel caso in cui (rx->header >> (header_size - 1)) != 0
            //       è un errore -> cambiare stato del motore dello sterzo

            if (sizeof(T) != (CANOpen::max_data_len - (rx.header >> 2) & 3)) return -1;
            memcpy(&res, rx.data, sizeof(T));
            return res;
        }
};