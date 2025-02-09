/*
 * msg_handler.hpp
 *
 *  Created on: Jan 20, 2025
 *      Author: yassine_bakkali
 */

#ifndef STM_UTILS_INC_MSG_HANDLER_HPP_
#define STM_UTILS_INC_MSG_HANDLER_HPP_


#include <cstdint> // For standard integer types
#include <fdcan.h>

#define CAN_NODE_ID 0x067
#define DEVC_NODE_ID 0x077

struct ref_msg_packet {
	uint8_t enable_module;
	uint8_t reset;
	uint8_t pow_limit;
	uint16_t energy_buffer;
} __attribute__((packed));

struct supercap_msg_packet {
	float chassis_power;
	uint8_t error;
	uint8_t cap_energy;

} __attribute__((packed));

void initCANComm(FDCAN_HandleTypeDef* fdcan_);

class CANComm {
public:
    // Constructor and Destructor
    CANComm(FDCAN_HandleTypeDef* fdcan_) : fdcan(fdcan_){txHeaderConfig();}
    ~CANComm(){}

    // Transmission
    void sendMessage();

    // Reception
    bool receiveMessage(uint32_t& id, uint8_t* data, uint8_t& length);


private:
    // Private helper methods
    void handleInterrupts();
    void txHeaderConfig();

    FDCAN_HandleTypeDef* fdcan;


    FDCAN_RxHeaderTypeDef RxHeader;
	uint8_t* RxData;
	FDCAN_TxHeaderTypeDef TxHeader;

	supercap_msg_packet txMsg;


    int errorStatus;
};



#endif /* STM_UTILS_INC_MSG_HANDLER_HPP_ */
