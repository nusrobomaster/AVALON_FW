/**
 * @file msg_handler.hpp
 * @brief CAN communication handler for sending and receiving control messages related to the supercapacitor power module.
 *
 * This header defines the message packet structures and the CANComm class that abstracts transmission and reception
 * of messages using STM32's FDCAN peripheral.
 *
 * Created on: Jan 20, 2025
 * Author: Yassine Bakkali
 */

#ifndef STM_UTILS_INC_MSG_HANDLER_HPP_
#define STM_UTILS_INC_MSG_HANDLER_HPP_

#include <cstdint>   ///< Standard integer types
#include <fdcan.h>   ///< FDCAN peripheral definitions

/**
 * @brief CAN ID for receiving command messages intended for this board.
 */
#define CAN_NODE_ID 0x067

/**
 * @brief CAN ID used by this board to send messages to others.
 */
#define DEVC_NODE_ID 0x077

/**
 * @struct ref_msg_packet
 * @brief Incoming reference message structure received over CAN.
 *
 * This message typically originates from the robot's main controller and is used
 * to configure and control the supercap module in real-time.
 */
struct ref_msg_packet {
	uint8_t enable_module;    ///< Enable/disable flag for supercap control loop
	uint8_t reset;            ///< If set, triggers a software reset
	uint8_t pow_limit;        ///< Maximum power budget (W)
	uint16_t energy_buffer;   ///< Optional buffer value to apply limiting strategy
} __attribute__((packed));

/**
 * @struct supercap_msg_packet
 * @brief Outgoing status message structure sent over CAN.
 *
 * This message is periodically sent from the supercap board to the main controller
 * and contains real-time power and error data.
 */
struct supercap_msg_packet {
	float chassis_power;      ///< Computed instantaneous power drawn by chassis
	uint8_t error;            ///< Error code or flag (0 = OK)
	uint8_t cap_energy;       ///< Supercap charge percentage (0–255 mapped)
} __attribute__((packed));

/**
 * @brief Initializes CAN communication: configures filters and starts the FDCAN peripheral.
 * @param fdcan_ Pointer to the FDCAN handle used (typically &hfdcan3).
 */
void initCANComm(FDCAN_HandleTypeDef* fdcan_);

/**
 * @class CANComm
 * @brief Abstraction class to manage CAN communication for the supercap board.
 *
 * This class handles CAN transmission of status messages and provides the scaffolding
 * for receiving command/control messages via interrupt.
 */
class CANComm {
public:
    /**
     * @brief Constructor: sets up header configuration for outgoing messages.
     * @param fdcan_ Pointer to FDCAN handle.
     */
    CANComm(FDCAN_HandleTypeDef* fdcan_) : fdcan(fdcan_) { txHeaderConfig(); }

    /**
     * @brief Destructor (does nothing but defined for completeness).
     */
    ~CANComm() {}

    /**
     * @brief Sends the current `supercap_msg_packet` status over CAN.
     */
    void sendMessage();

private:
    /**
     * @brief Configures the TxHeader structure for FDCAN transmission.
     */
    void txHeaderConfig();

    /**
     * @brief Placeholder for internal interrupt handling (currently unused).
     */
    void handleInterrupts();

    FDCAN_HandleTypeDef* fdcan;        ///< Pointer to active CAN peripheral

    FDCAN_RxHeaderTypeDef RxHeader;    ///< Receive header placeholder
	uint8_t* RxData;                   ///< Pointer to receive data (optional, not used yet)
	FDCAN_TxHeaderTypeDef TxHeader;    ///< Transmit header configuration

	supercap_msg_packet txMsg;         ///< Outgoing message payload

    int errorStatus;                   ///< Placeholder for CAN error status tracking
};

#endif /* STM_UTILS_INC_MSG_HANDLER_HPP_ */
