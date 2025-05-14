/**
 * @file msg_handler.cpp
 * @brief Handles CAN message reception, decoding, and response for supercap control.
 *
 * This module handles communication over FDCAN between the supercapacitor module and the main robot controller.
 * It allows receiving control signals (enable/disable, power limit updates) and transmits supercap status (power, energy).
 *
 * Created by: Yassine Bakkali
 * Date: Jan 20, 2025
 */


#include "msg_handler.hpp"

/// Local message object storing the latest received control packet.
static ref_msg_packet rxMsg;
extern float chassis_voltage;
extern float battery_current;
extern float cap_voltage;
extern float source_current;
float energy_buff;
extern uint8_t max_chassis_power;
uint8_t enable_supercap;

extern supercap_control_manager supercap_controller;


/**
 * @brief Initializes and configures the CAN peripheral and message filters.
 *
 * This function sets up the CAN ID filters, enables relevant interrupts,
 * configures delay compensation, and starts the FDCAN interface.
 *
 * @param fdcan_ Pointer to the FDCAN handle to initialize.
 */
void initCANComm(FDCAN_HandleTypeDef* fdcan_) {
	MX_FDCAN3_Init();

	FDCAN_FilterTypeDef can_filter;
	can_filter.IdType = FDCAN_STANDARD_ID;
	can_filter.FilterIndex = 0;
	can_filter.FilterType = FDCAN_FILTER_DUAL;
	can_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter.FilterID1 = CAN_NODE_ID;
	can_filter.FilterID2 = 0x7FF;
	HAL_FDCAN_ConfigFilter(fdcan_, &can_filter);

	can_filter.FilterIndex = 1;
	can_filter.FilterID1 = 0x7FF;
	can_filter.FilterID2 = 0x7FF;
	HAL_FDCAN_ConfigFilter(fdcan_, &can_filter);

	HAL_FDCAN_ConfigGlobalFilter(fdcan_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(fdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(fdcan_, FDCAN_IT_BUS_OFF, 0);

	HAL_FDCAN_ConfigTxDelayCompensation(fdcan_, fdcan_->Init.DataPrescaler * fdcan_->Init.DataTimeSeg1, 0);
	HAL_FDCAN_EnableTxDelayCompensation(fdcan_);
	HAL_FDCAN_Start(fdcan_);
}


/**
 * @brief Configures the CAN transmit header for sending power/energy feedback.
 *
 * Sets data frame format, DLC size, bitrate switch, and device ID.
 */
void CANComm::txHeaderConfig() {
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
	TxHeader.DataLength = FDCAN_DLC_BYTES_6;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.Identifier = DEVC_NODE_ID;
}


/**
 * @brief Sends a CAN message containing supercap status.
 *
 * Computes instantaneous chassis power, normalizes capacitor voltage into an 8-bit energy buffer estimate,
 * and transmits it using FDCAN.
 */
void CANComm::sendMessage() {
	txMsg.chassis_power = chassis_voltage * battery_current;

	// Compute normalized energy percentage [0.0 - 1.0]
	float cap_buffer = 0.0f;
	if (cap_voltage > SUPERCAP_MIN_VOLTAGE) {
		if (cap_voltage >= SUPERCAP_MAX_VOLTAGE)
			cap_buffer = 1.0f;
		else
			cap_buffer = (cap_voltage - SUPERCAP_MIN_VOLTAGE) / (SUPERCAP_MAX_VOLTAGE - SUPERCAP_MIN_VOLTAGE);
	}

	txMsg.cap_energy = cap_buffer > 1.0f ? 255 : cap_buffer * 255;
	txMsg.error = 0;

	HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader, (uint8_t *)&txMsg);
}


/**
 * @brief Callback triggered when a new CAN message is received.
 *
 * If the message matches the expected node ID, it handles:
 *  - Enabling/disabling the supercap loop
 *  - Resetting the system
 *  - Updating the power limit and energy buffer reference
 *
 * @param hfdcan FDCAN peripheral instance
 * @param RxFifo0ITs Interrupt source bitmask
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (hfdcan == &hfdcan3 && RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
		FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
		HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, (uint8_t *)&rxMsg);

		if (FDCAN_RxHeader.Identifier == CAN_NODE_ID) {
			static bool last_enable = true;

			if (rxMsg.reset)
				softwareReset();

			if (!rxMsg.enable_module) {
				enable_supercap = false;
				supercap_controller.stop_loop();
			}
			else if (!last_enable) {
				enable_supercap = true;
				supercap_controller.start_loop();
			}
			last_enable = rxMsg.enable_module;

			max_chassis_power = rxMsg.pow_limit;
			supercap_controller.set_ref_chassis_power(max_chassis_power);
			energy_buff = rxMsg.energy_buffer;
		}
	}
}


/**
 * @brief Called on FDCAN error interrupt (e.g. bus-off state).
 *
 * Resets the CAN peripheral and reinitializes the communication settings.
 *
 * @param hfdcan Pointer to the affected CAN instance
 * @param ErrorStatusITs Interrupt flags (unused)
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
	MX_FDCAN3_Init();        // Reset CAN peripheral
	initCANComm(hfdcan);     // Reapply all settings and filters
}
