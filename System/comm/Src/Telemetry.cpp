/*
 * Telemetry.cpp
 *
 *  Created on: Dec 20, 2023
 *      Author: Yassine
 */

#include "Telemetry.h"
#include "dummy_thread.h"

#include "typedefs.h"

#include "tim.h"
#include "spi.h"
#include "fdcan.h"

#include <SuperCapCommThread.h>


ROCANDriver* FDCAN3_driver = nullptr;
CANBus* FDCAN3_network = nullptr;


dummyThread* dummy = nullptr;
SuperCapCommThread* SuperCapComm = nullptr;

extern gimbal_control_t gimbal_ctrl_data;

void Telemetry::setup() {
		// UART line(s) initialization
//		UART1_line = new STMUARTDriver(&huart1);
//		UART1_network = new NetworkBus(UART1_line);

		// CANFD network initialization
		FDCAN3_driver = new ROCANDriver(&hfdcan3, CURRENT_NODE_ID);
		FDCAN3_network = new CANBus(FDCAN3_driver);
		dummy = new dummyThread();
		SuperCapComm = new SuperCapCommThread();

		FDCAN3_network->handle<dummyPacket>(&dummyThread::handle_dummy);
		FDCAN3_network->handle<MaxChassisPowerPacket>(&SuperCapCommThread::handle_max_power);


}

void Telemetry::set_id(uint32_t id) {
	dynamic_cast<ROCANDriver*>(FDCAN3_network->get_driver())->TxHeaderConfigID(id);
//	dynamic_cast<ROCANDriver*>(FDCAN2_network->get_driver())->TxHeaderConfigID(id);
}
