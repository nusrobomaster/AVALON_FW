/*
 * dummy_thread.cpp
 *
 *  Created on: Feb 13, 2024
 *      Author: Yassine
 */

#include <SuperCapCommThread.h>
#include <supercap_def.h>
#include <Telemetry.h>

SuperCapCommThread* SuperCapCommInstance = nullptr;

extern float chassis_voltage;
extern float cap_voltage;
extern float source_current;
extern uint8_t max_chassis_power;

SuperCapCommThread::~SuperCapCommThread(){
}

void SuperCapCommThread::init(){
	;;
}

// Declare your data with the proper data structure defined in DataStructures.h
static SuperCapData supercap_data;


// Declare the RoCo packet with the proper data structure defined in RoCo/Src/Protocol/Protocol24
static SuperCapDataPacket supercap_packet;
static int i = 0;

void SuperCapCommThread::loop()
{
	supercap_data.V_cap = cap_voltage;
	supercap_data.P_chassis = chassis_voltage * source_current;
	supercap_data.charge_state = (uint8_t)(((cap_voltage - 2.95f) / (SUPERCAP_MAX_VOLTAGE - 2.95f))*100);

	supercap_data.toArray((uint8_t*) &supercap_packet);

	MAKE_IDENTIFIABLE(supercap_packet);
	MAKE_RELIABLE(supercap_packet);
	Telemetry::set_id(OTHER_NODE_ID);

	FDCAN3_network->send(&supercap_packet);

	osDelay(100);

	portYIELD();
}

void SuperCapCommThread::handle_max_power(uint8_t sender_id, MaxChassisPowerPacket* packet) {
	if(!(IS_RELIABLE(*packet))) {
//		console.printf_error("Unreliable IMU calibration packet");
		return;
	}
	int temp_val = packet->max_chassis_power;
	if(temp_val < 50)
		temp_val = 50;
	max_chassis_power = temp_val;
}


