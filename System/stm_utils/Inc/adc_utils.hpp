/*
 * adc_utils.hpp
 *
 *  Created on: Jul 5, 2024
 *      Author: YassineBakkali
 */

#ifndef STM_UTILS_INC_ADC_UTILS_HPP_
#define STM_UTILS_INC_ADC_UTILS_HPP_

#include <adc.h>
#include <dma.h>
#include <stdint.h>
#include <arm_math.h>
#include "supercap_def.h"
#include "main.h"

// Offsets were obtained through zero-point calibration

//2^12, check datasheet.
#define ADC_RES 4095.0f

// (R1+R2)/R2, values can be found on the schematics.
#define INV_VOLT_DIV 11.0f
#define ADC_REF 3.3f

#define INA186A2_GAIN 50.0f //Check datasheet.
#define SHUNT_RESISTOR 0.004f // Check schematics.

#define VCAP_GAIN ADC_REF*INV_VOLT_DIV/ADC_RES
#define VBAT_GAIN ADC_REF*INV_VOLT_DIV/ADC_RES
// Current going from the PMM to the load (Supercap/Chassis etc...)
#define IBAT_GAIN -(ADC_REF/SHUNT_RESISTOR/INA186A2_GAIN/ADC_RES)
// Current flowing out of the Supercap on the battery's side.
#define ISOURCE_GAIN -(ADC_REF/SHUNT_RESISTOR/INA186A2_GAIN/ADC_RES)
// Current flowing into the Sueprcap bank on the side of the Caps.
#define ICAP_GAIN -(ADC_REF/SHUNT_RESISTOR/INA186A2_GAIN/ADC_RES)

#if BOARD_ID == 0x01
	#define VCAP_OFFSET 0.4914794921875f
	#define VBAT_OFFSET 0.5060302734375f
	#define IBAT_OFFSET 8.018306816f
	#define ISOURCE_OFFSET 8.0183068822f
	#define ICAP_OFFSET 7.97801410408f
#elif BOARD_ID == 0x02
	#define VCAP_OFFSET 0.5300927721875f
	#define VBAT_OFFSET 0.6178282734375f
	#define IBAT_OFFSET 7.9417500291f
	#define ISOURCE_OFFSET 7.9941314767f
	#define ICAP_OFFSET 7.95786746968f
#elif BOARD_ID == 0x03
	#define VCAP_OFFSET 0.5344686121875f
	#define VBAT_OFFSET 0.5866657734375f
	#define IBAT_OFFSET 8.018306816f
	#define ISOURCE_OFFSET 8.0183068822f
	#define ICAP_OFFSET 7.97801410408f
#elif BOARD_ID == 0x04
	#define VCAP_OFFSET 0.5344686121875f
	#define VBAT_OFFSET 0.5866657734375f
	#define IBAT_OFFSET 8.018306816f
	#define ISOURCE_OFFSET 8.0183068822f
	#define ICAP_OFFSET 7.97801410408f
#else
	#error "WRONG BOARD ID."
#endif

#define ADC1_CHANNELS 2
#define ADC2_CHANNELS 3
#define ADC_FIRST_BUFFER 0
#define ADC_SECOND_BUFFER 1
#define ADC_BUFFER_DEPTH 16

typedef enum {
	I_cap, V_cap,
	V_bat, I_chassis, I_bat
} adc_names;

typedef enum {
	P_bat, P_chassis,
	I_capa, V_cap_max, V_cap_min
} pid_names;

typedef struct {
	uint16_t window_filter[64];
	uint8_t  length;
	uint8_t  index;
	uint32_t sum;
	uint8_t full;
} mov_avrg_filter ;

typedef struct
{
	arm_pid_instance_f32 ArmPID_Instance;
	float IntegMax;
	float IntegMin;
	float OutputMax;
	float OutputMin;
}
LoopCtrl_PID;



class adc_manager {
public :
	adc_manager(ADC_HandleTypeDef* adc_cap, ADC_HandleTypeDef* adc_bat, ADC_HandleTypeDef* adc_temp) :
		adc_cap_(adc_cap), adc_bat_(adc_bat), adc_temp_(adc_temp) {};

	HAL_StatusTypeDef adc_init(uint8_t* filters_length);
	HAL_StatusTypeDef sample_adc();
	float get_temperature();
	bool calibration_ready = false;

private :

	// Private functions
	void set_gains_offsets();

	// ADC definers
	ADC_HandleTypeDef* adc_cap_;
	ADC_HandleTypeDef* adc_bat_;
	ADC_HandleTypeDef* adc_temp_;

	// PID parameters
	float P_KP_chassis, P_KI_chassis, P_KD_chassis;
	float P_KP_battery, P_KI_battery, P_KD_battery;
	float I_KP_supercap, I_KI_supercap, I_KD_supercap;
	float V_KP_capMax, V_KI_capMax, V_KD_capMax;
	float V_KP_capMin, V_KI_capMin, V_KD_capMin;

};

extern uint16_t adc1_samples[2][16][2];
extern uint16_t adc2_samples[2][16][3];
extern uint16_t ADC_filtered_data[5];
extern mov_avrg_filter filters[5];
extern float adc_gains[5];
extern float adc_offsets[5];

#define GET_COMPENSATED_ADC(ADC_VAL, NAME) (((float)(ADC_VAL)) * adc_gains[NAME]) + adc_offsets[NAME]


#endif /* STM_UTILS_INC_ADC_UTILS_HPP_ */
