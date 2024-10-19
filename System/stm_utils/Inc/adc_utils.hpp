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
	 uint16_t moving_average(mov_avrg_filter& filter, uint16_t new_sample);
	float get_compensated_adc(uint16_t val, adc_names name);
	 float get_temperature();

	uint16_t ADC_sampled_data[5];
	uint16_t ADC_filtered_data[5];

	bool calibration_ready = false;

private :

	// Private functions
	void set_gains_offsets();

	// ADC definers

	ADC_HandleTypeDef* adc_cap_;
	ADC_HandleTypeDef* adc_bat_;
	ADC_HandleTypeDef* adc_temp_;

	// ADC sampled data



	// Moving average parameters

	mov_avrg_filter filters[5];

	// PID parameters

	float P_KP_chassis, P_KI_chassis, P_KD_chassis;
	float P_KP_battery, P_KI_battery, P_KD_battery;
	float I_KP_supercap, I_KI_supercap, I_KD_supercap;
	float V_KP_capMax, V_KI_capMax, V_KD_capMax;
	float V_KP_capMin, V_KI_capMin, V_KD_capMin;

	float adc_gains[5];
	float adc_offsets[5];

};
#endif /* STM_UTILS_INC_ADC_UTILS_HPP_ */
