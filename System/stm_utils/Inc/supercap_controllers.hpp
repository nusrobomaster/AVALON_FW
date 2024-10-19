/*
 * supercap_controllers.hpp
 *
 *  Created on: Jul 6, 2024
 *      Author: YassineBakkali
 */

#ifndef STM_UTILS_INC_SUPERCAP_CONTROLLERS_HPP_
#define STM_UTILS_INC_SUPERCAP_CONTROLLERS_HPP_

#include "adc_utils.hpp"
#include "hrtim.h"

typedef enum
{
	firmwareErr,
	canErr,
	tempErr,
	INA240calErr,
	voltageErr,
	currentErr,
	dpErr,
	DMAErr
}
Safety_DetectItemTypedef;

typedef enum
{
	safe,
	warning,
	wtf,
	runAway,
	itsJoever
}
safetyLevelDef;

typedef enum
{
	silent,
	normal,
	charging
}
Mode_ModeTypedef;

class supercap_control_manager : public adc_manager{
public:

	supercap_control_manager(ADC_HandleTypeDef* adc_cap, ADC_HandleTypeDef* adc_bat, ADC_HandleTypeDef* adc_temp) :
		adc_manager(adc_cap, adc_bat, adc_temp){};

	~supercap_control_manager(){};

	 HAL_StatusTypeDef init_loop();
	void setup_pid_controllers();
	void set_ref_chassis_power(float ref_power_);

	  void loop_update(float Icap_, float Ibat_, float Vcap_, float Ichassis_, float Vbat_);
	  HAL_StatusTypeDef stop_loop();

	  HAL_StatusTypeDef start_gates_pwm(float init_duty_cycle);
	  HAL_StatusTypeDef stop_gates_pwm();

	 void all_safety_checks();
	 void silent_mode();

	 safetyLevelDef safetyLevel;
	 uint8_t safetyItemLevel[8];
	 uint8_t safetyChangeTrigger;
	 uint8_t current_mode = 1;

	// Switch PWMs duty cycles
	  void update_dutyCycle(float dutyCycle);

private:

	// PID getters
	  float get_PID(LoopCtrl_PID* pid_struct, float ref, float feedback, float ff_model);
	  float get_PID_FF(LoopCtrl_PID* pid_struct, float ref, float feedback, float ff_model);
	  void update_pid_maxpow();


	// Safety functions

	 void safety_check_voltages();
	 void safety_check_currents();
	 void safety_check_powers();
	 void safety_check_temp();
	 void safety_check_dma();

	 void safety_change_level(uint8_t Item, uint8_t Level);
	// Safety parameters

	uint16_t Vbat_counter = 0;
	uint16_t Vcap_counter = 0;
	uint16_t temp_counter = 0;
	uint16_t I_cap_counter = 0;
	uint16_t I_chassis_counter = 0;
	uint16_t P_ref_counter = 0;
	uint16_t P_ref_force_counter = 0;
	uint16_t I_cap_force_counter = 0;
	uint16_t DMA_counter = 0;

	uint8_t Vbat_status = 1;
	uint8_t Vcap_status = 1;
	uint8_t temp_status = 1;
	uint8_t is_init = false;
	uint16_t I_cap_status = 1;
	uint16_t I_chassis_status = 1;
	uint16_t P_ref_status = 1;

	// DSP PID instances and parameters

	LoopCtrl_PID pid[5];
	float I_supercap_last = 0.0f;
	float ref_power = 60.0f;
	bool in_loop = false;
};


#endif /* STM_UTILS_INC_SUPERCAP_CONTROLLERS_HPP_ */
