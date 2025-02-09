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
	voltageErr,
	currentErr,
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
	idle,
	normal,
}
Mode_ModeTypedef;

#define HRTIM_PERIOD 34000
#define HRTIM_HALF_PERIOD 17000

class supercap_control_manager : public adc_manager{
public:

	supercap_control_manager(ADC_HandleTypeDef* adc_cap, ADC_HandleTypeDef* adc_bat, ADC_HandleTypeDef* adc_temp) :
	adc_manager(adc_cap, adc_bat, adc_temp){};

	~supercap_control_manager(){};

	HAL_StatusTypeDef init_loop();
	void setup_pid_controllers();
	void set_ref_chassis_power(float ref_power_);

	void loop_update(float Icap_, float Ibat_, float Vcap_, float Ichassis_, float Vbat_);
	void stop_loop();
	HAL_StatusTypeDef start_loop();


	HAL_StatusTypeDef start_gates_pwm(float init_duty_cycle);

	void all_safety_checks();
	void silent_mode();

	safetyLevelDef safetyLevel;


private:

	float ref_power = 60.0f;
	};
void softwareReset();

extern uint8_t safetyItemLevel[8];
extern float chassis_voltage;
extern float battery_current;

#endif /* STM_UTILS_INC_SUPERCAP_CONTROLLERS_HPP_ */
