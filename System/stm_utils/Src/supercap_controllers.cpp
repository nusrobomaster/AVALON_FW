/*
 * supercap_controllers.cpp
 *
 *  Created on: Jul 6, 2024
 *      Author: YassineBakkali
 */

#include "supercap_controllers.hpp"

float Pchassis_output;
float icap_output;
float duty_ratio_output;
float out_duty_ratio_output;
uint8_t max_chassis_power = 45;

float temperature = 0.0f;
float duty_ratio = 0.9f;
float cap_voltage = 0.0f;
float chassis_voltage = 0.0f;
float cap_current = 0.0f;
float battery_current = 0.0f;
//float source_current = 0.0f;

static uint16_t ADC_sampled_data[5];

static uint8_t is_init = false;
static uint8_t in_loop = false;
static uint8_t safetyChangeTrigger;
static LoopCtrl_PID pid[5];

static float I_supercap_last = 0.0f;

Mode_ModeTypedef current_mode = normal;
uint8_t safetyItemLevel[8];

#define ALPHA_PID_POWER 0.98f

static inline float low_pass_filter(float current_value, float previous_value, float alpha);

static inline float low_pass_filter(float current_value, float previous_value, float alpha) {
    return alpha * previous_value + (1 - alpha) * current_value;
}

HAL_StatusTypeDef supercap_control_manager::init_loop(){
	setup_pid_controllers();
	return HAL_OK;
}
static inline void update_pid_maxpow();

static inline void update_pid_maxpow(){
	pid[0].IntegMax = max_chassis_power-3;
	pid[0].OutputMax = max_chassis_power-3;
}

void supercap_control_manager::setup_pid_controllers(){
	pid[0].ArmPID_Instance.Kp = P_BATTERY_KP;
	pid[0].ArmPID_Instance.Ki = P_BATTERY_KI;
	pid[0].ArmPID_Instance.Kd = P_BATTERY_KD;

	pid[0].IntegMax = max_chassis_power-3;
	pid[0].OutputMax = max_chassis_power-3;

	pid[0].IntegMin = P_BATTERY_MIN_OUTPUT;
	pid[0].OutputMin = P_BATTERY_MIN_OUTPUT;

//	pid[1].ArmPID_Instance.Kp = P_CHASSIS_KP;
//	pid[1].ArmPID_Instance.Ki = P_CHASSIS_KI;
//	pid[1].ArmPID_Instance.Kd = P_CHASSIS_KD;

	pid[2].ArmPID_Instance.Kp = I_SUPERCAP_KP;
	pid[2].ArmPID_Instance.Ki = I_SUPERCAP_KI;
	pid[2].ArmPID_Instance.Kd = I_SUPERCAP_KD;

	pid[2].IntegMax = I_SUPERCAP_INTEG_MAX;
	pid[2].IntegMin = I_SUPERCAP_INTEG_MIN;
	pid[2].OutputMax = I_SUPERCAP_MAX_OUTPUT;
	pid[2].OutputMin = I_SUPERCAP_MIN_OUTPUT;

	pid[3].ArmPID_Instance.Kp = V_SUPERCAPMAX_KP;
	pid[3].ArmPID_Instance.Ki = V_SUPERCAPMAX_KI;
	pid[3].ArmPID_Instance.Kd = V_SUPERCAPMAX_KD;

	pid[3].IntegMax = V_SUPERCAPMAX_INTEG_MAX;
	pid[3].IntegMin = V_SUPERCAPMAX_INTEG_MIN;
	pid[3].OutputMax = V_SUPERCAPMAX_MAX_OUTPUT;
	pid[3].OutputMin = V_SUPERCAPMAX_MIN_OUTPUT;

	pid[4].ArmPID_Instance.Kp = V_SUPERCAPMIN_KP;
	pid[4].ArmPID_Instance.Ki = V_SUPERCAPMIN_KI;
	pid[4].ArmPID_Instance.Kd = V_SUPERCAPMIN_KD;

	pid[4].IntegMax = V_SUPERCAPMIN_INTEG_MAX;
	pid[4].IntegMin = V_SUPERCAPMIN_INTEG_MIN;
	pid[4].OutputMax = V_SUPERCAPMIN_MAX_OUTPUT;
	pid[4].OutputMin = V_SUPERCAPMIN_MIN_OUTPUT;

	arm_pid_init_f32(&(pid[0].ArmPID_Instance), 1);
	pid[0].ArmPID_Instance.state[2] = pid[0].IntegMax;

//	arm_pid_init_f32(&(pid[1].ArmPID_Instance), 1);

	arm_pid_init_f32(&(pid[2].ArmPID_Instance), 1);

	HAL_Delay(100);
	float initDutyRatio = 15.0;
	while (initDutyRatio >= 2 or initDutyRatio <= 0){
		volatile float cap_temp = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);
		volatile float bat_temp = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], V_bat);
		initDutyRatio = cap_temp / bat_temp;
	}

	pid[2].ArmPID_Instance.state[2] = initDutyRatio;

	arm_pid_init_f32(&(pid[3].ArmPID_Instance), 1);
	pid[3].ArmPID_Instance.state[2] = pid[3].IntegMax;

	arm_pid_init_f32(&(pid[4].ArmPID_Instance), 1);
	pid[4].ArmPID_Instance.state[2] = pid[4].IntegMin;

	if(!CALIBRATION_MODE){
		HAL_StatusTypeDef status = start_gates_pwm(initDutyRatio);
		is_init = true;
		in_loop = true;
	}
}

static inline uint16_t moving_average(mov_avrg_filter& filter, uint16_t new_sample);

static inline uint16_t moving_average(mov_avrg_filter& filter, uint16_t new_sample){
	filter.sum -= filter.window_filter[filter.index];
	filter.sum += new_sample;
	filter.window_filter[filter.index] = new_sample;

	filter.index += 1;
	if(filter.index == filter.length){
		filter.index = 0;
		filter.full = true;
	}
	if(filter.full)
		return (uint16_t)(filter.sum/filter.length);
	else
		return (uint16_t)(filter.sum/filter.index);

}


static inline void sample_adc();

static inline void sample_adc(){
	uint8_t Cap_HalfFull, Bat_HalfFull;
	if(__HAL_DMA_GET_FLAG(hadc1.DMA_Handle, DMA_FLAG_HT1) == SET){
		__HAL_DMA_CLEAR_FLAG(hadc1.DMA_Handle, DMA_FLAG_HT1);
		Cap_HalfFull = ADC_FIRST_BUFFER;
	} else {
		Cap_HalfFull = ADC_SECOND_BUFFER;
	}
	if(__HAL_DMA_GET_FLAG(hadc2.DMA_Handle, DMA_FLAG_HT1) == SET) {
		__HAL_DMA_CLEAR_FLAG(hadc2.DMA_Handle, DMA_FLAG_HT1);
		Bat_HalfFull = ADC_FIRST_BUFFER;
	} else {
		Bat_HalfFull = ADC_SECOND_BUFFER;
	}
	uint16_t sum=0;
	for(uint8_t i = 0; i < ADC1_CHANNELS; i++){
		sum = 0;
		for(uint8_t j = 0; j < ADC_BUFFER_DEPTH; j++)
		{
			sum += adc1_samples[Cap_HalfFull][j][i];
		}
		ADC_sampled_data[i] = (sum>>4);
	}
	for(uint8_t i = 0; i < ADC2_CHANNELS ; i++){
		sum = 0;
		for(uint8_t j = 0; j < ADC_BUFFER_DEPTH; j++)
		{
			sum += adc2_samples[Bat_HalfFull][j][i];
		}
		ADC_sampled_data[i+ADC1_CHANNELS] = (sum>>4);
	}
	for(uint8_t i = 0; i < (ADC1_CHANNELS + ADC2_CHANNELS) ; i++)
		ADC_filtered_data[i] = moving_average(filters[i], ADC_sampled_data[i]);
}


 HAL_StatusTypeDef supercap_control_manager::start_loop(){

	if(!CALIBRATION_MODE)
	{
		in_loop = true;
		float initDutyRatio = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap) /
				GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], V_bat);
		start_gates_pwm(initDutyRatio);
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}

 static inline void stop_gates_pwm();

 static inline void stop_gates_pwm(){

//	return HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2);
	hhrtim1.Instance->sCommonRegs.ODISR |= HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2;
}

 static inline void stop_loop();

 void supercap_control_manager::stop_loop(){
	in_loop = false;
	stop_gates_pwm();
}

 static inline void stop_loop(){
	in_loop = false;
	stop_gates_pwm();
}


 void softwareReset()
 {
 	stop_loop();
     __disable_irq();
     while (true)
         NVIC_SystemReset();
 }
 static inline void idle_mode();

 static inline void idle_mode(){
	 if(current_mode != idle){
		 stop_loop();
		 current_mode = idle;
		 in_loop = false;
	 }
 }

 static inline void update_dutyCycle(float dutyRatio);

 static inline void update_dutyCycle(float dutyRatio){
	float D_chassis,D_cap;
	if(dutyRatio>0.0f && dutyRatio<=1.0f)
	{
		D_cap = 0.9f;
		D_chassis  = dutyRatio * 0.9f;
	}
	else if(dutyRatio > 1.0f)
	{
		D_chassis  = 0.9f;
		D_cap = 0.9f / dutyRatio;
	}
	uint16_t TA_CmpOffset = (uint16_t)(HRTIM_PERIOD*D_cap);
	uint16_t TA_CmpOffsetHalf = (uint16_t)(TA_CmpOffset / 2);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, HRTIM_HALF_PERIOD - TA_CmpOffsetHalf - (TA_CmpOffset % 2));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, HRTIM_HALF_PERIOD + TA_CmpOffsetHalf);

	uint16_t TE_CmpOffset = (uint16_t)(HRTIM_PERIOD*D_chassis);
	uint16_t TE_CmpOffsetHalf = (uint16_t)(TE_CmpOffset / 2);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, HRTIM_HALF_PERIOD - TE_CmpOffsetHalf - (TE_CmpOffset % 2));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, HRTIM_HALF_PERIOD + TE_CmpOffsetHalf);

}

 HAL_StatusTypeDef supercap_control_manager::start_gates_pwm(float init_duty_cycle){
	update_dutyCycle(init_duty_cycle);
	return HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2);
}

 static inline float get_PID(LoopCtrl_PID* pid_struct, float ref, float feedback, float ff_model);

 static inline float get_PID(LoopCtrl_PID* pid_struct, float ref, float feedback, float ff_model){
	float error = ref - feedback;
	float output = arm_pid_f32(&(pid_struct->ArmPID_Instance), error) + ff_model;

	if(pid_struct->ArmPID_Instance.state[2] > pid_struct->IntegMax)
		pid_struct->ArmPID_Instance.state[2] = pid_struct->IntegMax;
	else if(pid_struct->ArmPID_Instance.state[2] < pid_struct->IntegMin)
		pid_struct->ArmPID_Instance.state[2] = pid_struct->IntegMin;

	if(output > pid_struct->OutputMax)
		return pid_struct->OutputMax;
	else if(output < pid_struct->OutputMin)
		return pid_struct->OutputMin;
	else
		return output;
}

void supercap_control_manager::set_ref_chassis_power(float ref_power_){
	ref_power = ref_power_;
}

static inline void all_safety_checks();

static inline void safety_check_voltages();

static inline void safety_check_currents();

static inline void all_safety_checks(){
	if(is_init){
		safety_check_voltages();
		safety_check_currents();
	}
}

static inline void loop_update();
static inline void loop_update(){
	all_safety_checks();
	cap_voltage = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);
	chassis_voltage = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], adc_names::V_bat);
	cap_current = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_cap], adc_names::I_cap);
	battery_current = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_bat], adc_names::I_bat);
	if (in_loop){
		update_pid_maxpow();
		float ref_Pchassis = get_PID(&pid[pid_names::P_bat], max_chassis_power-3, chassis_voltage*battery_current, 0);
//		Pchassis_output = ref_Pchassis;
		float temp = ref_Pchassis/cap_voltage;

		if(temp > MAX_CAP_CURRENT)
			temp = MAX_CAP_CURRENT;
		else if (temp < -MAX_CAP_CURRENT)
			temp = -MAX_CAP_CURRENT;

		float ref_icap = ref_Pchassis / (cap_voltage + ESR_VALUE * (temp-I_supercap_last));

		if(ref_icap > MAX_CAP_CURRENT)
			ref_icap = MAX_CAP_CURRENT;
		else if(ref_icap < -MAX_CAP_CURRENT)
			ref_icap = -MAX_CAP_CURRENT;

		icap_output = ref_icap;
		I_supercap_last = ref_icap;

		float ref_duty_ratio = get_PID(&pid[pid_names::I_capa], ref_icap, cap_current, 0);
		float Vcapmax_ratio = get_PID(&pid[pid_names::V_cap_max], SUPERCAP_MAX_VOLTAGE, cap_voltage, 0);
		float Vcapmin_ratio = get_PID(&pid[pid_names::V_cap_min], SUPERCAP_MIN_VOLTAGE, cap_voltage, 0);

		float out_duty_ratio = 0.0;
		if(ref_duty_ratio > Vcapmax_ratio)
			out_duty_ratio = Vcapmax_ratio;
		else if (ref_duty_ratio < Vcapmin_ratio)
			out_duty_ratio = Vcapmin_ratio;
		else out_duty_ratio = ref_duty_ratio;

		update_dutyCycle(out_duty_ratio);
	}
}




static inline void safety_change_level(uint8_t Item, uint8_t Level);

 static inline void safety_change_level(uint8_t Item, uint8_t Level)
 {
 	if((safetyItemLevel[Item]<=warning && safetyItemLevel[Item]!=Level)
 	|| (safetyItemLevel[Item]==runAway && Level==itsJoever))
 	{
 		safetyItemLevel[Item] = Level;
 		safetyChangeTrigger = true;
 	}
 }

 static inline void safety_check_voltages(){
	static uint16_t Vbat_counter = 0;
	static uint16_t Vcap_counter = 0;

	static safetyLevelDef Vbat_status = safe;
	static safetyLevelDef Vcap_status = safe;

	float V_bat_ = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], adc_names::V_bat);
	float V_cap_ = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);

	if(V_bat_ > MAX_PMM_VOLTAGE or V_bat_ < MIN_PMM_VOLTAGE) Vbat_counter += 1000;
	else
	if(Vbat_counter > 0) Vbat_counter--;

	if(Vbat_counter == 0 and Vbat_status != safe) Vbat_status = safe;
	else
	if (Vbat_counter > 0 and Vbat_status != wtf) Vbat_status = wtf;

	if(V_cap_ > V_SUPERCAPMAX_REF || V_cap_ < SUPERCAP_MIN_VOLTAGE)
	{
		if(Vcap_counter<650)
		{
			Vcap_counter++;
		}
	}
	else if(Vcap_counter>0) Vcap_counter--;

	if(Vcap_counter==0 && Vcap_status != safe) Vcap_status = safe;
	else if(Vcap_counter==650 && Vcap_status != wtf) Vcap_status = wtf;
	uint8_t OutputLevel = (Vbat_status>Vcap_status ? Vbat_status : Vcap_status);
	if(OutputLevel== safe && safetyItemLevel[voltageErr]!= safe)
	{
		safety_change_level(voltageErr, safe);
		;
	}
	if(OutputLevel==wtf && safetyItemLevel[voltageErr]!=wtf)
	{
		__disable_irq();
		idle_mode();
		__enable_irq();
		safety_change_level(voltageErr, warning);
	}
}

 static inline void safety_check_currents(){

	static safetyLevelDef I_chassis_status = safe;
	static safetyLevelDef I_cap_status = safe;

	if(safetyItemLevel[currentErr] != itsJoever)
	{
		float I_cap_ = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_cap], adc_names::I_cap);
		float I_gateA= GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_chassis], adc_names::I_chassis);

		float GetIsource = (I_gateA >=0.0f ? I_gateA : ((-1.0f)*I_gateA));
		float GetIcap    = (I_cap_ >=0.0f    ? I_cap_    : ((-1.0f)*I_cap_));

		if(GetIsource>=SUPERCAP_CURRENT_SAFETY_MAX) I_chassis_status = wtf;

		if(GetIcap>=SUPERCAP_CURRENT_SAFETY_MAX) I_cap_status = wtf;

		uint8_t OutputLevel = (I_chassis_status>=I_cap_status ? I_chassis_status : I_cap_status);
		if(safetyItemLevel[currentErr] != OutputLevel)
		{
			switch(safetyItemLevel[currentErr])
			{
				case safe:
				case warning:
				{
					switch(OutputLevel)
					{
						case safe:
						case warning:
						break;
						case wtf:
						{
							__disable_irq();
							idle_mode();
							__enable_irq();
						}
						break;
						case itsJoever:
						{
							__disable_irq();
							idle_mode();
							__enable_irq();
						}
						break;
						default:;
					}
				}
				break;
				default:;
			}

			safety_change_level(currentErr, OutputLevel);
		}
	}
}

void AnalogSignal_ADCDMA_OVRStop(ADC_HandleTypeDef *hadc){hadc->Instance->CR |= 0x00000010;}//ADC->CR->ADSTP write 1

void AnalogSignal_ADCDMA_OVRRecovery(ADC_HandleTypeDef *hadc)
{
	__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
	hadc->Instance->CFGR |= 0x00000001;//ADC->CFGR->DMAEN write 1

	__HAL_DMA_CLEAR_FLAG(hadc->DMA_Handle, DMA_FLAG_TE1);
	hadc->DMA_Handle->Instance->CCR |= 0x00000001;//DMA->CCRx->EN write 1

	while(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == 0UL)
	{
		;
	}
	hadc->Instance->CR |= 0x00000004;//ADC->CR->ADSTART write 1
}

float elapsed_time_s = 0;

 extern "C"
 {
 void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx){
 	uint8_t ADC_OVR_Flag=0;
 	if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)==1UL)
 	{
 		AnalogSignal_ADCDMA_OVRRecovery(&hadc1);
 		ADC_OVR_Flag=1;
 	}
 	if(__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_OVR)==1UL)
 	{
 		AnalogSignal_ADCDMA_OVRRecovery(&hadc2);
 		ADC_OVR_Flag=1;
 	}
 	if(ADC_OVR_Flag==0)
 		{
 			sample_adc();
 			if(is_init)
 			{
				loop_update();
// 			);
 			}
 	}
 }
 }


