/*
 * supercap_controllers.cpp
 *
 *  Created on: Jul 6, 2024
 *      Author: YassineBakkali
 */

#include "supercap_controllers.hpp"
#include "cmsis_os.h"

uint8_t max_chassis_power = 60;


HAL_StatusTypeDef supercap_control_manager::init_loop(){
	setup_pid_controllers();
	return HAL_OK;
}

void supercap_control_manager::update_pid_maxpow(){
	pid[0].IntegMax = max_chassis_power + 10;
	pid[0].OutputMax = max_chassis_power + 10;
}

void supercap_control_manager::setup_pid_controllers(){
	pid[0].ArmPID_Instance.Kp = P_BATTERY_KP;
	pid[0].ArmPID_Instance.Ki = P_BATTERY_KI;
	pid[0].ArmPID_Instance.Kd = P_BATTERY_KD;

	pid[0].IntegMax = max_chassis_power + 10;
	pid[0].OutputMax = max_chassis_power + 10;

	pid[0].IntegMin = P_BATTERY_MIN_OUTPUT;
	pid[0].OutputMin = P_BATTERY_MIN_OUTPUT;

	pid[1].ArmPID_Instance.Kp = P_CHASSIS_KP;
	pid[1].ArmPID_Instance.Ki = P_CHASSIS_KI;
	pid[1].ArmPID_Instance.Kd = P_CHASSIS_KD;

	pid[1].IntegMax =  0.5f;
	pid[1].IntegMin = -  0.5f;
	pid[1].OutputMax =  0.5f;
	pid[1].OutputMin = - 0.5f;

	pid[2].ArmPID_Instance.Kp = I_SUPERCAP_KP;
	pid[2].ArmPID_Instance.Ki = I_SUPERCAP_KI;
	pid[2].ArmPID_Instance.Kd = I_SUPERCAP_KD;

	pid[2].IntegMax = 24.0f/19.5f;
	pid[2].IntegMin = 0.127f;
	pid[2].OutputMax = 24.0f/19.5f;
	pid[2].OutputMin = 0.127f;

	pid[3].ArmPID_Instance.Kp = V_SUPERCAPMAX_KP;
	pid[3].ArmPID_Instance.Ki = V_SUPERCAPMAX_KI;
	pid[3].ArmPID_Instance.Kd = V_SUPERCAPMAX_KD;

	pid[3].IntegMax = 1.5f;
	pid[3].IntegMin = 1.005f;
	pid[3].OutputMax = 1.5f;
	pid[3].OutputMin = 1.005f;

	pid[4].ArmPID_Instance.Kp = V_SUPERCAPMIN_KP;
	pid[4].ArmPID_Instance.Ki = V_SUPERCAPMIN_KI;
	pid[4].ArmPID_Instance.Kd = V_SUPERCAPMIN_KD;

	pid[4].IntegMax = 0.179f;
	pid[4].IntegMin = 0.129f;
	pid[4].OutputMax = 0.179f;
	pid[4].OutputMin = 0.129f;

	arm_pid_init_f32(&(pid[0].ArmPID_Instance), 1);
	pid[0].ArmPID_Instance.state[2] = pid[0].IntegMax;

	arm_pid_init_f32(&(pid[1].ArmPID_Instance), 1);

	arm_pid_init_f32(&(pid[2].ArmPID_Instance), 1);

	osDelay(100);

	float initDutyRatio = this->get_compensated_adc(this->ADC_filtered_data[adc_names::V_cap], adc_names::V_cap) /
				this->get_compensated_adc(this->ADC_filtered_data[adc_names::V_bat], V_bat);

	pid[2].ArmPID_Instance.state[2] = initDutyRatio;

	arm_pid_init_f32(&(pid[3].ArmPID_Instance), 1);
	pid[3].ArmPID_Instance.state[2] = pid[3].IntegMax;

	arm_pid_init_f32(&(pid[4].ArmPID_Instance), 1);
	pid[4].ArmPID_Instance.state[2] = pid[4].IntegMin;

	HAL_StatusTypeDef status = start_gates_pwm(initDutyRatio);
	is_init = true;

//
	in_loop = true;
}

 void supercap_control_manager::loop_update(float Icap_, float Ibat_, float Vcap_, float Ichassis_, float Vbat_){
//	float ref_pmm_power = get_pid()
	if (in_loop){
		update_pid_maxpow();
		float ref_Pchassis = get_PID(&pid[pid_names::P_bat], max_chassis_power, Vbat_*Ibat_, 0);

		float temp = ref_Pchassis/Vcap_;

		if(temp > MAX_CAP_CURRENT)
			temp = MAX_CAP_CURRENT;
		else if (temp < -MAX_CAP_CURRENT)
			temp = -MAX_CAP_CURRENT;

		float model = ref_Pchassis / (Vcap_ + ESR_VALUE * (temp-I_supercap_last));

		float ref_icap = get_PID_FF(&pid[pid_names::P_chassis], ref_Pchassis, Vbat_*Ichassis_, model);

		I_supercap_last = ref_icap;

		float ref_duty_ratio = get_PID(&pid[pid_names::I_capa], ref_icap, Icap_, 0);
		float Vcapmax_ratio = get_PID(&pid[pid_names::V_cap_max], 21.5, Vcap_, 0);
		float Vcapmin_ratio = get_PID(&pid[pid_names::V_cap_min], 3.0, Vcap_, 0);

		float out_duty_ratio = 0.0;
		if(ref_duty_ratio > Vcapmax_ratio)
			out_duty_ratio = Vcapmax_ratio;
		else if (ref_duty_ratio < Vcapmin_ratio)
			out_duty_ratio = Vcapmin_ratio;
		else out_duty_ratio = ref_duty_ratio;

		update_dutyCycle(out_duty_ratio);
	}
}

 HAL_StatusTypeDef supercap_control_manager::stop_loop(){

	in_loop = false;
	stop_gates_pwm();
	return HAL_OK;
}

 HAL_StatusTypeDef supercap_control_manager::start_gates_pwm(float init_duty_cycle){
	update_dutyCycle(init_duty_cycle);
	return HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2);
}

 HAL_StatusTypeDef supercap_control_manager::stop_gates_pwm(){

	return HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2);
}

 void supercap_control_manager::silent_mode(){
	 if(current_mode != silent){
		 stop_loop();
		 current_mode = silent;
		 in_loop = false;
	 }
 }

 void supercap_control_manager::update_dutyCycle(float dutyCycle){
	float D_chassis,D_cap;
	if(dutyCycle>0.0f && dutyCycle<=1.0f)
	{
		D_cap = 0.9f;
		D_chassis  = dutyCycle * 0.9f;
	}
	else if(dutyCycle > 1.0f)
	{
		D_chassis  = 0.9f;
		D_cap = 0.9f / dutyCycle;
	}
	uint16_t TA_CmpOffset = (uint16_t)(34000*D_cap);
	uint16_t TA_CmpOffsetHalf = (uint16_t)(TA_CmpOffset / 2);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, 17000 - TA_CmpOffsetHalf - (TA_CmpOffset%2));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, 17000 + TA_CmpOffsetHalf);

	uint16_t TE_CmpOffset = (uint16_t)(34000*D_chassis);
	uint16_t TE_CmpOffsetHalf = (uint16_t)(TE_CmpOffset / 2);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1, 17000 - TE_CmpOffsetHalf - (TE_CmpOffset%2));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2, 17000 + TE_CmpOffsetHalf);

}

 float supercap_control_manager::get_PID(LoopCtrl_PID* pid_struct, float ref, float feedback, float ff_model){
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

 float supercap_control_manager::get_PID_FF(LoopCtrl_PID* pid_struct, float ref, float feedback, float ff_model){
    float error = ref - feedback;
	float output = arm_pid_f32(&(pid_struct->ArmPID_Instance), error);
	float temp = 0;
	if(pid_struct->ArmPID_Instance.state[2] > pid_struct->IntegMax)
		pid_struct->ArmPID_Instance.state[2] = pid_struct->IntegMax;
	else if(pid_struct->ArmPID_Instance.state[2] < pid_struct->IntegMin)
		pid_struct->ArmPID_Instance.state[2] = pid_struct->IntegMin;

	if(output > pid_struct->OutputMax)
		temp = pid_struct->OutputMax;
	else if(output < pid_struct->OutputMin)
		temp = pid_struct->OutputMin;
	else
		temp = output;

	temp += ff_model;
	if(temp > MAX_CAP_CURRENT)
		return MAX_CAP_CURRENT;
	else if(temp < -MAX_CAP_CURRENT)
		return -MAX_CAP_CURRENT;
	else
		return temp;
}

void supercap_control_manager::set_ref_chassis_power(float ref_power_){
	ref_power = ref_power_;
}

 void supercap_control_manager::all_safety_checks(){
	if(is_init){
		safety_check_voltages();
		safety_check_currents();
//		safety_check_powers();
	//	safety_check_temp();
	}
}

 void supercap_control_manager::safety_change_level(uint8_t Item, uint8_t Level)
 {
 	if((safetyItemLevel[Item]<=warning && safetyItemLevel[Item]!=Level)
 	|| (safetyItemLevel[Item]==runAway && Level==itsJoever))
 	{
 		safetyItemLevel[Item] = Level;
 		safetyChangeTrigger = true;
 	}
 }

 void supercap_control_manager::safety_check_voltages(){
	float V_bat_ = get_compensated_adc(ADC_filtered_data[adc_names::V_bat], adc_names::V_bat);
	float V_cap_ = get_compensated_adc(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);

	if(V_bat_ > MAX_PMM_VOLTAGE or V_bat_ < MIN_PMM_VOLTAGE) Vbat_counter = 1000;
	else
	if(Vbat_counter > 0) Vbat_counter--;

	if(Vbat_counter == 0 and Vbat_status != safe) Vbat_status = safe;
	else
	if (Vbat_counter > 0 and Vbat_status != wtf) Vbat_status = wtf;

	if(V_cap_ > 26.0f || V_cap_ < SUPERCAP_MIN_VOLTAGE)
	{
		if(Vcap_counter<650)
		{
			Vcap_counter++;
		}
	}
	else if(Vcap_counter>0)
	{
		Vcap_counter--;
	}

	if(Vcap_counter==0 && Vcap_status != safe)
	{
		Vcap_status = safe;
	}
	else if(Vcap_counter==650 && Vcap_status != wtf)
	{
		Vcap_status = wtf;
	}
	uint8_t OutputLevel = (Vbat_status>Vcap_status ? Vbat_status : Vcap_status);
	if(OutputLevel== safe && safetyItemLevel[voltageErr]!= safe)
	{
		safety_change_level(voltageErr, safe);
//		Safety_TryUserCtrlMode();
		;
	}
	if(OutputLevel==wtf && safetyItemLevel[voltageErr]!=wtf)
	{
		__disable_irq();
		silent_mode();
//		Mode_EN = 0U;
		__enable_irq();
		safety_change_level(voltageErr, warning);
	}
}

 void supercap_control_manager::safety_check_currents(){

	if(safetyItemLevel[currentErr] != itsJoever)
	{
		float I_cap_ = get_compensated_adc(ADC_filtered_data[adc_names::I_cap], adc_names::I_cap);
		float I_chassis_= get_compensated_adc(ADC_filtered_data[adc_names::I_chassis], adc_names::I_chassis);
		float GetIsource = (I_chassis_>=0.0f ? I_chassis_ : ((-1.0f)*I_chassis_));
		float GetIcap    = (I_cap_>=0.0f    ? I_cap_    : ((-1.0f)*I_cap_));

		if(GetIsource<SUPERCAP_CURRENT_SAFETY_MAX && I_chassis_counter>0)
		{
			I_chassis_counter--;
		}
		else if(GetIsource>=SUPERCAP_CURRENT_SAFETY_MAX && GetIsource<13.5f && I_chassis_counter<100)
		{
			I_chassis_counter++;
		}
		else if(GetIsource>=13.5f)
		{
			I_chassis_counter=100;
		}

		if(I_chassis_counter==0 && I_chassis_status!=safe)
		{
			I_chassis_status = safe;
		}
		else if(I_chassis_counter>0 && I_chassis_counter<100 && I_chassis_status<warning)
		{
			I_chassis_status = warning;
		}
		else if(I_chassis_counter==100 && I_chassis_status < wtf)
		{
			I_chassis_status = wtf;
		}

		if(I_chassis_status == wtf && I_chassis_counter < 150)
		{
			I_chassis_counter++;
		}
		else if(I_chassis_status < wtf && I_chassis_counter>0)
		{
			I_chassis_counter--;
		}

		if(I_chassis_counter==150)
		{
			I_chassis_status = itsJoever;
		}

		if(GetIcap<SUPERCAP_CURRENT_SAFETY_MAX && I_cap_counter>0)
		{
			I_cap_counter--;
		}
		else if(GetIcap>=SUPERCAP_CURRENT_SAFETY_MAX && GetIcap<13.5f && I_cap_counter<100)
		{
			I_cap_counter++;
		}
		else if(GetIcap>=13.5f)
		{
			I_cap_counter=100;
		}

		if(I_cap_counter==0 && I_cap_status!=safe)
		{
			I_cap_status = safe;
		}
		else if(I_cap_counter>0 && I_cap_counter < 100 && I_cap_status < warning)
		{
			I_cap_status = warning;
		}
		else if(I_cap_counter==100 && I_cap_status < wtf)
		{
			I_cap_status = wtf;
		}

		if(I_cap_status == wtf && I_cap_force_counter<150)
		{
			I_cap_force_counter++;
		}
		else if(I_cap_status < wtf && I_cap_force_counter>0)
		{
			I_cap_force_counter--;
		}

		if(I_cap_force_counter==150)
		{
			I_cap_status = itsJoever;
		}


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
							silent_mode();
	//						Mode_EN = 0U;
							__enable_irq();
						}
						break;
						case itsJoever:
						{
							__disable_irq();
							silent_mode();
	//						Mode_EN = 0U;
							__enable_irq();
	//						Safety_FlashErrorRecord(Current_Error);
						}
						break;
						default:;
					}
				}
				break;
				case wtf:
				{
					switch(OutputLevel)
					{
						case safe:
						case warning:
						{
	//						Safety_TryUserCtrlMode();
						}
						break;
						case itsJoever:
						{
	//						Safety_FlashErrorRecord(Current_Error);
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

 void supercap_control_manager::safety_check_temp(){
	float temp_s = 0;
	if(safetyItemLevel[tempErr] != itsJoever){
		if(temp_s<70.0f && temp_counter>0)
		{
			temp_counter--;
		}
		else if(temp_s>=70.0f && temp_s<80.0f && temp_counter<50)
		{
			temp_counter++;
		}
		else if(temp_s>=80.0f)
		{
			temp_counter = 50;
		}

		if(temp_counter==0 && safetyItemLevel[tempErr]!= safe)
		{
			safety_change_level(tempErr, safe);
	//		Safety_TryUserCtrlMode();
		}
		else if(temp_counter>0 && temp_counter<50 && safetyItemLevel[tempErr]<warning)
		{
			safety_change_level(tempErr, warning);
		}
		else if(temp_counter==50 && safetyItemLevel[tempErr] < wtf)
		{
			__disable_irq();
			silent_mode();
	//		Mode_EN = 0U;
			__enable_irq();
			safety_change_level(tempErr, wtf);
		}

		if(safetyItemLevel[tempErr] == wtf && temp_counter<100)
		{
			temp_counter++;
		}
		else if(safetyItemLevel[tempErr] < wtf && temp_counter>0)
		{
			temp_counter--;
		}

		if(temp_counter==100)
		{
			safety_change_level(tempErr, itsJoever);
	//		Safety_FlashErrorRecord(Temp_Error);
		}
	}
}

 void supercap_control_manager::safety_check_powers(){
	if(safetyItemLevel[dpErr] != itsJoever)
	{
		float V_bat_ = get_compensated_adc(ADC_filtered_data[adc_names::V_bat], adc_names::V_bat);
		float I_chassis_ = get_compensated_adc(ADC_filtered_data[adc_names::I_chassis], adc_names::I_chassis);
		float V_cap_ = get_compensated_adc(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);
		float I_cap_ = get_compensated_adc(ADC_filtered_data[adc_names::I_cap], adc_names::I_cap);

		float deltaP = V_bat_*I_chassis_ - V_cap_*I_cap_;
		deltaP = (deltaP>=0.0f ? deltaP : ((-1.0f)*deltaP));

		if(deltaP<50.0f && P_ref_counter>0)
		{
			P_ref_counter--;
		}
		else if(deltaP>=25.0f && P_ref_counter<350)
		{
			P_ref_counter++;
		}

		if(P_ref_counter == 0 && safetyItemLevel[dpErr] != safe)
		{
			safety_change_level(dpErr, safe);
	//		Safety_TryUserCtrlMode();
		}
		else if(P_ref_counter>0 && P_ref_counter<350 && safetyItemLevel[dpErr] < warning)
		{
			safety_change_level(dpErr, warning);
		}
		else if(P_ref_counter==350 && safetyItemLevel[dpErr] < wtf)
		{
			__disable_irq();
			silent_mode();
//			Mode_EN = 0U;
			__enable_irq();
			safety_change_level(dpErr, wtf);
		}

		if(safetyItemLevel[dpErr] == wtf && P_ref_force_counter<450)
		{
			P_ref_force_counter++;
		}
		else if(safetyItemLevel[dpErr] < wtf && P_ref_force_counter>0)
		{
			P_ref_force_counter--;
		}

		if(P_ref_force_counter == 450)
		{
			safety_change_level(dpErr, itsJoever);
	//		Safety_FlashErrorRecord(deltaP_Error);
		}
	}
}


 void supercap_control_manager::safety_check_dma(void)
 {
 	if(DMA_counter<3)
 	{
 		DMA_counter++;

 		if(safetyItemLevel[DMAErr] != warning)
 		{
 			safety_change_level(DMAErr, warning);
 		}
 	}
 	else
 	{
 		if(safetyItemLevel[DMAErr] != runAway)
 		{
 			__disable_irq();
// 			Mode_EN = 0U;
 			silent_mode();
 			__enable_irq();
 			safety_change_level(DMAErr, runAway);
 		}
 	}
 }


