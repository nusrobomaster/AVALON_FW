/*
 * supercap_controllers.cpp
 *
 * Firmware control logic for Avalon – a bidirectional buck-boost supercapacitor power manager.
 * Handles analog sensing, PID loop control, power stage modulation, and safety management.
 *
 * Created on: Jul 6, 2024
 * Author: Yassine Bakkali
 */

#include "supercap_controllers.hpp"

/**
 * @brief Output variables used in control and monitoring.
 */
float Pchassis_output;         ///< Output power estimation to the chassis
float icap_output;             ///< PID-calculated reference current for supercap
float duty_ratio_output;       ///< Raw PID output for duty ratio
float out_duty_ratio_output;   ///< Clamped and validated duty ratio to apply

uint8_t max_chassis_power = 45; ///< Max allowed chassis power under RMUC rules

/**
 * @brief Global sensing variables updated during each loop.
 */
float temperature = 0.0f;
float duty_ratio = 0.9f;
float cap_voltage = 0.0f;
float chassis_voltage = 0.0f;
float cap_current = 0.0f;
float battery_current = 0.0f;

static uint16_t ADC_sampled_data[5]; ///< Intermediate ADC buffer after averaging

/**
 * @brief Internal state variables for control loop execution.
 */
static uint8_t is_init = false;        ///< True if control loop is initialized
static uint8_t in_loop = false;        ///< True if control is actively running
static uint8_t safetyChangeTrigger;    ///< Tracks whether a safety level transition occurred
static LoopCtrl_PID pid[5];            ///< Array of PID control structures

static float I_supercap_last = 0.0f;   ///< Stores last supercap current reference

Mode_ModeTypedef current_mode = normal; ///< Current control mode (normal, idle, etc.)
uint8_t safetyItemLevel[8];             ///< Tracks safety status per monitored item

#define ALPHA_PID_POWER 0.98f           ///< Low-pass filter alpha constant for smoothing power signal

/**
 * @brief First-order low pass filter.
 * @param current_value New input value
 * @param previous_value Previously filtered output
 * @param alpha Filter constant (0 < alpha < 1)
 * @return Filtered result
 */
static inline float low_pass_filter(float current_value, float previous_value, float alpha) {
    return alpha * previous_value + (1 - alpha) * current_value;
}
/**
 * @brief Initializes the supercapacitor control loop.
 *
 * This function is the main entry point for setting up all the PID controllers used in
 * power, current, and voltage regulation of the supercapacitor buck-boost control system.
 * It prepares the system to safely start PWM generation and transitions into active loop mode.
 *
 * @return HAL_OK if initialization was successful.
 */
HAL_StatusTypeDef supercap_control_manager::init_loop(){
	setup_pid_controllers();
	return HAL_OK;
}


/**
 * @brief Updates the internal PID limits for chassis power.
 *
 * This function is used internally to update the maximum allowed output and integration range
 * for the chassis power PID controller. It ensures that the controller stays within
 * competition-legal limits by capping both the integral term and final output.
 */
static inline void update_pid_maxpow(){
	pid[0].IntegMax = max_chassis_power - 3;
	pid[0].OutputMax = max_chassis_power - 3;
}


/**
 * @brief Initializes all PID controllers used in the control loop.
 *
 * This sets up the PID control parameters for:
 * - pid[0]: Battery power control loop (P_bat)
 * - pid[2]: Supercapacitor current control loop (I_capa)
 * - pid[3]: Supercap upper voltage limit clamp (V_cap_max)
 * - pid[4]: Supercap lower voltage limit clamp (V_cap_min)
 *
 * For each loop, the gain parameters and min/max output ranges are set.
 * The controller states are initialized using `arm_pid_init_f32`, and duty ratio is
 * precomputed based on real-time readings to avoid unstable startup behavior.
 *
 * If `CALIBRATION_MODE` is disabled, this also starts the PWM gates using the computed
 * initial duty ratio and marks the system as initialized.
 */
void supercap_control_manager::setup_pid_controllers(){
	// Battery power PID
	pid[0].ArmPID_Instance.Kp = P_BATTERY_KP;
	pid[0].ArmPID_Instance.Ki = P_BATTERY_KI;
	pid[0].ArmPID_Instance.Kd = P_BATTERY_KD;
	pid[0].IntegMax = max_chassis_power - 3;
	pid[0].OutputMax = max_chassis_power - 3;
	pid[0].IntegMin = P_BATTERY_MIN_OUTPUT;
	pid[0].OutputMin = P_BATTERY_MIN_OUTPUT;

	// Supercap current PID
	pid[2].ArmPID_Instance.Kp = I_SUPERCAP_KP;
	pid[2].ArmPID_Instance.Ki = I_SUPERCAP_KI;
	pid[2].ArmPID_Instance.Kd = I_SUPERCAP_KD;
	pid[2].IntegMax = I_SUPERCAP_INTEG_MAX;
	pid[2].IntegMin = I_SUPERCAP_INTEG_MIN;
	pid[2].OutputMax = I_SUPERCAP_MAX_OUTPUT;
	pid[2].OutputMin = I_SUPERCAP_MIN_OUTPUT;

	// Supercap over-voltage limit PID
	pid[3].ArmPID_Instance.Kp = V_SUPERCAPMAX_KP;
	pid[3].ArmPID_Instance.Ki = V_SUPERCAPMAX_KI;
	pid[3].ArmPID_Instance.Kd = V_SUPERCAPMAX_KD;
	pid[3].IntegMax = V_SUPERCAPMAX_INTEG_MAX;
	pid[3].IntegMin = V_SUPERCAPMAX_INTEG_MIN;
	pid[3].OutputMax = V_SUPERCAPMAX_MAX_OUTPUT;
	pid[3].OutputMin = V_SUPERCAPMAX_MIN_OUTPUT;

	// Supercap under-voltage limit PID
	pid[4].ArmPID_Instance.Kp = V_SUPERCAPMIN_KP;
	pid[4].ArmPID_Instance.Ki = V_SUPERCAPMIN_KI;
	pid[4].ArmPID_Instance.Kd = V_SUPERCAPMIN_KD;
	pid[4].IntegMax = V_SUPERCAPMIN_INTEG_MAX;
	pid[4].IntegMin = V_SUPERCAPMIN_INTEG_MIN;
	pid[4].OutputMax = V_SUPERCAPMIN_MAX_OUTPUT;
	pid[4].OutputMin = V_SUPERCAPMIN_MIN_OUTPUT;

	// Initialize the PID controllers with CMSIS-DSP
	arm_pid_init_f32(&(pid[0].ArmPID_Instance), 1);
	pid[0].ArmPID_Instance.state[2] = pid[0].IntegMax;

	arm_pid_init_f32(&(pid[2].ArmPID_Instance), 1);

	// Estimate initial duty cycle using live voltage readings to avoid unsafe startup
	HAL_Delay(100);
	float initDutyRatio = 15.0;
	while (initDutyRatio >= 2.0f || initDutyRatio <= 0.0f){
		volatile float cap_temp = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);//NOT TEMPERATURE IF YOU WERE WONDERING
		volatile float bat_temp = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], V_bat);//NOT TEMPERATURE IF YOU WERE WONDERING
		initDutyRatio = cap_temp / bat_temp;
	}
	pid[2].ArmPID_Instance.state[2] = initDutyRatio;

	// Voltage clamps
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


/**
 * @brief Calculates a moving average on a circular buffer filter structure.
 *
 * The moving average filter stores a rolling window of past `n` samples and returns the average
 * based on either the full window or current fill state. This is used for noise filtering
 * of analog signals such as current and voltage before feeding them into PID loops.
 *
 * @param filter The filter state structure (holds index, window buffer, etc.)
 * @param new_sample The latest ADC sample to be added to the buffer
 * @return Filtered (smoothed) 16-bit average value
 */
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
		return (uint16_t)(filter.sum / filter.length);
	else
		return (uint16_t)(filter.sum / filter.index);
}


/**
 * @brief Samples and filters ADC data from both ADC1 and ADC2.
 *
 * This function determines which half of the DMA double-buffer is ready for each ADC,
 * averages multiple samples per channel (to reduce noise), and applies a moving average
 * filter to smooth the data before it's used for PID control and safety checks.
 */
static inline void sample_adc(){
	uint8_t Cap_HalfFull, Bat_HalfFull;

	// Determine current buffer for ADC1 (cap side)
	if(__HAL_DMA_GET_FLAG(hadc1.DMA_Handle, DMA_FLAG_HT1) == SET){
		__HAL_DMA_CLEAR_FLAG(hadc1.DMA_Handle, DMA_FLAG_HT1);
		Cap_HalfFull = ADC_FIRST_BUFFER;
	} else {
		Cap_HalfFull = ADC_SECOND_BUFFER;
	}

	// Determine current buffer for ADC2 (battery side)
	if(__HAL_DMA_GET_FLAG(hadc2.DMA_Handle, DMA_FLAG_HT1) == SET) {
		__HAL_DMA_CLEAR_FLAG(hadc2.DMA_Handle, DMA_FLAG_HT1);
		Bat_HalfFull = ADC_FIRST_BUFFER;
	} else {
		Bat_HalfFull = ADC_SECOND_BUFFER;
	}

	uint16_t sum=0;

	// Average ADC1 samples
	for(uint8_t i = 0; i < ADC1_CHANNELS; i++){
		sum = 0;
		for(uint8_t j = 0; j < ADC_BUFFER_DEPTH; j++){
			sum += adc1_samples[Cap_HalfFull][j][i];
		}
		ADC_sampled_data[i] = (sum >> 4);
	}

	// Average ADC2 samples
	for(uint8_t i = 0; i < ADC2_CHANNELS ; i++){
		sum = 0;
		for(uint8_t j = 0; j < ADC_BUFFER_DEPTH; j++){
			sum += adc2_samples[Bat_HalfFull][j][i];
		}
		ADC_sampled_data[i + ADC1_CHANNELS] = (sum >> 4);
	}

	// Apply moving average filter on all channels
	for(uint8_t i = 0; i < (ADC1_CHANNELS + ADC2_CHANNELS) ; i++)
		ADC_filtered_data[i] = moving_average(filters[i], ADC_sampled_data[i]);
}


/**
 * @brief Starts the control loop by re-enabling PWM gates and computing the initial duty ratio.
 *
 * Only runs if not in calibration mode. Reads current capacitor and battery voltages to estimate
 * a stable initial duty cycle for the buck-boost circuit, then starts gate switching.
 *
 * @return HAL_OK if started successfully, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef supercap_control_manager::start_loop(){
	if(!CALIBRATION_MODE) {
		in_loop = true;
		float initDutyRatio = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap) /
		                      GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], V_bat);
		start_gates_pwm(initDutyRatio);
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}


/**
 * @brief Disables all HRTIM PWM outputs driving the power gates.
 *
 * Forces an output disable through the HRTIM register. Used in safety shutdown and
 * during idle transitions to prevent accidental switching activity.
 */
static inline void stop_gates_pwm(){
	hhrtim1.Instance->sCommonRegs.ODISR |= HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
	                                       HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2;
}


/**
 * @brief Halts the control loop and disables PWM gate signals.
 *
 * This is the object method version. Sets loop flags to false and stops PWM via `stop_gates_pwm`.
 */
void supercap_control_manager::stop_loop(){
	in_loop = false;
	stop_gates_pwm();
}


/**
 * @brief Internal version of loop stop (non-class variant).
 *
 * Exists in addition to the class method for situations where non-member access is needed.
 */
static inline void stop_loop(){
	in_loop = false;
	stop_gates_pwm();
}


/**
 * @brief Forces a software reset of the MCU.
 *
 * Used during critical fault recovery to completely reboot the board. Disables interrupts and
 * enters an infinite loop until the system reset is triggered via NVIC.
 */
void softwareReset(){
	stop_loop();
	__disable_irq();
	while (true)
		NVIC_SystemReset();
}


/**
 * @brief Transitions the system into idle mode safely.
 *
 * Used during safety events or power-off conditions to gracefully exit the control loop.
 * Ensures gates are off and no control loop logic executes.
 */
static inline void idle_mode(){
	if(current_mode != idle){
		stop_loop();
		current_mode = idle;
		in_loop = false;
	}
}


/**
 * @brief Updates the duty cycle for both half-bridges using the HRTIM peripheral.
 *
 * Based on the current ratio, calculates the appropriate compare values for each timer output.
 * The cap and chassis sides are controlled separately using a normalized (0.0–1.0+) input ratio.
 *
 * @param dutyRatio Desired duty ratio (V_cap / V_bat), expected to be in [0.0, 2.0]
 */
static inline void update_dutyCycle(float dutyRatio){
	float D_chassis, D_cap;

	// Determine each half-bridge's duty cycle
	if(dutyRatio > 0.0f && dutyRatio <= 1.0f){
		D_cap = 0.9f;
		D_chassis = dutyRatio * 0.9f;
	} else if(dutyRatio > 1.0f){
		D_chassis = 0.9f;
		D_cap = 0.9f / dutyRatio;
	}

	// Compute compare values for cap side (Timer A)
	uint16_t TA_CmpOffset = (uint16_t)(HRTIM_PERIOD * D_cap);
	uint16_t TA_CmpOffsetHalf = (uint16_t)(TA_CmpOffset / 2);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1,
	                       HRTIM_HALF_PERIOD - TA_CmpOffsetHalf - (TA_CmpOffset % 2));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2,
	                       HRTIM_HALF_PERIOD + TA_CmpOffsetHalf);

	// Compute compare values for chassis side (Timer E)
	uint16_t TE_CmpOffset = (uint16_t)(HRTIM_PERIOD * D_chassis);
	uint16_t TE_CmpOffsetHalf = (uint16_t)(TE_CmpOffset / 2);

	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_1,
	                       HRTIM_HALF_PERIOD - TE_CmpOffsetHalf - (TE_CmpOffset % 2));
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_COMPAREUNIT_2,
	                       HRTIM_HALF_PERIOD + TE_CmpOffsetHalf);
}


/**
 * @brief Starts the high-resolution PWM gate outputs with an initial duty cycle.
 *
 * This function is typically called after PID controllers are initialized.
 * It updates the gate PWM duty cycle and starts all 4 gate signals using the STM32 HRTIM peripheral.
 *
 * @param init_duty_cycle The duty cycle to start with, based on Vcap / Vbat ratio.
 * @return HAL_OK if HRTIM started successfully.
 */
HAL_StatusTypeDef supercap_control_manager::start_gates_pwm(float init_duty_cycle){
	update_dutyCycle(init_duty_cycle);
	return HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1|HRTIM_OUTPUT_TA2|HRTIM_OUTPUT_TE1|HRTIM_OUTPUT_TE2);
}


/**
 * @brief Executes a single PID computation and clamps the result.
 *
 * Applies the PID algorithm using the CMSIS-DSP library. The output is adjusted with a feedforward term.
 * Integral and output values are clamped to safe min/max bounds to prevent wind-up or instability.
 *
 * @param pid_struct Pointer to the PID controller structure.
 * @param ref Desired reference value (setpoint).
 * @param feedback Current feedback value (measurement).
 * @param ff_model Optional feedforward term added to the output.
 * @return Final, clamped output value from the controller.
 */
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


/**
 * @brief Sets the chassis power reference for the outermost control loop.
 *
 * Used by the application layer to define the desired output power level that the
 * control system should regulate around.
 *
 * @param ref_power_ Desired chassis power reference in watts.
 */
void supercap_control_manager::set_ref_chassis_power(float ref_power_){
	ref_power = ref_power_;
}


/**
 * @brief Safely updates the level of a safety item if required.
 *
 * Changes the level only if escalating or recovering from a serious state.
 *
 * @param Item Index of the safety item.
 * @param Level New safety level to apply.
 */
static inline void safety_change_level(uint8_t Item, uint8_t Level){
	if((safetyItemLevel[Item] <= warning && safetyItemLevel[Item] != Level)
	|| (safetyItemLevel[Item] == runAway && Level == itsJoever))
	{
		safetyItemLevel[Item] = Level;
		safetyChangeTrigger = true;
	}
}


/**
 * @brief Performs safety checks on both chassis and supercapacitor voltages.
 *
 * Uses counters for debounce behavior and disables the control loop if unsafe
 * conditions persist (too high or low voltage on either side).
 */
static inline void safety_check_voltages(){
	static uint16_t Vbat_counter = 0;
	static uint16_t Vcap_counter = 0;

	static safetyLevelDef Vbat_status = safe;
	static safetyLevelDef Vcap_status = safe;

	float V_bat_ = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], adc_names::V_bat);
	float V_cap_ = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);

	if(V_bat_ > MAX_PMM_VOLTAGE || V_bat_ < MIN_PMM_VOLTAGE)
		Vbat_counter += 1000;
	else if(Vbat_counter > 0)
		Vbat_counter--;

	if(Vbat_counter == 0 && Vbat_status != safe)
		Vbat_status = safe;
	else if (Vbat_counter > 0 && Vbat_status != wtf)
		Vbat_status = wtf;

	if(V_cap_ > V_SUPERCAPMAX_REF || V_cap_ < SUPERCAP_MIN_VOLTAGE) {
		if(Vcap_counter < 650)
			Vcap_counter++;
	} else if(Vcap_counter > 0)
		Vcap_counter--;

	if(Vcap_counter == 0 && Vcap_status != safe)
		Vcap_status = safe;
	else if(Vcap_counter == 650 && Vcap_status != wtf)
		Vcap_status = wtf;

	uint8_t OutputLevel = (Vbat_status > Vcap_status ? Vbat_status : Vcap_status);

	if(OutputLevel == safe && safetyItemLevel[voltageErr] != safe) {
		safety_change_level(voltageErr, safe);
	}
	if(OutputLevel == wtf && safetyItemLevel[voltageErr] != wtf) {
		__disable_irq();
		idle_mode();
		__enable_irq();
		safety_change_level(voltageErr, warning);
	}
}


/**
 * @brief Checks for overcurrent conditions on both the chassis and the supercapacitor.
 *
 * Monitors the current flowing through the capacitor and the main gate.
 * If the current exceeds the defined safe limits, the system transitions into idle mode.
 *
 * Uses absolute values for current readings since direction doesn’t matter for safety.
 * Only acts if the current error hasn't already reached its most critical state.
 */
static inline void safety_check_currents(){
	static safetyLevelDef I_chassis_status = safe;
	static safetyLevelDef I_cap_status = safe;

	if (safetyItemLevel[currentErr] != itsJoever) {
		float I_cap_  = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_cap], adc_names::I_cap);
		float I_gateA = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_chassis], adc_names::I_chassis);

		float GetIsource = fabsf(I_gateA);
		float GetIcap    = fabsf(I_cap_);

		if (GetIsource >= SUPERCAP_CURRENT_SAFETY_MAX) I_chassis_status = wtf;
		if (GetIcap    >= SUPERCAP_CURRENT_SAFETY_MAX) I_cap_status = wtf;

		uint8_t OutputLevel = (I_chassis_status >= I_cap_status ? I_chassis_status : I_cap_status);

		if (safetyItemLevel[currentErr] != OutputLevel) {
			switch (safetyItemLevel[currentErr]) {
				case safe:
				case warning:
					switch (OutputLevel) {
						case wtf:
						case itsJoever:
							__disable_irq();
							idle_mode();  // Enters a safe idle state to prevent hardware damage
							__enable_irq();
							break;
						default:
							break;
					}
					break;
				default:
					break;
			}
			safety_change_level(currentErr, OutputLevel);
		}
	}
}


/**
 * @brief Runs all safety checks for voltage and current conditions.
 *
 * Combines the voltage and current safety check routines if the system is initialized.
 */
static inline void all_safety_checks(){
	if(is_init){
		safety_check_voltages();
		safety_check_currents();
	}
}


/**
 * @brief Updates the control loop at each HRTIM callback cycle.
 *
 * This function is the heart of the closed-loop control logic.
 * It:
 *  - Runs safety checks
 *  - Reads filtered sensor values
 *  - Computes a desired supercap current to keep battery power below limits
 *  - Computes duty cycle through cascaded PID loops with voltage clamping
 *  - Updates PWM outputs accordingly
 */
static inline void loop_update(){
	all_safety_checks();

	cap_voltage = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_cap], adc_names::V_cap);
	chassis_voltage = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::V_bat], adc_names::V_bat);
	cap_current = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_cap], adc_names::I_cap);
	battery_current = GET_COMPENSATED_ADC(ADC_filtered_data[adc_names::I_bat], adc_names::I_bat);

	if (in_loop){
		update_pid_maxpow();

		// Estimate desired chassis power (W)
		float ref_Pchassis = get_PID(&pid[pid_names::P_bat], max_chassis_power-3, chassis_voltage * battery_current, 0);

		// Estimate supercap current needed to maintain ref_Pchassis
		float temp = ref_Pchassis / cap_voltage;

		// Clamp intermediate target current
		if(temp > MAX_CAP_CURRENT)
			temp = MAX_CAP_CURRENT;
		else if (temp < -MAX_CAP_CURRENT)
			temp = -MAX_CAP_CURRENT;

		// Refine supercap current using ESR model
		float ref_icap = ref_Pchassis / (cap_voltage + ESR_VALUE * (temp - I_supercap_last));

		// Final current clamping
		if(ref_icap > MAX_CAP_CURRENT)
			ref_icap = MAX_CAP_CURRENT;
		else if(ref_icap < -MAX_CAP_CURRENT)
			ref_icap = -MAX_CAP_CURRENT;

		icap_output = ref_icap;
		I_supercap_last = ref_icap;

		// Cascade PID: get duty cycle from current loop
		float ref_duty_ratio = get_PID(&pid[pid_names::I_capa], ref_icap, cap_current, 0);

		// Clamp duty cycle based on voltage thresholds
		float Vcapmax_ratio = get_PID(&pid[pid_names::V_cap_max], SUPERCAP_MAX_VOLTAGE, cap_voltage, 0);
		float Vcapmin_ratio = get_PID(&pid[pid_names::V_cap_min], SUPERCAP_MIN_VOLTAGE, cap_voltage, 0);

		float out_duty_ratio = 0.0;
		if(ref_duty_ratio > Vcapmax_ratio)
			out_duty_ratio = Vcapmax_ratio;
		else if (ref_duty_ratio < Vcapmin_ratio)
			out_duty_ratio = Vcapmin_ratio;
		else
			out_duty_ratio = ref_duty_ratio;

		update_dutyCycle(out_duty_ratio);
	}
}



/**
 * @brief Stops the ongoing ADC DMA conversion in case of overrun.
 *
 * This is a low-level emergency stop. It halts the ADC immediately to prevent further data corruption.
 *
 * @param hadc Pointer to the ADC handle.
 */
void AnalogSignal_ADCDMA_OVRStop(ADC_HandleTypeDef *hadc){
	hadc->Instance->CR |= 0x00000010; // ADC_CR_ADSTP = 1
}


/**
 * @brief Recovers ADC and DMA after an overrun (OVR) condition.
 *
 * Clears error flags and restarts the ADC and DMA modules to restore normal operation.
 * Required to recover from transient ADC DMA overflows due to latency or missed callbacks.
 *
 * @param hadc Pointer to the ADC handle that experienced overrun.
 */
void AnalogSignal_ADCDMA_OVRRecovery(ADC_HandleTypeDef *hadc){
	__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
	hadc->Instance->CFGR |= 0x00000001; // Enable DMA again

	__HAL_DMA_CLEAR_FLAG(hadc->DMA_Handle, DMA_FLAG_TE1);
	hadc->DMA_Handle->Instance->CCR |= 0x00000001; // Re-enable DMA

	// Wait until ADC is ready again
	while(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == 0UL) {
		// spin until ready
	}

	hadc->Instance->CR |= 0x00000004; // Start ADC again
}


/**
 * @brief Main control loop callback called at each HRTIM repetition event.
 *
 * This function is tied to a high-frequency hardware timer (HRTIM).
 * It:
 *  - Checks for ADC overruns and recovers if needed
 *  - Samples ADC data
 *  - Executes the main closed-loop control update
 *
 * This is where the entire feedback control logic is anchored, providing real-time regulation
 * of current, power, and duty cycles for the supercap management system.
 *
 * @param hhrtim Pointer to the HRTIM handle (unused here).
 * @param TimerIdx Index of the timer triggering the event (unused).
 */
extern "C" void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim, uint32_t TimerIdx){
	uint8_t ADC_OVR_Flag = 0;

	// Check and recover ADC1 if it overflowed
	if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR) == 1UL) {
		AnalogSignal_ADCDMA_OVRRecovery(&hadc1);
		ADC_OVR_Flag = 1;
	}

	// Check and recover ADC2 if it overflowed
	if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_OVR) == 1UL) {
		AnalogSignal_ADCDMA_OVRRecovery(&hadc2);
		ADC_OVR_Flag = 1;
	}

	// Continue normal control loop only if no overflow occurred
	if (ADC_OVR_Flag == 0) {
		sample_adc();
		if (is_init) {
			loop_update();  // Core of supercap control logic
		}
	}
}
