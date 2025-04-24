/**
 * @file supercap_controllers.hpp
 * @brief Core control logic for the supercapacitor power manager.
 *
 * This header defines the controller class and enums responsible for managing
 * the supercapacitor charge/discharge behavior, error safety levels, and PID loops.
 *
 * Created on: Jul 6, 2024
 * Author: Yassine Bakkali
 */

#ifndef STM_UTILS_INC_SUPERCAP_CONTROLLERS_HPP_
#define STM_UTILS_INC_SUPERCAP_CONTROLLERS_HPP_

#include "adc_utils.hpp"
#include "hrtim.h"

/**
 * @brief Enumerates the various error detection sources used for safety monitoring.
 */
typedef enum {
	voltageErr,    ///< Detected unsafe or unstable voltage conditions
	currentErr     ///< Detected unsafe current draw or injection
} Safety_DetectItemTypedef;

/**
 * @brief Safety level definitions for runtime system checks.
 *
 * These are used internally to determine how critical a condition is
 * and what kind of reaction the controller should take.
 */
typedef enum {
	safe,       ///< Normal operation
	warning,    ///< Mild error, could recover automatically
	wtf,        ///< Dangerous state; enters idle
	runAway,    ///< Reserved for runaway behavior
	itsJoever   ///< Catastrophic failure (system shutdown trigger)
} safetyLevelDef;

/**
 * @brief Modes for the supercap control loop.
 */
typedef enum {
	idle,    ///< Supercap system inactive
	normal   ///< Supercap active and managing power
} Mode_ModeTypedef;

/**
 * @brief Defines the full period (and half-period) for HRTIM PWM in timer ticks.
 * Used to calculate duty cycle offset during PWM generation.
 */
#define HRTIM_PERIOD 34000
#define HRTIM_HALF_PERIOD 17000

/**
 * @class supercap_control_manager
 * @brief Main control class for managing the supercapacitor charge/discharge behavior.
 *
 * Inherits from `adc_manager` to reuse ADC data collection logic. This class manages
 * PID-based power balancing, duty cycle generation, and safety enforcement logic.
 */
class supercap_control_manager : public adc_manager {
public:
	/**
	 * @brief Constructor for the supercap controller.
	 * @param adc_cap Pointer to ADC for supercap measurements
	 * @param adc_bat Pointer to ADC for battery-side measurements
	 * @param adc_temp Pointer to ADC used for temperature sensing
	 */
	supercap_control_manager(ADC_HandleTypeDef* adc_cap, ADC_HandleTypeDef* adc_bat, ADC_HandleTypeDef* adc_temp) :
		adc_manager(adc_cap, adc_bat, adc_temp) {};

	/// Destructor (default)
	~supercap_control_manager() {};

	/**
	 * @brief Initializes the control loop by setting up the PID controllers.
	 * @return HAL_OK if setup is successful
	 */
	HAL_StatusTypeDef init_loop();

	/**
	 * @brief Configures PID controllers for power, current, and voltage regulation.
	 */
	void setup_pid_controllers();

	/**
	 * @brief Sets the desired reference power draw (in watts) for the chassis.
	 * @param ref_power_ The power target.
	 */
	void set_ref_chassis_power(float ref_power_);

	/**
	 * @brief Performs a full control loop update using current sensor inputs.
	 * @param Icap_ Current into the supercap
	 * @param Ibat_ Current from the battery
	 * @param Vcap_ Voltage of the supercap bank
	 * @param Ichassis_ Current drawn by the chassis
	 * @param Vbat_ Voltage of the battery
	 */
	void loop_update(float Icap_, float Ibat_, float Vcap_, float Ichassis_, float Vbat_);

	/**
	 * @brief Stops the control loop and disables gate driving.
	 */
	void stop_loop();

	/**
	 * @brief Starts the supercap control loop (if not in calibration mode).
	 * @return HAL_OK on success, HAL_ERROR otherwise
	 */
	HAL_StatusTypeDef start_loop();

	/**
	 * @brief Initializes the gate PWM output using a given starting duty cycle.
	 * @param init_duty_cycle Duty ratio to apply
	 * @return HAL_OK if successful
	 */
	HAL_StatusTypeDef start_gates_pwm(float init_duty_cycle);

	/**
	 * @brief Executes all defined safety checks (voltage, current).
	 */
	void all_safety_checks();

	/**
	 * @brief Manually forces system into a silent mode (e.g., for emergency recovery).
	 */
	void silent_mode();

	/// Current system safety level status
	safetyLevelDef safetyLevel;

private:
	/// Reference power (in watts) used as control target for PID logic
	float ref_power = 60.0f;
};

/**
 * @brief Software-triggered MCU reset function (called on fatal error or remote command).
 */
void softwareReset();

/**
 * @brief Global safety flags indexed by Safety_DetectItemTypedef.
 */
extern uint8_t safetyItemLevel[8];

/**
 * @brief Extern global values for chassis voltage and battery current
 * (used across control loops and diagnostics).
 */
extern float chassis_voltage;
extern float battery_current;

#endif /* STM_UTILS_INC_SUPERCAP_CONTROLLERS_HPP_ */
