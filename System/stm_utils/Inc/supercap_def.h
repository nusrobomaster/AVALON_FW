/**
 * @file supercap_def.h
 * @brief Configuration constants for the supercapacitor power manager firmware.
 *
 * This file defines all global parameters for PID tuning, safety limits, thermal calculations,
 * and voltage/current thresholds used throughout the supercapacitor controller logic.
 *
 * Created on: Jul 5, 2024
 * Author: Yassine Bakkali
 */

#ifndef STM_UTILS_INC_SUPERCAP_DEF_H_
#define STM_UTILS_INC_SUPERCAP_DEF_H_

// ───── Board & Mode Configuration ───────────────────────────────
#define BOARD_ID 3
#define CALIBRATION_MODE 0
/**
 * @brief Board ID must be defined externally via compiler or build system.
 * Each board ID maps to a set of calibration constants (in adc_utils.hpp).
 */
#ifndef BOARD_ID
#error "BOARD_ID VARIABLE MUST BE DEFINED."
#endif

/**
 * @brief Enable or disable calibration mode.
 * When enabled, loop update is skipped and manual calibration logic may be run.
 */
#ifndef CALIBRATION_MODE
#error "CALIBRATION MODE VARIABLE MUST BE DEFINED."
#endif

// ───── PID Parameters: Power Loop ───────────────────────────────

#define P_BATTERY_KP 3.85f   ///< Proportional gain for battery power regulation
#define P_BATTERY_KI 0.048f  ///< Integral gain for battery power regulation
#define P_BATTERY_KD 0.001f  ///< Derivative gain for battery power regulation
#define P_BATTERY_MIN_OUTPUT -400.0f  ///< Minimum output for battery power PID

#define P_CHASSIS_MAX_OUTPUT 2.0f     ///< Max output for chassis PID (currently unused)
#define P_CHASSIS_MIN_OUTPUT -2.0f    ///< Min output for chassis PID (currently unused)

// ───── PID Parameters: Current Loop (Supercap) ──────────────────

#define I_SUPERCAP_KP 0.0207f
#define I_SUPERCAP_KI 0.00022f
#define I_SUPERCAP_KD 0.0f

#define I_SUPERCAP_INTEG_MAX 1.621f
#define I_SUPERCAP_INTEG_MIN 0.127f
#define I_SUPERCAP_MAX_OUTPUT 1.621f
#define I_SUPERCAP_MIN_OUTPUT 0.127f

#define I_SUPERCAP_INIT_VAL 0.14f  ///< Initial value for I-supercap PID state

// ───── PID Parameters: Voltage Control (Supercap Max) ───────────

#define V_SUPERCAPMAX_KP 0.001f
#define V_SUPERCAPMAX_KI 0.0025f
#define V_SUPERCAPMAX_KD 0.001f

#define V_SUPERCAPMAX_INTEG_MAX 1.6f
#define V_SUPERCAPMAX_INTEG_MIN 1.005f
#define V_SUPERCAPMAX_MAX_OUTPUT 1.6f
#define V_SUPERCAPMAX_MIN_OUTPUT 1.005f

#define V_SUPERCAPMAX_REF 26.0f
#define V_SUPERCAPMAX_INIT_VAL 1.4f

// ───── PID Parameters: Voltage Control (Supercap Min) ───────────

#define V_SUPERCAPMIN_KP 0.001f
#define V_SUPERCAPMIN_KI 0.0025f
#define V_SUPERCAPMIN_KD 0.0f

#define V_SUPERCAPMIN_INTEG_MAX 0.179f
#define V_SUPERCAPMIN_INTEG_MIN 0.127f
#define V_SUPERCAPMIN_MAX_OUTPUT 0.179f
#define V_SUPERCAPMIN_MIN_OUTPUT 0.127f

#define V_SUPERCAPMIN_REF 2.7f
#define V_SUPERCAPMIN_INIT_VAL 0.1f

// ───── Safety Thresholds ────────────────────────────────────────

#define SUPERCAP_CURRENT_MAX 18.0f   ///< Absolute max current for cap control
#define SUPERCAP_CURRENT_MIN -18.0f
#define SUPERCAP_CURRENT_SAFETY_MAX 20.5f ///< Current limit before triggering safety shutdown

#define ESR_VALUE 0.17f ///< Equivalent series resistance of the supercap bank

#define SUPERCAP_MAX_VOLTAGE 25.5f
#define SUPERCAP_MIN_VOLTAGE 3.00f

#define MIN_PMM_VOLTAGE 18.0f  ///< Minimum voltage allowed from PMM (input rail)
#define MAX_PMM_VOLTAGE 28.0f  ///< Maximum voltage allowed from PMM

#define MAX_CAP_CURRENT 18.0f  ///< Max current used in feedforward model

// ───── Temperature Sensor Parameters (NTC) ──────────────────────

#define TEMP_PARAM_B 3380.0f
#define TEMP_FIX_RES 10000.0f  ///< Fixed resistor in the voltage divider
#define TEMP_NTC_RES 10000.0f  ///< Nominal resistance of the NTC
#define TEMP_CELSIUS_CONST 273.15f
#define TEMP_AMBIANT 298.15f

#define MIN_BAT_VOLTAGE 20.97f  ///< Min voltage to consider battery healthy

#endif /* STM_UTILS_INC_SUPERCAP_DEF_H_ */
