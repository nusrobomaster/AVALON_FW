/**
 * @file adc_utils.hpp
 * @brief Header file for managing ADC sampling, compensation, and filtering for the supercapacitor control board.
 *
 * Defines gain/offset macros, buffer sizes, filtering structures, and the ADC manager class.
 *
 * Created on: Jul 5, 2024
 * Author: Yassine Bakkali
 */

#ifndef STM_UTILS_INC_ADC_UTILS_HPP_
#define STM_UTILS_INC_ADC_UTILS_HPP_

#include <adc.h>
#include <dma.h>
#include <stdint.h>
#include <arm_math.h>
#include "supercap_def.h"
#include "main.h"

// ==========================
// ADC and Signal Definitions
// ==========================

/** @brief ADC maximum value (12-bit) */
#define ADC_RES 4095.0f

/** @brief Inverse of voltage divider ratio (used for voltage measurement scaling) */
#define INV_VOLT_DIV 11.0f

/** @brief Reference voltage for ADC (V) */
#define ADC_REF 3.3f

/** @brief INA186A2 current amplifier gain (datasheet) */
#define INA186A2_GAIN 50.0f

/** @brief Value of current sensing shunt resistor (Ohms) */
#define SHUNT_RESISTOR 0.004f

/** @brief Voltage measurement gain: V = (ADC / 4095) * 3.3 * 11 */
#define VCAP_GAIN ADC_REF * INV_VOLT_DIV / ADC_RES
#define VBAT_GAIN ADC_REF * INV_VOLT_DIV / ADC_RES

/** @brief Current measurement gain from each source, sign indicates direction */
#define IBAT_GAIN -(ADC_REF / SHUNT_RESISTOR / INA186A2_GAIN / ADC_RES)
#define ISOURCE_GAIN -(ADC_REF / SHUNT_RESISTOR / INA186A2_GAIN / ADC_RES)
#define ICAP_GAIN -(ADC_REF / SHUNT_RESISTOR / INA186A2_GAIN / ADC_RES)

// ==========================
// Offset Calibration per Board
// ==========================

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

// ==========================
// ADC Buffers and Channels
// ==========================

#define ADC1_CHANNELS 2
#define ADC2_CHANNELS 3
#define ADC_FIRST_BUFFER 0
#define ADC_SECOND_BUFFER 1
#define ADC_BUFFER_DEPTH 16

// ==========================
// Enumerations
// ==========================

/**
 * @brief Enum for identifying ADC channels
 */
typedef enum {
	I_cap,      ///< Current from capacitor side
	V_cap,      ///< Voltage on supercap bank
	V_bat,      ///< Voltage from battery input
	I_chassis,  ///< Current flowing to chassis
	I_bat       ///< Current from PMM to board
} adc_names;

/**
 * @brief Enum for identifying PID loops
 */
typedef enum {
	P_bat,
	P_chassis,
	I_capa,
	V_cap_max,
	V_cap_min
} pid_names;

// ==========================
// Data Structures
// ==========================

/**
 * @brief Moving average filter structure
 */
typedef struct {
	uint16_t window_filter[64]; ///< Circular buffer of values
	uint8_t  length;            ///< Number of samples in filter
	uint8_t  index;             ///< Current index in circular buffer
	uint32_t sum;               ///< Running sum of buffer
	uint8_t  full;              ///< True if buffer has been filled at least once
} mov_avrg_filter ;

/**
 * @brief PID loop controller structure
 */
typedef struct {
	arm_pid_instance_f32 ArmPID_Instance; ///< CMSIS ARM PID instance
	float IntegMax;       ///< Maximum integral term
	float IntegMin;       ///< Minimum integral term
	float OutputMax;      ///< Maximum output
	float OutputMin;      ///< Minimum output
} LoopCtrl_PID;

// ==========================
// ADC Manager Class
// ==========================

/**
 * @class adc_manager
 * @brief Class to manage ADC hardware sampling and filtering
 */
class adc_manager {
public:
	/**
	 * @brief Constructor
	 * @param adc_cap Pointer to ADC for capacitor-side sampling
	 * @param adc_bat Pointer to ADC for battery-side sampling
	 * @param adc_temp Pointer to ADC for temperature sensing
	 */
	adc_manager(ADC_HandleTypeDef* adc_cap, ADC_HandleTypeDef* adc_bat, ADC_HandleTypeDef* adc_temp)
		: adc_cap_(adc_cap), adc_bat_(adc_bat), adc_temp_(adc_temp) {}

	/**
	 * @brief Initializes ADCs and filter states
	 * @param filters_length Pointer to array of filter lengths per channel
	 * @return HAL_OK if successful
	 */
	HAL_StatusTypeDef adc_init(uint8_t* filters_length);

	/**
	 * @brief Reads temperature using onboard NTC
	 * @return Temperature in Celsius
	 */
	float get_temperature();

	bool calibration_ready = false; ///< Optional flag for calibration flow

private:
	void set_gains_offsets(); ///< Applies gain/offset based on board ID

	// ADC handles
	ADC_HandleTypeDef* adc_cap_;
	ADC_HandleTypeDef* adc_bat_;
	ADC_HandleTypeDef* adc_temp_;

	// (Deprecated) unused PID constants
	float P_KP_chassis, P_KI_chassis, P_KD_chassis;
	float P_KP_battery, P_KI_battery, P_KD_battery;
	float I_KP_supercap, I_KI_supercap, I_KD_supercap;
	float V_KP_capMax, V_KI_capMax, V_KD_capMax;
	float V_KP_capMin, V_KI_capMin, V_KD_capMin;
};

// ==========================
// External Variables
// ==========================

extern uint16_t adc1_samples[2][16][2]; ///< Raw buffer for ADC1
extern uint16_t adc2_samples[2][16][3]; ///< Raw buffer for ADC2
extern uint16_t ADC_filtered_data[5];  ///< Filtered data per channel
extern mov_avrg_filter filters[5];     ///< Filter states per channel
extern float adc_gains[5];             ///< Gain correction per channel
extern float adc_offsets[5];           ///< Offset correction per channel

/**
 * @brief Helper macro for applying gain + offset compensation on raw ADC readings
 */
#define GET_COMPENSATED_ADC(ADC_VAL, NAME) (((float)(ADC_VAL)) * adc_gains[NAME]) + adc_offsets[NAME]

#endif /* STM_UTILS_INC_ADC_UTILS_HPP_ */
