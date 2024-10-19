/*
 * adc_utils.cpp
 *
 *  Created on: Jul 5, 2024
 *      Author: YassineBakkali
 */

#include "adc_utils.hpp"
#include "math.h"

uint16_t adc1_samples[2][16][2];
uint16_t adc2_samples[2][16][3];

HAL_StatusTypeDef adc_manager::adc_init(uint8_t* filters_length){

  HAL_ADC_Start_DMA(adc_cap_, (uint32_t*)(&adc1_samples[0][0][0]),64);
  HAL_ADC_Start_DMA(adc_bat_, (uint32_t*)(&adc2_samples[0][0][0]),96);

  HAL_ADCEx_Calibration_Start(adc_temp_, ADC_SINGLE_ENDED);
  HAL_ADC_Start(adc_temp_);

  for(int i = 0; i < 5; ++i){
	  filters[i].sum = 0;
	  filters[i].index = 0;
	  filters[i].full = 0;
	  filters[i].length = filters_length[i];
	  for (int j = 0; j < filters[i].length; j++) {
	      filters[i].window_filter[j] = 0;  // Initialize each element to 0.0
	  }
  }
  set_gains_offsets();
}

 HAL_StatusTypeDef adc_manager::sample_adc(){
	uint8_t Cap_HalfFull, Bat_HalfFull;
	if(__HAL_DMA_GET_FLAG(hadc1.DMA_Handle, DMA_FLAG_HT1) == SET){
		__HAL_DMA_CLEAR_FLAG(hadc1.DMA_Handle, DMA_FLAG_HT1);
		Cap_HalfFull=0;
	} else {
		Cap_HalfFull=1;
	}
	if(__HAL_DMA_GET_FLAG(hadc2.DMA_Handle, DMA_FLAG_HT1) == SET) {
		__HAL_DMA_CLEAR_FLAG(hadc2.DMA_Handle, DMA_FLAG_HT1);
		Bat_HalfFull=0;
	} else {
		Bat_HalfFull=1;
	}
	uint16_t sum=0;
	for(uint8_t i = 0; i < 2 ; i++){
		sum = 0;
		for(uint8_t j = 0; j < 16; j++)
		{
			sum += adc1_samples[Cap_HalfFull][j][i];
		}
		ADC_sampled_data[i] = (sum>>4);
	}
	for(uint8_t i = 0; i < 3 ; i++){
		sum = 0;
		for(uint8_t j = 0; j < 16; j++)
		{
			sum += adc2_samples[Bat_HalfFull][j][i];
		}
		ADC_sampled_data[i+2] = (sum>>4);
	}
	for(uint8_t i = 0; i < 5; i++)
		ADC_filtered_data[i] = moving_average(filters[i], ADC_sampled_data[i]);
}

 uint16_t adc_manager::moving_average(mov_avrg_filter& filter, uint16_t new_sample){
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

void adc_manager::set_gains_offsets(){
	adc_gains[adc_names::V_cap] = 0.0088623046875f;
	adc_offsets[adc_names::V_cap] = 0.4914794921875f;

	adc_gains[adc_names::I_cap] = -(0.0040293f);
	adc_offsets[adc_names::I_cap] = -(-8.25f + 0.31227863582f - 0.0402927399f);

	adc_gains[adc_names::V_bat] = 0.0088623046875f;
	adc_offsets[adc_names::V_bat] = 0.5060302734375f;

	adc_gains[adc_names::I_chassis] = -(0.0040293f);
	adc_offsets[adc_names::I_chassis] = -(-8.25f + 0.4089821269f + 0.0161170959f -0.193406105f);

	adc_gains[adc_names::I_bat] = -(0.0040293f);
	adc_offsets[adc_names::I_bat] = -(-8.25f + 0.2598981019f + 0.012087822f - 0.0402927399f) ;
}

 float adc_manager::get_temperature(){
	float V_temp = 0.001f * __HAL_ADC_CALC_DATA_TO_VOLTAGE(3300, HAL_ADC_GetValue(adc_temp_), ADC_RESOLUTION_12B);
	float NTC_Res = (3.3f - V_temp) / (V_temp/TEMP_NTC_RES);
	return ((1/((1.0f/TEMP_PARAM_B) * ((float)log((double)(NTC_Res/TEMP_FIX_RES))) + 1.0f/TEMP_AMBIANT)) - TEMP_CELSIUS_CONST);

}

float adc_manager::get_compensated_adc(uint16_t val, adc_names name){
	return (((float)(val)) * adc_gains[name]) +  adc_offsets[name];
}
