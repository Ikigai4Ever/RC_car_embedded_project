/**
 * functions.c
 *
 *  Created on: 11/19/2025
 *      Author: thahrens42
 */


// HEADER FILES //
#include "main.h"


// FUNCTIONS //

/*
 * Get the x and y values from the joystick module
 */
JoyXY Get_Direction(ADC_HandleTypeDef* ADCx){

	JoyXY joystickVal = {0, 0};
	ADC_ChannelConfTypeDef sConfig = {0};

	// --- Read joystick analog values ---
	// --- Read X-axis ---
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	HAL_ADC_ConfigChannel(ADCx, &sConfig);

	HAL_ADC_Start(ADCx);
	HAL_ADC_PollForConversion(ADCx, HAL_MAX_DELAY);
	joystickVal.x = HAL_ADC_GetValue(ADCx);
	HAL_ADC_Stop(ADCx);

	// --- Read Y-axis ---
	sConfig.Channel = ADC_CHANNEL_1;
	HAL_ADC_ConfigChannel(ADCx, &sConfig);

	HAL_ADC_Start(ADCx);
	HAL_ADC_PollForConversion(ADCx, HAL_MAX_DELAY);
	joystickVal.y = HAL_ADC_GetValue(ADCx);
	HAL_ADC_Stop(ADCx);


	return joystickVal;
}

/*
 * @brief Creates custom characters
 */

void Create_Custom_Characters(void){
	char cc1[8] = {0x00, 0x00, 0x0A, 0x00, 0x11, 0x0E, 0x00, 0x00};  // smiley
	char cc2[8] = {0x0E, 0x0E, 0x04, 0x0E, 0x15, 0x04, 0x0A, 0x0A};  // Robo
	char cc3[8] = {0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00};  // arrow
	char cc4[8] = {0x00, 0x04, 0x0E, 0x0E, 0x0E, 0x1F, 0x04, 0x00};  // bell
	char cc5[8] = {0x00, 0x00, 0x0A, 0x15, 0x11, 0x0E, 0x04, 0x00};  // Heart
	char cc6[8] = {0x00, 0x0E, 0x11, 0x11, 0x11, 0x0A, 0x1B, 0x00};  // omega
	char cc7[8] = {0x0E, 0x10, 0x17, 0x12, 0x12, 0x12, 0x10, 0x0E};  // CT
	char cc8[8] = {0x04, 0x04, 0x1F, 0x04, 0x04, 0x00, 0x1F, 0x00};  // +-
}
