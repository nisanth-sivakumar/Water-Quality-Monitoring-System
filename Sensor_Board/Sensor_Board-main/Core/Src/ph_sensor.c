/*
 * ph_sensor.c
 *
 *  Created on: Nov 22, 2024
 *      Author: nisan
 */

#include "ph_sensor.h"

/* Global variable to hold the ADC handle */
static ADC_HandleTypeDef *ph_adc_handle;

/**
 * @brief Initializes the pH sensor module.
 * @param hadc: Pointer to the ADC handle used for the sensor.
 */
void PH_Sensor_Init(ADC_HandleTypeDef *hadc) {
    ph_adc_handle = hadc;  // Assign the ADC handle
}

/**
 * @brief Reads the pH value from the sensor.
 * @return pH value as a float.
 */
float PH_Sensor_Read(void) {
    uint32_t adcValue = 0;
    float voltage = 0.0f;
    float pH = 0.0f;

    // Start ADC Conversion
    HAL_ADC_Start(ph_adc_handle);
    if (HAL_ADC_PollForConversion(ph_adc_handle, HAL_MAX_DELAY) == HAL_OK) {
        adcValue = HAL_ADC_GetValue(ph_adc_handle);  // Get ADC value
    }
    HAL_ADC_Stop(ph_adc_handle);

    // Convert ADC value to voltage
    voltage = (adcValue / ADC_RESOLUTION) * VREF;

    // Convert voltage to pH
    pH = 7.0f + ((voltage - PH_NEUTRAL_VOLTAGE) / PH_SENSITIVITY);

    return pH;
}
