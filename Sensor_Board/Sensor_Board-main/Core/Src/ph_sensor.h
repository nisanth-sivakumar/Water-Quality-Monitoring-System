/*
 * ph_sensor.h
 *
 *  Created on: Nov 22, 2024
 *      Author: nisan
 */

#ifndef SRC_PH_SENSOR_H_
#define SRC_PH_SENSOR_H_

#include "stm32f4xx_hal.h"

// constants for the pH sensor
#define PH_NEUTRAL_VOLTAGE 2.5f  //
#define PH_SENSITIVITY 0.17f     //
#define ADC_RESOLUTION 4096.0f   //
#define VREF 3.3f                //

// Function declerations
void PH_Sensor_Init(ADC_HandleTypeDef *hadc);
float PH_Sensor_Read(void);

#endif /* SRC_PH_SENSOR_H_ */
