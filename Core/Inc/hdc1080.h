/*
 * hdc1080.h
 *
 *  Created on: Sep 27, 2023
 *      Author: Troshca
 */

#ifndef __HDC1080_H__
#define __HDC1080_H__

#include "stm32l4xx_hal.h"

#define         HDC_1080_ADD                            0x40
#define         Configuration_register_add              0x02
#define         Temperature_register_add                0x00
#define         Humidity_register_add                   0x01


typedef enum
{
  Temperature_Resolution_14_bit = 0,
  Temperature_Resolution_11_bit = 1
}Temp_Reso;

typedef enum
{
  Humidity_Resolution_14_bit = 0,
  Humidity_Resolution_11_bit = 1,
  Humidity_Resolution_8_bit =2
}Humi_Reso;

struct TempHum
{
	uint8_t Humidity;
	float Temperature;
};

struct TempHum hdc1080_get_temp_hum(I2C_HandleTypeDef* hi2c_x);
HAL_StatusTypeDef hdc1080_ask_temp_hum(I2C_HandleTypeDef* hi2c_x);
struct TempHum hdc1080_receive_temp_hum(I2C_HandleTypeDef* hi2c_x);
void hdc1080_init(I2C_HandleTypeDef* hi2c_x,Temp_Reso Temperature_Resolution_x_bit,Humi_Reso Humidity_Resolution_x_bit);
uint8_t hdc1080_start_measurement(I2C_HandleTypeDef* hi2c_x,float* temperature, uint8_t* humidity);
#endif // __HDC1080_H__
