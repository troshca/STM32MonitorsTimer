/*
 * hdc1080.c
 *
 *  Created on: Sep 27, 2023
 *      Author: Troshca
 */


#include "hdc1080.h"
#include "stm32l4xx_hal_i2c.h"

/*
8.4 Device Functional Modes
The HDC1080 has two modes of operation: sleep mode and measurement mode. After power up, the HDC1080
is in sleep mode. In this mode, the HDC1080 waits for I2C input including commands to configure the conversion
times, read the status of the battery, trigger a measurement, and read measurements. Once it receives a
command to trigger a measurement, the HDC1080 moves from sleep mode to measurement mode. After
completing the measurement the HDC1080 returns to sleep mode.
*/

/**
 * @brief Initializes the HDC1080. Sets clock halt bit to 0 to start timing.
 * @param Temp_Reso Temperature_Resolution in bits
 * @param Humi_Reso Humidity_Resolution in bits
 * @param temperature Pointer to the temperature
 * @param humidity Pointer to the humidity
 */
void hdc1080_init(I2C_HandleTypeDef* hi2c_x,Temp_Reso Temperature_Resolution_x_bit,Humi_Reso Humidity_Resolution_x_bit)
{
	/* Temperature and Humidity are acquired in sequence, Temperature first
	 * Default:   Temperature resolution = 14 bit,
	 *            Humidity resolution = 14 bit
	 */

	/* Set the acquisition mode to measure both temperature and humidity by setting Bit[12] to 1 */
	uint16_t config_reg_value=0x1000;
	uint8_t data_send[2];

	if(Temperature_Resolution_x_bit == Temperature_Resolution_11_bit)
	{
		config_reg_value |= (1<<10); //11 bit
	}

	switch(Humidity_Resolution_x_bit)
	{
	case Humidity_Resolution_11_bit:
		config_reg_value|= (1<<8);
		break;
	case Humidity_Resolution_8_bit:
		config_reg_value|= (1<<9);
		break;
	}

	data_send[0]= (config_reg_value>>8);
	data_send[1]= (config_reg_value&0x00ff);

	HAL_I2C_Mem_Write(hi2c_x,HDC_1080_ADD<<1,Configuration_register_add,I2C_MEMADD_SIZE_8BIT,data_send,2,1000);
}


uint8_t hdc1080_start_measurement(I2C_HandleTypeDef* hi2c_x,float* temperature, uint8_t* humidity)
{
	uint8_t receive_data[4];
	uint16_t temp_x,humi_x;
	uint8_t send_data = Temperature_register_add;

	HAL_I2C_Master_Transmit(hi2c_x,HDC_1080_ADD<<1,&send_data,1,1000);

	/* Delay here 15ms for conversion compelete.
	 * Note: datasheet say maximum is 7ms, but when delay=7ms, the read value is not correct
	 */
	HAL_Delay(15);

	/* Read temperature and humidity */
	HAL_I2C_Master_Receive(hi2c_x,HDC_1080_ADD<<1,receive_data,4,1000);

	temp_x =((receive_data[0]<<8)|receive_data[1]);
	humi_x =((receive_data[2]<<8)|receive_data[3]);

	*temperature=((temp_x/65536.0)*165.0)-40.0;
	*humidity=(uint8_t)((humi_x/65536.0)*100.0);

	return 0;

}

struct TempHum hdc1080_get_temp_hum(I2C_HandleTypeDef* hi2c_x)
{
	uint8_t receive_data[4];
	uint16_t temp_x,humi_x;
	uint8_t send_data = Temperature_register_add;

	HAL_I2C_Master_Transmit(hi2c_x,HDC_1080_ADD<<1,&send_data,1,1000);

	HAL_Delay(15);

	/* Read temperature and humidity */
	HAL_I2C_Master_Receive(hi2c_x,HDC_1080_ADD<<1,receive_data,4,1000);

	temp_x =((receive_data[0]<<8)|receive_data[1]);
	humi_x =((receive_data[2]<<8)|receive_data[3]);

	float temperature=((temp_x/65536.0)*165.0)-40.0;
	uint8_t humidity=(uint8_t)((humi_x/65536.0)*100.0);

	struct TempHum tempHum;
	tempHum.Humidity = humidity;
	tempHum.Temperature = temperature;
	return tempHum;
}

HAL_StatusTypeDef hdc1080_ask_temp_hum(I2C_HandleTypeDef* hi2c_x)
{
	return HAL_I2C_Master_Transmit(hi2c_x,HDC_1080_ADD<<1,Temperature_register_add,1,1000);
}

struct TempHum hdc1080_receive_temp_hum(I2C_HandleTypeDef* hi2c_x)
{
	uint8_t receive_data[4];
	uint16_t temp_x,humi_x;

	HAL_I2C_Master_Receive(hi2c_x,HDC_1080_ADD<<1,receive_data,4,1000);
	temp_x =((receive_data[0]<<8)|receive_data[1]);

	humi_x =((receive_data[2]<<8)|receive_data[3]);

	struct TempHum tempHum;
	tempHum.Humidity = (uint8_t)((humi_x/65536.0)*100.0);
	tempHum.Temperature = ((temp_x/65536.0)*165.0)-40.0;
	return tempHum;
}

